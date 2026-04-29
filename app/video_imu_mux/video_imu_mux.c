/*
 * video_imu_mux.c - H.265 RTSP video and IMU muxer
 *
 * Pipeline: VI (GC4653) -> ISP -> VPSS -> H.265 VENC -> RTSP server
 * Mode: VI_OFFLINE_VPSS_ONLINE + VPSS_INPUT_ISP (VPSS_MODE_SINGLE)
 *
 * Usage: video_imu_mux [enc_width enc_height]
 *   Default encode resolution: 2560x1440 (GC4653 max)
 *   Sensor size: read from /mnt/data/sensor_cfg.ini (GC4653 = 2560x1440)
 *   Stream URL: rtsp://<device-ip>:8554/live
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <sys/stat.h>

#define LOG_MODULE "MUX"
#include "imu.h"
#include "log.h"
#include "sample_comm.h"
#include "rtsp-server.h"
#include "cvi_audio.h"

/* ------------------------------------------------------------------ */
/* Global state                                                        */
/* ------------------------------------------------------------------ */

static volatile int            g_exit_flag = 0;
static SAMPLE_VI_CONFIG_S      g_stViConfig;
static SAMPLE_INI_CFG_S        g_stIniCfg;
static imu_parser_t           *g_imu_parser = NULL;

/* ------------------------------------------------------------------ */
/* IMU batch buffer + pending video frame                             */
/*                                                                    */
/* IMU packets are accumulated in order. Once IMU_BATCH_SIZE packets  */
/* have arrived AND a video frame is cached, the batch is emitted as  */
/* a prefix SEI immediately followed by the video NALs. The batch     */
/* counter is then reset to zero so the next batch starts fresh.      */
/*                                                                    */
/* A single video frame is cached from VENC as soon as it is ready.  */
/* If another frame arrives while the cached frame is still waiting   */
/* for its IMU batch, it is drained to prevent encoder stall.         */
/* ------------------------------------------------------------------ */
#define IMU_BATCH_SIZE 8    /* packets packed into one SEI */
#define IMU_BUF_SIZE   16   /* accumulation limit; reset to 0 when reached */
#define IMU_PKT_WIRE_SIZE 28

/* ------------------------------------------------------------------ */
/* Local video file recording                                         */
/* Raw Annex-B H.265 files, split at VIDEO_FILE_SIZE_LIMIT bytes.     */
/* ------------------------------------------------------------------ */
#define VIDEO_FILE_DIR        "/mnt/data/video"
#define VIDEO_FILE_SIZE_LIMIT (10 * 1024 * 1024)  /* 10 MB */

static FILE  *g_video_file       = NULL;
static size_t g_video_file_bytes = 0;
static int    g_video_file_idx   = 0;

static uint8_t g_imu_buf[IMU_BUF_SIZE][IMU_PKT_WIRE_SIZE];
static int     g_imu_count = 0;

/* Base timestamp subtracted before casting system_time to float32.
 * Set on the first received IMU packet. Keeps the wire value small so
 * float32 precision stays in the microsecond range regardless of how
 * large the absolute system_time has grown. */
static double  g_system_time_base  = 0.0;
static int     g_system_time_set   = 0;

typedef struct {
	int      valid;
	CVI_U64  pts;
	uint8_t *data;     /* flat copy of all pack payloads */
	size_t   data_len;
	struct {
		size_t offset;
		size_t length;
	} packs[64];       /* max packs per frame; 64 is generous */
	CVI_U32  pack_count;
} pending_frame_t;

static pending_frame_t g_pending_frame;

static void build_timu_pkt(uint8_t out[IMU_PKT_WIRE_SIZE], const imu_tilt_t *tilt);
static int  capture_pending_frame(VENC_CHN t_chn);
static void release_pending_frame(void);
static void drain_venc(VENC_CHN t_chn);
static int  try_send_pending_frame(void);
static void video_file_write(const uint8_t *t_data, size_t t_len);
static void video_file_close(void);


static void sig_handle(int signo)
{
	(void)signo;
	signal(SIGINT,  SIG_IGN);
	signal(SIGTERM, SIG_IGN);
	g_exit_flag = 1;
}


/* Accumulate decoded IMU packets into g_imu_buf.
 * Called from imu_parser_feed() on every complete, checksummed packet.
 * Drops the packet silently if the batch is already full (the main loop
 * has not consumed it yet). */
static void on_imu_packet(const imu_packet_t *t_pkt, void *t_user)
{
	imu_tilt_t tilt;

	(void)t_user;

	if (!t_pkt || imu_decode_tilt(t_pkt, &tilt) != 0)
		return;

	if (!g_system_time_set) {
		g_system_time_base = tilt.system_time;
		g_system_time_set  = 1;
	}

	if (g_imu_count >= IMU_BUF_SIZE)
		g_imu_count = 0;

	build_timu_pkt(g_imu_buf[g_imu_count], &tilt);
	g_imu_count++;

	LOGT("imu[%d] ts=%.3f gyro=%.2f %.2f %.2f\n",
	     g_imu_count - 1,
	     (float)tilt.system_time,
	     tilt.gyro[0], tilt.gyro[1], tilt.gyro[2]);
}

static int uart1_init(int *t_uart_fd)
{
	int fd;
	struct termios tty;

	if (!t_uart_fd)
		return -1;

	fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
		return -1;

	if (tcgetattr(fd, &tty) != 0) {
		close(fd);
		return -1;
	}

	cfmakeraw(&tty);
	cfsetispeed(&tty, B460800);
	cfsetospeed(&tty, B460800);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSTOPB;         /* 1 stop bit */
	tty.c_cflag &= ~PARENB;         /* no parity */
	tty.c_cflag &= ~CRTSCTS;        /* no HW flow control */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;             /* 8 data bits */
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* no SW flow control */
	tty.c_iflag |= IGNPAR | IGNBRK; /* silently discard framing/parity errors */
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		close(fd);
		return -1;
	}

	if (!g_imu_parser) {
		g_imu_parser = imu_parser_create(on_imu_packet, NULL);
		if (!g_imu_parser) {
			close(fd);
			return -1;
		}
	}

	*t_uart_fd = fd;
	return 0;
}

static void uart1_deinit(int t_uart_fd)
{
	if (t_uart_fd >= 0)
		close(t_uart_fd);

	if (g_imu_parser) {
		imu_parser_destroy(g_imu_parser);
		g_imu_parser = NULL;
	}
}

static void uart1_handle_rx_tx(int t_uart_fd)
{
	char rx_buf[256];
	ssize_t rx_len;

	if (t_uart_fd < 0)
		return;

	rx_len = read(t_uart_fd, rx_buf, sizeof(rx_buf));
	if (rx_len <= 0) {
		if (rx_len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
			LOGP("uart1 read");
		return;
	}

	// printf("uart1 rx %zd bytes\r\n", rx_len);
	if (g_imu_parser)
		imu_parser_feed(g_imu_parser, (const uint8_t *)rx_buf, (size_t)rx_len);
}

static const uint8_t k_sei_uuid[16] = {
	0xAA, 0xAA, 0xAA, 0xAA, 0xBB, 0xBB, 0xBB, 0xBB,
	0xCC, 0xCC, 0xCC, 0xCC, 0xDD, 0xDD, 0xDD, 0xDD
};

static void build_timu_pkt(uint8_t out[IMU_PKT_WIRE_SIZE], const imu_tilt_t *tilt)
{
	/* TImuData: float ts, x0, y0, z0, x1, y1, z1 — 7 x 4 = 28 bytes
	 *
	 * ts is stored as (system_time - base) so the value stays small and
	 * float32 precision remains in the microsecond range. At ~100s relative
	 * time the float32 ULP is ~12 μs; at the raw absolute value (~20000 s)
	 * it would be ~1.9 ms, causing the periodic jump you would otherwise see
	 * every ~14 frames. */
	float v[7];
	v[0] = (float)(tilt->system_time - g_system_time_base);
	v[1] = tilt->gyro[0];
	v[2] = tilt->gyro[1];
	v[3] = tilt->gyro[2];
	v[4] = tilt->has_accel ? tilt->accel[0] : 0.0f;
	v[5] = tilt->has_accel ? tilt->accel[1] : 0.0f;
	v[6] = tilt->has_accel ? tilt->accel[2] : 0.0f;
	memcpy(out, v, 28);
}

/* Build one H.265 prefix SEI NAL (user_data_unregistered) with 8 TImuPkt blobs. */
static size_t build_h265_prefix_sei(uint8_t *dst, size_t dst_size,
	const uint8_t t_imu_buf[IMU_BATCH_SIZE][IMU_PKT_WIRE_SIZE])
{
	uint8_t sei_rbsp[1 + 4 + 16 + 1 + (IMU_BATCH_SIZE * IMU_PKT_WIRE_SIZE) + 1];
	size_t rbsp_len = 0;
	size_t out = 0;
	size_t i;
	int zero_count = 0;
	int payload_size = 16 + 1 + (IMU_BATCH_SIZE * IMU_PKT_WIRE_SIZE);
	int remain = payload_size;

	if (!dst || !t_imu_buf)
		return 0;

	/* SEI payload_type=5 (user_data_unregistered). */
	sei_rbsp[rbsp_len++] = 5;
	while (remain >= 0xFF) {
		sei_rbsp[rbsp_len++] = 0xFF;
		remain -= 0xFF;
	}
	sei_rbsp[rbsp_len++] = (uint8_t)remain;

	memcpy(&sei_rbsp[rbsp_len], k_sei_uuid, sizeof(k_sei_uuid));
	rbsp_len += sizeof(k_sei_uuid);
	sei_rbsp[rbsp_len++] = 2; /* userdata0: payload version/type */
	memcpy(&sei_rbsp[rbsp_len], &t_imu_buf[0][0], IMU_BATCH_SIZE * IMU_PKT_WIRE_SIZE);
	rbsp_len += IMU_BATCH_SIZE * IMU_PKT_WIRE_SIZE;
	sei_rbsp[rbsp_len++] = 0x80; /* rbsp_trailing_bits */

	if (dst_size < 6)
		return 0;

	/* Annex-B start code + prefix SEI NAL header (nal_unit_type=39). */
	dst[out++] = 0x00;
	dst[out++] = 0x00;
	dst[out++] = 0x00;
	dst[out++] = 0x01;
	dst[out++] = 39 << 1;
	dst[out++] = 0x01;

	/* Emulation prevention for RBSP bytes only. */
	for (i = 0; i < rbsp_len; i++) {
		uint8_t b = sei_rbsp[i];

		if (zero_count >= 2 && b <= 0x03) {
			if (out >= dst_size)
				return 0;
			dst[out++] = 0x03;
			zero_count = 0;
		}

		if (out >= dst_size)
			return 0;
		dst[out++] = b;

		if (b == 0x00)
			zero_count++;
		else
			zero_count = 0;
	}

	return out;
}

/* Capture one VENC frame into g_pending_frame (heap copy).
 * The encoder stream is released immediately after the copy so the
 * encoder buffer is never held across loop iterations.
 * Returns 0 on success, -1 if no frame was available or on error. */
static int capture_pending_frame(VENC_CHN t_chn)
{
	VENC_CHN_STATUS_S stStat   = {};
	VENC_STREAM_S     stStream = {};
	size_t            total    = 0;
	size_t            offset   = 0;
	CVI_U32           i;

	if (g_pending_frame.valid)
		return 0;

	if (CVI_VENC_QueryStatus(t_chn, &stStat) != CVI_SUCCESS ||
	    stStat.u32CurPacks == 0)
		return -1;

	stStream.pstPack = (VENC_PACK_S *)malloc(
	    sizeof(VENC_PACK_S) * stStat.u32CurPacks);
	if (!stStream.pstPack)
		return -1;

	if (CVI_VENC_GetStream(t_chn, &stStream, 0) != CVI_SUCCESS) {
		free(stStream.pstPack);
		return -1;
	}

	if (stStream.u32PackCount > 64) {
		LOGW("pack_count %u > 64, clamping\n", stStream.u32PackCount);
		stStream.u32PackCount = 64;
	}

	for (i = 0; i < stStream.u32PackCount; i++) {
		VENC_PACK_S *p = &stStream.pstPack[i];
		size_t len = (p->u32Len > p->u32Offset)
		             ? (size_t)(p->u32Len - p->u32Offset) : 0;
		g_pending_frame.packs[i].offset = total;
		g_pending_frame.packs[i].length = len;
		total += len;
	}

	g_pending_frame.data = (uint8_t *)malloc(total);
	if (!g_pending_frame.data) {
		CVI_VENC_ReleaseStream(t_chn, &stStream);
		free(stStream.pstPack);
		return -1;
	}

	for (i = 0; i < stStream.u32PackCount; i++) {
		VENC_PACK_S *p   = &stStream.pstPack[i];
		size_t       len = g_pending_frame.packs[i].length;
		if (len > 0) {
			memcpy(g_pending_frame.data + offset,
			       p->pu8Addr + p->u32Offset, len);
			offset += len;
		}
	}

	g_pending_frame.valid      = 1;
	g_pending_frame.pts        = stStream.pstPack[0].u64PTS;
	g_pending_frame.pack_count = stStream.u32PackCount;
	g_pending_frame.data_len   = total;

	CVI_VENC_ReleaseStream(t_chn, &stStream);
	free(stStream.pstPack);
	return 0;
}

static void release_pending_frame(void)
{
	free(g_pending_frame.data);
	memset(&g_pending_frame, 0, sizeof(g_pending_frame));
}

/* Drain one VENC frame without sending to prevent encoder stall. */
static void drain_venc(VENC_CHN t_chn)
{
	VENC_CHN_STATUS_S stStat   = {};
	VENC_STREAM_S     stStream = {};

	if (CVI_VENC_QueryStatus(t_chn, &stStat) != CVI_SUCCESS ||
	    stStat.u32CurPacks == 0)
		return;

	stStream.pstPack = (VENC_PACK_S *)malloc(
	    sizeof(VENC_PACK_S) * stStat.u32CurPacks);
	if (!stStream.pstPack)
		return;

	if (CVI_VENC_GetStream(t_chn, &stStream, 0) == CVI_SUCCESS)
		CVI_VENC_ReleaseStream(t_chn, &stStream);

	free(stStream.pstPack);
}

/* Send the cached frame with a SEI prefix built from the current IMU batch.
 * Called only when both conditions are met: IMU batch is full AND a frame
 * is cached. Resets the IMU counter and releases the frame on success.
 * Returns 1 if sent, 0 if conditions not met. */
static int try_send_pending_frame(void)
{
	uint8_t sei_nal[1024];
	size_t  sei_len;
	CVI_U32 i;

	if (!g_pending_frame.valid || g_imu_count < IMU_BATCH_SIZE)
		return 0;

	sei_len = build_h265_prefix_sei(sei_nal, sizeof(sei_nal), g_imu_buf);
	if (sei_len == 0) {
		LOGW("build_h265_prefix_sei failed, dropping pending frame\n");
		release_pending_frame();
		g_imu_count = 0;
		return 0;
	}

	g_imu_count = 0;

	rtsp_send_h265_data(sei_nal, sei_len);
	video_file_write(sei_nal, sei_len);

	for (i = 0; i < g_pending_frame.pack_count; i++) {
		size_t len = g_pending_frame.packs[i].length;
		if (len > 0) {
			uint8_t *ptr =
			    g_pending_frame.data + g_pending_frame.packs[i].offset;
			rtsp_send_h265_data(ptr, len);
			video_file_write(ptr, len);
		}
	}

	LOGT("sent frame pts=%llu with %d imu pkts\n",
	     (unsigned long long)g_pending_frame.pts, IMU_BATCH_SIZE);

	release_pending_frame();
	return 1;
}

/* Open the next numbered file for recording. */
static void video_file_open_next(void)
{
	char path[128];

	mkdir(VIDEO_FILE_DIR, 0755);
	snprintf(path, sizeof(path), "%s/video_%04d.h265",
	         VIDEO_FILE_DIR, g_video_file_idx++);
	g_video_file = fopen(path, "wb");
	if (!g_video_file) {
		LOGW("cannot open video file %s: %s\n", path, strerror(errno));
		return;
	}
	g_video_file_bytes = 0;
	LOGI("recording to %s\n", path);
}

/* Write data to the current recording file, opening or rotating as needed. */
static void video_file_write(const uint8_t *t_data, size_t t_len)
{
	if (!t_data || t_len == 0)
		return;

	if (!g_video_file)
		video_file_open_next();

	if (!g_video_file)
		return;

	fwrite(t_data, 1, t_len, g_video_file);
	g_video_file_bytes += t_len;

	if (g_video_file_bytes >= VIDEO_FILE_SIZE_LIMIT) {
		fclose(g_video_file);
		g_video_file = NULL;
		LOGI("video file closed at %zu bytes, starting next\n",
		     g_video_file_bytes);
	}
}

static void video_file_close(void)
{
	if (g_video_file) {
		fclose(g_video_file);
		g_video_file = NULL;
		LOGI("video file closed (%zu bytes)\n", g_video_file_bytes);
	}
}


/* ------------------------------------------------------------------ */
/* Audio recording (AI -> AENC G711A, saved alongside video files)    */
/* ------------------------------------------------------------------ */
#define AUDIO_SAMPLE_RATE  8000   /* Hz */
#define AUDIO_PT_NUM       320    /* samples per frame = 40 ms at 8 kHz */

static FILE           *g_audio_file  = NULL;
static pthread_t       g_audio_thread;
static volatile int    g_audio_stop  = 0;

static void audio_file_open(int idx)
{
	char path[128];

	snprintf(path, sizeof(path), "%s/audio_%04d.g711a",
	         VIDEO_FILE_DIR, idx);
	g_audio_file = fopen(path, "wb");
	if (!g_audio_file)
		LOGW("cannot open audio file %s: %s\n", path, strerror(errno));
	else
		LOGI("audio recording to %s\n", path);
}

static void audio_file_close(void)
{
	if (g_audio_file) {
		fclose(g_audio_file);
		g_audio_file = NULL;
	}
}

/* Audio capture thread: drains AENC stream and writes to file.
 * File rotation is tied to video file rotation: when g_video_file_idx
 * increments (a new video_NNNN.h265 was opened), a matching
 * audio_NNNN.g711a is opened. */
static void *audio_capture_thread(void *arg)
{
	int last_video_idx = 0;

	(void)arg;

	while (!g_audio_stop) {
		int cur = g_video_file_idx;

		if (cur != last_video_idx && cur > 0) {
			audio_file_close();
			audio_file_open(cur - 1);
			last_video_idx = cur;
		}

		AUDIO_STREAM_S stStream = {};
		CVI_S32 ret = CVI_AENC_GetStream(0, &stStream, 40);
		if (ret == CVI_SUCCESS) {
			if (g_audio_file && stStream.pStream && stStream.u32Len > 0)
				fwrite(stStream.pStream, 1,
				       (size_t)stStream.u32Len, g_audio_file);
			CVI_AENC_ReleaseStream(0, &stStream);
		}
	}

	audio_file_close();
	return NULL;
}

static int sys_audio_init(void)
{
	AIO_ATTR_S       stAiAttr   = {};
	AENC_ATTR_G711_S stG711Attr = { .resv = 0 };
	AENC_CHN_ATTR_S  stAencAttr = {};
	MMF_CHN_S        stSrc, stDst;
	CVI_S32          s32Ret;

	stAiAttr.enSamplerate   = AUDIO_SAMPLE_RATE_8000;
	stAiAttr.u32ChnCnt      = 2;   /* inner codec requires 2 channels */
	stAiAttr.enSoundmode    = AUDIO_SOUND_MODE_MONO;
	stAiAttr.enBitwidth     = AUDIO_BIT_WIDTH_16;
	stAiAttr.enWorkmode     = AIO_MODE_I2S_MASTER;
	stAiAttr.u32EXFlag      = 0;
	stAiAttr.u32FrmNum      = 10;
	stAiAttr.u32PtNumPerFrm = AUDIO_PT_NUM;
	stAiAttr.u32ClkSel      = 0;
	stAiAttr.enI2sType      = AIO_I2STYPE_INNERCODEC;

	s32Ret = CVI_AUDIO_INIT();
	if (s32Ret != CVI_SUCCESS) {
		LOGE("CVI_AUDIO_INIT failed 0x%x\n", s32Ret);
		return -1;
	}

	s32Ret = CVI_AI_SetPubAttr(0, &stAiAttr);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("CVI_AI_SetPubAttr failed 0x%x\n", s32Ret);
		goto err_deinit;
	}

	s32Ret = CVI_AI_Enable(0);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("CVI_AI_Enable failed 0x%x\n", s32Ret);
		goto err_deinit;
	}

	s32Ret = CVI_AI_EnableChn(0, 0);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("CVI_AI_EnableChn failed 0x%x\n", s32Ret);
		goto err_disable_dev;
	}

	stAencAttr.enType         = PT_G711A;
	stAencAttr.u32PtNumPerFrm = AUDIO_PT_NUM;
	stAencAttr.u32BufSize     = 30;
	stAencAttr.pValue         = &stG711Attr;
	stAencAttr.bFileDbgMode   = CVI_FALSE;

	s32Ret = CVI_AENC_CreateChn(0, &stAencAttr);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("CVI_AENC_CreateChn failed 0x%x\n", s32Ret);
		goto err_disable_chn;
	}

	stSrc.enModId  = CVI_ID_AI;
	stSrc.s32DevId = 0;
	stSrc.s32ChnId = 0;
	stDst.enModId  = CVI_ID_AENC;
	stDst.s32DevId = 0;
	stDst.s32ChnId = 0;
	CVI_AUD_SYS_Bind(&stSrc, &stDst);

	g_audio_stop = 0;
	if (pthread_create(&g_audio_thread, NULL, audio_capture_thread, NULL) != 0) {
		LOGE("audio thread create failed\n");
		CVI_AUD_SYS_UnBind(&stSrc, &stDst);
		goto err_aenc;
	}

	LOGI("audio init OK (G711A %d Hz mono)\n", AUDIO_SAMPLE_RATE);
	return 0;

err_aenc:
	CVI_AENC_DestroyChn(0);
err_disable_chn:
	CVI_AI_DisableChn(0, 0);
err_disable_dev:
	CVI_AI_Disable(0);
err_deinit:
	CVI_AUDIO_DEINIT();
	return -1;
}

static void sys_audio_deinit(void)
{
	MMF_CHN_S stSrc, stDst;

	g_audio_stop = 1;
	pthread_join(g_audio_thread, NULL);

	stSrc.enModId  = CVI_ID_AI;
	stSrc.s32DevId = 0;
	stSrc.s32ChnId = 0;
	stDst.enModId  = CVI_ID_AENC;
	stDst.s32DevId = 0;
	stDst.s32ChnId = 0;
	CVI_AUD_SYS_UnBind(&stSrc, &stDst);

	CVI_AENC_DestroyChn(0);
	CVI_AI_DisableChn(0, 0);
	CVI_AI_Disable(0);
	CVI_AUDIO_DEINIT();
}

/* ------------------------------------------------------------------ */
/* VI + VPSS initialisation                                            */
/* ------------------------------------------------------------------ */

static int sys_vi_init(int enc_w, int enc_h)
{
	CVI_S32            s32Ret;
	SAMPLE_INI_CFG_S   stIniCfg  = {};
	SAMPLE_VI_CONFIG_S stViConfig = {};
	PIC_SIZE_E         enPicSize;
	SIZE_S             stSensorSize;   /* native sensor frame size */
	SIZE_S             stEncSize;      /* desired VPSS output / encode size */
	LOG_LEVEL_CONF_S   log_conf;

	/* Silence middleware logs to INFO level */
	log_conf.enModId  = CVI_ID_LOG;
	log_conf.s32Level = CVI_DBG_INFO;
	CVI_LOG_SetLevelConf(&log_conf);

	/* Step 1: parse sensor_cfg.ini */
	if (SAMPLE_COMM_VI_ParseIni(&stIniCfg))
		LOGI("sensor_cfg.ini parsed OK\n");

	CVI_VI_SetDevNum(stIniCfg.devNum);

	/* Step 2: build VI config from ini */
	s32Ret = SAMPLE_COMM_VI_IniToViCfg(&stIniCfg, &stViConfig);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("IniToViCfg failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	/* Step 3: get native sensor frame size */
	s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stIniCfg.enSnsType[0], &enPicSize);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("GetSizeBySensor failed 0x%x\n", s32Ret);
		return s32Ret;
	}
	s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSensorSize);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("GetPicSize failed 0x%x\n", s32Ret);
		return s32Ret;
	}
	LOGI("sensor size: %ux%u\n",
	     stSensorSize.u32Width, stSensorSize.u32Height);

	/* Step 4: init VB pools sized for sensor frame */
	s32Ret = SAMPLE_PLAT_SYS_INIT(stSensorSize);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("SYS_INIT failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	/* Step 5: select VI_OFFLINE_VPSS_ONLINE + VPSS_MODE_SINGLE + ISP input */
	{
		VI_VPSS_MODE_S stVIVPSSMode = {};
		VPSS_MODE_S    stVPSSMode   = {};

		stVIVPSSMode.aenMode[0] = VI_OFFLINE_VPSS_ONLINE;
		stVIVPSSMode.aenMode[1] = VI_OFFLINE_VPSS_ONLINE;

		stVPSSMode.enMode       = VPSS_MODE_SINGLE;
		stVPSSMode.aenInput[0]  = VPSS_INPUT_ISP;
		stVPSSMode.ViPipe[0]    = 0;

		s32Ret = CVI_SYS_SetVIVPSSMode(&stVIVPSSMode);
		if (s32Ret != CVI_SUCCESS) {
			LOGE("SetVIVPSSMode failed 0x%x\n", s32Ret);
			return s32Ret;
		}
		s32Ret = CVI_SYS_SetVPSSModeEx(&stVPSSMode);
		if (s32Ret != CVI_SUCCESS) {
			LOGE("SetVPSSModeEx failed 0x%x\n", s32Ret);
			return s32Ret;
		}
	}

	/* Step 6: start sensor, MIPI, ISP, VI pipe & channels */
	s32Ret = SAMPLE_PLAT_VI_INIT(&stViConfig);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("PLAT_VI_INIT failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	/* Save for teardown */
	memcpy(&g_stViConfig, &stViConfig, sizeof(g_stViConfig));
	memcpy(&g_stIniCfg,   &stIniCfg,   sizeof(g_stIniCfg));

	/* Step 7: create and start VPSS group 0
	 *   - group input  = sensor frame (NV21)
	 *   - channel 0 output = desired encode size (NV21) */
	stEncSize.u32Width  = (CVI_U32)enc_w;
	stEncSize.u32Height = (CVI_U32)enc_h;

	{
		VPSS_GRP_ATTR_S  stGrpAttr  = {};
		VPSS_CHN_ATTR_S  stChnAttr[VPSS_MAX_PHY_CHN_NUM] = {};
		CVI_BOOL         abChnEnable[VPSS_MAX_PHY_CHN_NUM] = { CVI_FALSE };

		stGrpAttr.stFrameRate.s32SrcFrameRate = -1;
		stGrpAttr.stFrameRate.s32DstFrameRate = -1;
		stGrpAttr.enPixelFormat               = PIXEL_FORMAT_NV21;
		stGrpAttr.u32MaxW                     = stSensorSize.u32Width;
		stGrpAttr.u32MaxH                     = stSensorSize.u32Height;
		stGrpAttr.u8VpssDev                   = 0;

		stChnAttr[0].u32Width                    = stEncSize.u32Width;
		stChnAttr[0].u32Height                   = stEncSize.u32Height;
		stChnAttr[0].enVideoFormat               = VIDEO_FORMAT_LINEAR;
		stChnAttr[0].enPixelFormat               = PIXEL_FORMAT_NV21;
		stChnAttr[0].stFrameRate.s32SrcFrameRate = 30;
		stChnAttr[0].stFrameRate.s32DstFrameRate = 30;
		stChnAttr[0].u32Depth                    = 0;
		stChnAttr[0].bMirror                     = CVI_FALSE;
		stChnAttr[0].bFlip                       = CVI_TRUE;
		stChnAttr[0].stAspectRatio.enMode        = ASPECT_RATIO_NONE;
		stChnAttr[0].stNormalize.bEnable         = CVI_FALSE;

		abChnEnable[0] = CVI_TRUE;

		/* Defensive cleanup in case a previous run exited without destroying group 0 */
		CVI_VPSS_StopGrp(0);
		CVI_VPSS_DestroyGrp(0);

		s32Ret = SAMPLE_COMM_VPSS_Init(0, abChnEnable, &stGrpAttr, stChnAttr);
		if (s32Ret != CVI_SUCCESS) {
			LOGE("VPSS_Init failed 0x%x\n", s32Ret);
			return s32Ret;
		}
		s32Ret = SAMPLE_COMM_VPSS_Start(0, abChnEnable, &stGrpAttr, stChnAttr);
		if (s32Ret != CVI_SUCCESS) {
			LOGE("VPSS_Start failed 0x%x\n", s32Ret);
			return s32Ret;
		}
	}

	/* Step 8: bind VI pipe 0 channel 0 -> VPSS group 0 */
	s32Ret = SAMPLE_COMM_VI_Bind_VPSS(0, 0, 0);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("VI_Bind_VPSS failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	LOGI("VI+VPSS init OK, encode %ux%u\n",
	     stEncSize.u32Width, stEncSize.u32Height);
	return CVI_SUCCESS;
}

static void sys_vi_deinit(void)
{
	CVI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = { CVI_TRUE };

	/* Unbind VI->VPSS first, then stop ISP so no frames reach VPSS during teardown */
	SAMPLE_COMM_VI_UnBind_VPSS(0, 0, 0);
	SAMPLE_COMM_VI_DestroyIsp(&g_stViConfig);
	SAMPLE_COMM_VI_DestroyVi(&g_stViConfig);
	SAMPLE_COMM_VPSS_Stop(0, abChnEnable);
	SAMPLE_COMM_SYS_Exit();
}

/* ------------------------------------------------------------------ */
/* H.265 VENC initialisation                                          */
/* ------------------------------------------------------------------ */

static int sys_venc_h265_init(int enc_w, int enc_h)
{
	CVI_S32          s32Ret;
	VENC_CHN_ATTR_S  stChnAttr = {};
	VENC_RECV_PIC_PARAM_S stRecvParam = {};

	stChnAttr.stVencAttr.enType         = PT_H265;
	stChnAttr.stVencAttr.u32MaxPicWidth  = (CVI_U32)enc_w;
	stChnAttr.stVencAttr.u32MaxPicHeight = (CVI_U32)enc_h;
	stChnAttr.stVencAttr.u32BufSize      = (CVI_U32)(enc_w * enc_h);
	stChnAttr.stVencAttr.u32Profile      = 0;   /* Main profile */
	stChnAttr.stVencAttr.bByFrame        = CVI_TRUE;
	stChnAttr.stVencAttr.u32PicWidth     = (CVI_U32)enc_w;
	stChnAttr.stVencAttr.u32PicHeight    = (CVI_U32)enc_h;
	stChnAttr.stVencAttr.bSingleCore     = CVI_FALSE;
	stChnAttr.stVencAttr.bEsBufQueueEn   = CVI_TRUE;
	stChnAttr.stVencAttr.bIsoSendFrmEn   = CVI_TRUE;

	stChnAttr.stRcAttr.enRcMode                  = VENC_RC_MODE_H265CBR;
	stChnAttr.stRcAttr.stH265Cbr.u32Gop          = 30;
	stChnAttr.stRcAttr.stH265Cbr.u32StatTime     = 1;
	stChnAttr.stRcAttr.stH265Cbr.u32SrcFrameRate = 30;
	stChnAttr.stRcAttr.stH265Cbr.fr32DstFrameRate = 30;
	stChnAttr.stRcAttr.stH265Cbr.u32BitRate      = 8192;  /* 8 Mbps */
	stChnAttr.stRcAttr.stH265Cbr.bVariFpsEn      = CVI_FALSE;

	stChnAttr.stGopAttr.enGopMode            = VENC_GOPMODE_NORMALP;
	stChnAttr.stGopAttr.stNormalP.s32IPQpDelta = 2;

	s32Ret = CVI_VENC_CreateChn(0, &stChnAttr);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("CVI_VENC_CreateChn failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	/* Bind VPSS group 0 channel 0 -> VENC channel 0 */
	s32Ret = SAMPLE_COMM_VPSS_Bind_VENC(0, 0, 0);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("VPSS_Bind_VENC failed 0x%x\n", s32Ret);
		CVI_VENC_DestroyChn(0);
		return s32Ret;
	}

	stRecvParam.s32RecvPicNum = -1;   /* encode indefinitely */
	s32Ret = CVI_VENC_StartRecvFrame(0, &stRecvParam);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("StartRecvFrame failed 0x%x\n", s32Ret);
		SAMPLE_COMM_VPSS_UnBind_VENC(0, 0, 0);
		CVI_VENC_DestroyChn(0);
		return s32Ret;
	}

	LOGI("VENC H265 init OK\n");
	return CVI_SUCCESS;
}

static void sys_venc_deinit(void)
{
	SAMPLE_COMM_VPSS_UnBind_VENC(0, 0, 0);
	CVI_VENC_StopRecvFrame(0);
	CVI_VENC_ResetChn(0);
	CVI_VENC_DestroyChn(0);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
	int enc_w = 2560;
	int enc_h = 1440;
	int ret   = 0;

	if (argc == 3) {
		enc_w = atoi(argv[1]);
		enc_h = atoi(argv[2]);
		if (enc_w <= 0 || enc_h <= 0) {
			LOGE("Usage: %s [enc_width enc_height]\n", argv[0]);
			return 1;
		}
	} else if (argc != 1) {
		LOGE("Usage: %s [enc_width enc_height]\n", argv[0]);
		return 1;
	}

	signal(SIGINT,  sig_handle);
	signal(SIGTERM, sig_handle);
	signal(SIGPIPE, SIG_IGN);

	LOGI("starting, encode %dx%d H265 (log level=%d)\n", enc_w, enc_h, LOG_LEVEL);

	/* 1. Start RTSP server */
	if (rtsp_server_init(NULL, 8554) != 0) {
		LOGE("rtsp_server_init failed\n");
		return 1;
	}
	if (rtsp_server_start() != 0) {
		LOGE("rtsp_server_start failed\n");
		rtsp_server_deinit();
		return 1;
	}
	LOGI("RTSP URL: rtsp://%s:%d/live\n",
	     rtsp_get_server_ip(), rtsp_get_server_port());

	/* 2. Init VI + VPSS */
	if (sys_vi_init(enc_w, enc_h) != 0) {
		LOGE("sys_vi_init failed\n");
		ret = 1;
		goto err_rtsp;
	}

	/* 3. Init VENC H265 */
	if (sys_venc_h265_init(enc_w, enc_h) != 0) {
		LOGE("sys_venc_h265_init failed\n");
		ret = 1;
		goto err_vi;
	}

	/* 4. Init audio (non-fatal: continue without audio if unavailable) */
	if (sys_audio_init() != 0)
		LOGW("audio init failed, recording without audio\n");

	/* 5. Encode → stream loop */
	{
		VENC_CHN       VencChn  = 0;
		CVI_S32        venc_fd  = CVI_VENC_GetFd(VencChn);
		int            uart_fd  = -1;
		uint64_t       frame_count = 0;

		if (venc_fd <= 0) {
			LOGE("CVI_VENC_GetFd failed %d\n", venc_fd);
			ret = 1;
			goto err_venc;
		}

		if (uart1_init(&uart_fd) == 0)
			LOGI("uart1 ready: /dev/ttyS1 460800 8N1 (A18/A19)\n");
		else
			LOGW("uart1 init failed, continue without uart\n");

		LOGI("entering encode/stream loop (fd=%d)\n", venc_fd);

		/*
		 * Main loop:
		 *   - When venc_fd is readable, capture the encoded frame into a
		 *     heap buffer (if no frame is cached yet) or drain it (if one
		 *     is already waiting) to prevent the encoder ring from stalling.
		 *   - IMU packets are accumulated in g_imu_buf as they arrive.
		 *   - After each select(), try_send_pending_frame() checks whether
		 *     both the cached frame and a full IMU batch are ready; if so,
		 *     it sends SEI + video NALs and resets for the next pair.
		 */
		while (!g_exit_flag) {
			struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 }; /* 50 ms */
			fd_set read_fds;
			int max_fd, sel;

			FD_ZERO(&read_fds);
			FD_SET(venc_fd, &read_fds);
			max_fd = venc_fd;
			if (uart_fd >= 0) {
				FD_SET(uart_fd, &read_fds);
				if (uart_fd > max_fd)
					max_fd = uart_fd;
			}

			sel = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
			if (sel < 0) {
				if (g_exit_flag) break;
				LOGP("select");
				break;
			}

			if (uart_fd >= 0 && FD_ISSET(uart_fd, &read_fds))
				uart1_handle_rx_tx(uart_fd);

			if (FD_ISSET(venc_fd, &read_fds)) {
				if (!g_pending_frame.valid)
					capture_pending_frame(VencChn);
				else
					drain_venc(VencChn);
			}

			if (try_send_pending_frame()) {
				frame_count++;
				if (frame_count == 1)
					LOGI("first frame sent\n");
				LOGI_RL(5000000ULL, "sent %llu frames\n",
				        (unsigned long long)frame_count);
			}
		}

		LOGI("shutting down after %llu frames\n",
		     (unsigned long long)frame_count);

		release_pending_frame();
		video_file_close();
		uart1_deinit(uart_fd);
	}

	sys_audio_deinit();
err_venc:
	sys_venc_deinit();
err_vi:
	sys_vi_deinit();
err_rtsp:
	rtsp_server_stop();
	rtsp_server_deinit();
	return ret;
}
