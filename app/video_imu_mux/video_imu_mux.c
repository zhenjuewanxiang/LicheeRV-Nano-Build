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
#include <sys/mman.h>
#include <pthread.h>

#define LOG_MODULE "MUX"
#include "imu.h"
#include "log.h"
#include "sample_comm.h"
#include "rtsp-server.h"

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

/* ------------------------------------------------------------------ */
/* UVC gadget — MJPEG channel                                         */
/*                                                                    */
/* A second VPSS+VENC channel encodes at UVC_ENC_W x UVC_ENC_H MJPEG.*/
/* Frames are pushed into POSIX shared memory that the pre-built      */
/* uvc-gadget-server.elf reads and forwards to the USB host via the   */
/* Linux UVC gadget driver.                                           */
/*                                                                    */
/* Shared memory layout (must match uvc-gadget-server exactly):       */
/*   byte 0         current_buffer: bit7=MJPEG flag, bit0=buf index  */
/*   byte 1         consumer_reading: 1 while gadget server is reading*/
/*   bytes 2..2+N-1 buffer[0]: uint32 JPEG size + JPEG payload        */
/*   bytes 2+N..end buffer[1]: same layout                            */
/* ------------------------------------------------------------------ */
#define UVC_ENC_W         1280
#define UVC_ENC_H          720
#define UVC_MJPEG_CHN        1   /* VENC channel index */
#define UVC_VPSS_CHN         1   /* VPSS channel index */
#define UVC_SHM_NAME   "/uvc_shared_mem_yuyv"
#define UVC_MAX_FRAME_SIZE (2560 * 1440 * 3 / 2)  /* must match gadget server */
#define UVC_SHM_SIZE       (2 + 2 * (size_t)UVC_MAX_FRAME_SIZE)
#define UVC_MAX_JPEG_SIZE  (UVC_ENC_W * UVC_ENC_H * 2)

typedef struct {
	uint8_t current_buffer;
	uint8_t consumer_reading;
	uint8_t buffer[2][UVC_MAX_FRAME_SIZE];
} uvc_shared_mem_t;

static int               g_uvc_shm_fd = -1;
static uvc_shared_mem_t *g_uvc_shm    = NULL;

static int  sys_venc_mjpeg_init(void);
static void sys_venc_mjpeg_deinit(void);
static int  uvc_shm_init(void);
static void uvc_shm_write_mjpeg(const uint8_t *t_jpeg, uint32_t t_len);
static void uvc_shm_deinit(void);
static void *mjpeg_thread_fn(void *arg);


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

	for (i = 0; i < g_pending_frame.pack_count; i++) {
		size_t len = g_pending_frame.packs[i].length;
		if (len > 0)
			rtsp_send_h265_data(
			    g_pending_frame.data + g_pending_frame.packs[i].offset,
			    len);
	}

	LOGT("sent frame pts=%llu with %d imu pkts\n",
	     (unsigned long long)g_pending_frame.pts, IMU_BATCH_SIZE);

	release_pending_frame();
	return 1;
}

/* ------------------------------------------------------------------ */
/* UVC shared memory + MJPEG frame push                               */
/* ------------------------------------------------------------------ */

static int uvc_shm_init(void)
{
	g_uvc_shm_fd = shm_open(UVC_SHM_NAME,
	                        O_CREAT | O_RDWR,
	                        S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
	if (g_uvc_shm_fd < 0) {
		LOGW("uvc shm_open failed: %s\n", strerror(errno));
		return -1;
	}

	if (ftruncate(g_uvc_shm_fd, (off_t)UVC_SHM_SIZE) < 0) {
		LOGW("uvc ftruncate failed: %s\n", strerror(errno));
		close(g_uvc_shm_fd);
		g_uvc_shm_fd = -1;
		return -1;
	}

	g_uvc_shm = (uvc_shared_mem_t *)mmap(NULL, UVC_SHM_SIZE,
	                PROT_READ | PROT_WRITE, MAP_SHARED, g_uvc_shm_fd, 0);
	if (g_uvc_shm == MAP_FAILED) {
		LOGW("uvc mmap failed: %s\n", strerror(errno));
		close(g_uvc_shm_fd);
		g_uvc_shm_fd = -1;
		g_uvc_shm    = NULL;
		return -1;
	}

	LOGI("UVC shared memory ready (%s, %zu bytes)\n",
	     UVC_SHM_NAME, UVC_SHM_SIZE);
	return 0;
}

/* Write one MJPEG frame into the inactive double buffer and publish it.
 * Format expected by uvc-gadget-server: first 4 bytes = uint32 JPEG size,
 * followed by the JPEG payload bytes.  The high bit of current_buffer
 * signals MJPEG mode to the gadget server. */
static void uvc_shm_write_mjpeg(const uint8_t *t_jpeg, uint32_t t_len)
{
	uint8_t read_buf;
	uint8_t write_buf;

	if (!g_uvc_shm || !t_jpeg || t_len == 0)
		return;

	if ((size_t)t_len + 4 > UVC_MAX_FRAME_SIZE) {
		LOGD_RL(1000000ULL, "uvc: MJPEG frame %u bytes exceeds shm slot\n",
		        t_len);
		return;
	}

	read_buf  = __atomic_load_n(&g_uvc_shm->current_buffer,
	                             __ATOMIC_SEQ_CST) & 0x0f;
	write_buf = read_buf ^ 1;

	*(uint32_t *)g_uvc_shm->buffer[write_buf] = t_len;
	memcpy(g_uvc_shm->buffer[write_buf] + 4, t_jpeg, t_len);

	__atomic_store_n(&g_uvc_shm->current_buffer,
	                 (uint8_t)(write_buf | 0x80), __ATOMIC_SEQ_CST);
}

static void uvc_shm_deinit(void)
{
	if (g_uvc_shm && g_uvc_shm != MAP_FAILED) {
		munmap(g_uvc_shm, UVC_SHM_SIZE);
		g_uvc_shm = NULL;
	}
	if (g_uvc_shm_fd >= 0) {
		close(g_uvc_shm_fd);
		g_uvc_shm_fd = -1;
	}
}

/* Dedicated thread: pulls MJPEG frames from VENC and writes to UVC shm.
 * Runs at the VENC output rate (~30 fps) completely independently of the
 * H.265 / RTSP main loop, so UVC frame delivery is never stalled by IMU
 * batching or H.265 select() waits. */
static void *mjpeg_thread_fn(void *arg)
{
	(void)arg;
	static uint8_t  jpeg_buf[UVC_MAX_JPEG_SIZE];

	while (!g_exit_flag) {
		VENC_CHN_STATUS_S stStat   = {};
		VENC_STREAM_S     stStream = {};
		size_t            total    = 0;
		CVI_U32           i;
		static uint64_t   cnt      = 0;

		if (!g_uvc_shm) {
			usleep(10000);
			continue;
		}

		/* Block up to 40 ms for the next encoded MJPEG frame.
		 * This keeps CPU at zero between frames. */
		if (CVI_VENC_QueryStatus(UVC_MJPEG_CHN, &stStat)
		    != CVI_SUCCESS) {
			usleep(5000);
			continue;
		}
		if (stStat.u32CurPacks == 0) {
			usleep(5000);
			continue;
		}

		stStream.pstPack = (VENC_PACK_S *)malloc(
		    sizeof(VENC_PACK_S) * stStat.u32CurPacks);
		if (!stStream.pstPack) {
			usleep(5000);
			continue;
		}

		if (CVI_VENC_GetStream(UVC_MJPEG_CHN, &stStream, 40)
		    != CVI_SUCCESS) {
			free(stStream.pstPack);
			usleep(5000);
			continue;
		}

		for (i = 0; i < stStream.u32PackCount; i++) {
			VENC_PACK_S *p   = &stStream.pstPack[i];
			size_t       len = (p->u32Len > p->u32Offset)
			                   ? (size_t)(p->u32Len -
			                   p->u32Offset) : 0;
			if (total + len > UVC_MAX_JPEG_SIZE) break;
			memcpy(jpeg_buf + total,
			       p->pu8Addr + p->u32Offset, len);
			total += len;
		}

		CVI_VENC_ReleaseStream(UVC_MJPEG_CHN, &stStream);
		free(stStream.pstPack);

		if (total > 0) {
			if (++cnt == 1)
				LOGI("MJPEG thread: first frame %zu bytes "
				     "SOI=%02x%02x\n",
				     total, jpeg_buf[0], jpeg_buf[1]);
			LOGI_RL(5000000ULL,
			        "MJPEG thread: %llu frames, last %u B\n",
			        (unsigned long long)cnt, (unsigned)total);
			uvc_shm_write_mjpeg(jpeg_buf, (uint32_t)total);
		}
	}
	return NULL;
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

		/* Channel 1: UVC/MJPEG output at UVC_ENC_W x UVC_ENC_H */
		stChnAttr[1].u32Width                    = UVC_ENC_W;
		stChnAttr[1].u32Height                   = UVC_ENC_H;
		stChnAttr[1].enVideoFormat               = VIDEO_FORMAT_LINEAR;
		stChnAttr[1].enPixelFormat               = PIXEL_FORMAT_NV21;
		stChnAttr[1].stFrameRate.s32SrcFrameRate = 30;
		stChnAttr[1].stFrameRate.s32DstFrameRate = 30;
		stChnAttr[1].u32Depth                    = 0;
		stChnAttr[1].bMirror                     = CVI_FALSE;
		stChnAttr[1].bFlip                       = CVI_TRUE;
		stChnAttr[1].stAspectRatio.enMode        = ASPECT_RATIO_NONE;
		stChnAttr[1].stNormalize.bEnable         = CVI_FALSE;

		abChnEnable[1] = CVI_TRUE;

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
	CVI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = { CVI_TRUE, CVI_TRUE };

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
/* MJPEG VENC for UVC (channel UVC_MJPEG_CHN, VPSS ch UVC_VPSS_CHN)  */
/* ------------------------------------------------------------------ */
static int sys_venc_mjpeg_init(void)
{
	CVI_S32               s32Ret;
	VENC_CHN_ATTR_S       stChnAttr  = {};
	VENC_MJPEG_FIXQP_S   *pstFixQp;
	VENC_RECV_PIC_PARAM_S stRecvParam = {};

	stChnAttr.stVencAttr.enType         = PT_MJPEG;
	stChnAttr.stVencAttr.u32MaxPicWidth  = UVC_ENC_W;
	stChnAttr.stVencAttr.u32MaxPicHeight = UVC_ENC_H;
	stChnAttr.stVencAttr.u32BufSize      = (CVI_U32)(UVC_ENC_W * UVC_ENC_H * 2);
	stChnAttr.stVencAttr.bByFrame        = CVI_TRUE;
	stChnAttr.stVencAttr.u32PicWidth     = UVC_ENC_W;
	stChnAttr.stVencAttr.u32PicHeight    = UVC_ENC_H;

	stChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
	pstFixQp = &stChnAttr.stRcAttr.stMjpegFixQp;
	pstFixQp->u32Qfactor        = 50;
	pstFixQp->u32SrcFrameRate   = 30;
	pstFixQp->fr32DstFrameRate  = 30;
	pstFixQp->bVariFpsEn        = CVI_FALSE;

	s32Ret = CVI_VENC_CreateChn(UVC_MJPEG_CHN, &stChnAttr);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("MJPEG CVI_VENC_CreateChn failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	s32Ret = SAMPLE_COMM_VPSS_Bind_VENC(0, UVC_VPSS_CHN, UVC_MJPEG_CHN);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("MJPEG VPSS_Bind_VENC failed 0x%x\n", s32Ret);
		CVI_VENC_DestroyChn(UVC_MJPEG_CHN);
		return s32Ret;
	}

	stRecvParam.s32RecvPicNum = -1;
	s32Ret = CVI_VENC_StartRecvFrame(UVC_MJPEG_CHN, &stRecvParam);
	if (s32Ret != CVI_SUCCESS) {
		LOGE("MJPEG StartRecvFrame failed 0x%x\n", s32Ret);
		SAMPLE_COMM_VPSS_UnBind_VENC(0, UVC_VPSS_CHN, UVC_MJPEG_CHN);
		CVI_VENC_DestroyChn(UVC_MJPEG_CHN);
		return s32Ret;
	}

	LOGI("VENC MJPEG init OK (%dx%d qfactor=50)\n", UVC_ENC_W, UVC_ENC_H);
	return CVI_SUCCESS;
}

static void sys_venc_mjpeg_deinit(void)
{
	SAMPLE_COMM_VPSS_UnBind_VENC(0, UVC_VPSS_CHN, UVC_MJPEG_CHN);
	CVI_VENC_StopRecvFrame(UVC_MJPEG_CHN);
	CVI_VENC_ResetChn(UVC_MJPEG_CHN);
	CVI_VENC_DestroyChn(UVC_MJPEG_CHN);
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

	/* 4. Init MJPEG VENC + UVC shared memory (best-effort; failures only
	 *    disable UVC, the H.265/RTSP stream continues unaffected). */
	{
		int uvc_ok = 0;

		if (sys_venc_mjpeg_init() == 0) {
			if (uvc_shm_init() == 0) {
				uvc_ok = 1;
				LOGI("UVC ready: %dx%d MJPEG via %s\n",
				     UVC_ENC_W, UVC_ENC_H, UVC_SHM_NAME);

				/* Pre-fill shm with the first real MJPEG frame so
				 * that Windows Camera gets a valid frame immediately
				 * when it connects. Block up to 500 ms for VENC. */
				{
					VENC_CHN_STATUS_S st     = {};
					VENC_STREAM_S     stream = {};
					int               ok     = 0;
					int               t;

					for (t = 0; t < 50 && !ok; t++) {
						usleep(10000); /* wait 10 ms */
						if (CVI_VENC_QueryStatus(UVC_MJPEG_CHN,
						    &st) != CVI_SUCCESS ||
						    st.u32CurPacks == 0)
							continue;
						stream.pstPack = (VENC_PACK_S *)malloc(
						    sizeof(VENC_PACK_S) *
						    st.u32CurPacks);
						if (!stream.pstPack)
							break;
						if (CVI_VENC_GetStream(
						    UVC_MJPEG_CHN, &stream,
						    0) == CVI_SUCCESS) {
							CVI_U32 j;
							static uint8_t pre_buf[UVC_MAX_JPEG_SIZE];
							size_t pre_len = 0;
							for (j = 0;
							     j < stream.u32PackCount;
							     j++) {
								VENC_PACK_S *p =
								    &stream.pstPack[j];
								size_t l =
								    (p->u32Len > p->u32Offset)
								    ? (size_t)(p->u32Len -
								    p->u32Offset) : 0;
								if (pre_len + l >
								    UVC_MAX_JPEG_SIZE)
									break;
								memcpy(pre_buf + pre_len,
								    p->pu8Addr +
								    p->u32Offset, l);
								pre_len += l;
							}
							CVI_VENC_ReleaseStream(
							    UVC_MJPEG_CHN, &stream);
							if (pre_len > 0) {
								uvc_shm_write_mjpeg(
								    pre_buf,
								    (uint32_t)pre_len);
								LOGI("UVC shm pre-filled: "
								     "%zu bytes  SOI=%02x%02x"
								     " (attempt %d)\n",
								     pre_len,
								     pre_buf[0],
								     pre_buf[1], t+1);
								ok = 1;
							}
						} else {
							/* GetStream failed — do NOT call
							 * ReleaseStream; just free the pack array */
						}
						free(stream.pstPack);
						stream.pstPack = NULL;
					}
					if (!ok)
						LOGW("UVC shm pre-fill timeout\n");
				}
			} else {
				LOGW("UVC shm init failed — UVC disabled\n");
				sys_venc_mjpeg_deinit();
			}
		} else {
			LOGW("MJPEG VENC init failed — UVC disabled\n");
		}

		/* 5. Encode → stream loop */
		{
			VENC_CHN VencChn   = 0;
			CVI_S32  venc_fd   = CVI_VENC_GetFd(VencChn);
			CVI_S32  mjpeg_fd  = uvc_ok
			                     ? CVI_VENC_GetFd(UVC_MJPEG_CHN) : -1;
			pthread_t mjpeg_tid = 0;
			int      uart_fd  = -1;
			uint64_t frame_count = 0;

			if (venc_fd <= 0) {
				LOGE("CVI_VENC_GetFd failed %d\n", venc_fd);
				ret = 1;
				if (uvc_ok) {
					uvc_shm_deinit();
					sys_venc_mjpeg_deinit();
				}
				goto err_venc;
			}

			if (uart1_init(&uart_fd) == 0)
				LOGI("uart1 ready: /dev/ttyS1 460800 8N1 (A18/A19)\n");
			else
				LOGW("uart1 init failed, continue without uart\n");

			/* Start dedicated MJPEG→UVC thread */
			if (uvc_ok) {
				if (pthread_create(&mjpeg_tid, NULL,
				    mjpeg_thread_fn, NULL) != 0) {
					LOGW("MJPEG thread create failed"
					     " — UVC may be choppy\n");
					mjpeg_tid = 0;
				} else {
					LOGI("MJPEG thread started\n");
				}
			}

			LOGI("entering encode/stream loop (h265_fd=%d mjpeg_fd=%d)\n",
			     venc_fd, mjpeg_fd);

			/*
			 * Main loop:
			 *   - H.265 channel (ch0): cache one frame, wait for a full IMU
			 *     batch of IMU_BATCH_SIZE, then send SEI + video together.
			 *   - MJPEG channel (ch1): send every frame immediately to the
			 *     UVC shared memory so uvc-gadget-server can push it to USB.
			 *   - IMU bytes from uart1 are processed as they arrive.
			 */
			while (!g_exit_flag) {
				struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
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
			/* mjpeg_fd is intentionally NOT added to read_fds:
			 * some SDK versions don't wake select() for MJPEG
			 * channels. We poll it unconditionally every iteration
			 * via send_mjpeg_to_uvc() below instead. */
			(void)mjpeg_fd;

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

				/* MJPEG→UVC is handled by mjpeg_thread_fn */

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

			/* Stop MJPEG thread (g_exit_flag already set) */
			if (mjpeg_tid) {
				pthread_join(mjpeg_tid, NULL);
				mjpeg_tid = 0;
			}

			release_pending_frame();
			if (uvc_ok) {
				uvc_shm_deinit();
				sys_venc_mjpeg_deinit();
			}
			uart1_deinit(uart_fd);
		}
	}

err_venc:
	sys_venc_deinit();
err_vi:
	sys_vi_deinit();
err_rtsp:
	rtsp_server_stop();
	rtsp_server_deinit();
	return ret;
}
