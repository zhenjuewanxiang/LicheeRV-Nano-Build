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
/* IMU ring buffer for high-precision frame sync                       */
/* Stores the last IMU_RING_SIZE samples with their CLOCK_MONOTONIC    */
/* capture timestamps so we can interpolate to any frame PTS.          */
/* The loop is single-threaded (select on venc_fd + uart_fd), so no    */
/* mutex is required.                                                  */
/* ------------------------------------------------------------------ */
#define IMU_RING_SIZE  64

typedef struct {
	uint64_t   capture_us; /* CLOCK_MONOTONIC µs when bytes arrived */
	imu_tilt_t tilt;
} imu_sample_t;

static imu_sample_t g_imu_ring[IMU_RING_SIZE];
static int          g_imu_ring_head  = 0; /* next write slot */
static int          g_imu_ring_count = 0; /* valid entries   */

/*
 * Clock-domain calibration: offset_us = CLOCK_MONOTONIC_us - imu_system_time_us
 * Computed once at the first received IMU packet and held constant.
 * Converts any imu_tilt_t.system_time to CLOCK_MONOTONIC µs:
 *   capture_us = (uint64_t)(system_time * 1e6) + g_imu_clk_offset_us
 */
static int64_t  g_imu_clk_offset_us    = 0;
static int      g_imu_clk_offset_valid = 0;

/* Forward declaration — defined in Helpers section below. */
static uint64_t get_time_us(void);

/* Push a new sample into the ring (overwrites oldest when full).
 * capture_us must already be in CLOCK_MONOTONIC µs domain. */
static void imu_ring_push(const imu_tilt_t *tilt, uint64_t capture_us)
{
	g_imu_ring[g_imu_ring_head].capture_us = capture_us;
	g_imu_ring[g_imu_ring_head].tilt       = *tilt;
	g_imu_ring_head = (g_imu_ring_head + 1) % IMU_RING_SIZE;
	if (g_imu_ring_count < IMU_RING_SIZE)
		g_imu_ring_count++;
}

/*
 * Interpolate (or clamp-extrapolate) the IMU ring buffer to target_us.
 *
 * Iterates from oldest to newest entry.  Finds the two adjacent samples
 * that bracket target_us and linearly interpolates between them.
 * If target_us is outside the buffered range the nearest endpoint is used.
 *
 * Returns 0 on success, -1 when the ring is empty.
 * On success *residual_us receives the distance to the nearest sample
 * (0 when perfectly between two samples, positive when extrapolating).
 */
static int imu_ring_interpolate(uint64_t target_us, imu_tilt_t *out,
                                uint32_t *residual_us)
{
	int count = g_imu_ring_count;
	int oldest, i;
	int prev_idx = -1, next_idx = -1;

	if (count == 0)
		return -1;

	/* Oldest slot: if buffer not full it starts at 0; otherwise at head. */
	oldest = (count < IMU_RING_SIZE) ? 0 : g_imu_ring_head;

	/* Find bracketing pair. */
	for (i = 0; i < count; i++) {
		int idx = (oldest + i) % IMU_RING_SIZE;

		if (g_imu_ring[idx].capture_us <= target_us)
			prev_idx = idx;
		else {
			next_idx = idx;
			break;
		}
	}

	if (prev_idx < 0) {
		/* target is before all samples — clamp to oldest */
		*out = g_imu_ring[oldest % IMU_RING_SIZE].tilt;
		*residual_us = (uint32_t)(g_imu_ring[oldest % IMU_RING_SIZE].capture_us
		                          - target_us);
		return 0;
	}
	if (next_idx < 0) {
		/* target is after all samples — clamp to newest */
		int newest = (g_imu_ring_head - 1 + IMU_RING_SIZE) % IMU_RING_SIZE;
		*out = g_imu_ring[newest].tilt;
		*residual_us = (uint32_t)(target_us
		                          - g_imu_ring[newest].capture_us);
		return 0;
	}

	/* Linear interpolation between prev and next. */
	{
		uint64_t t0    = g_imu_ring[prev_idx].capture_us;
		uint64_t t1    = g_imu_ring[next_idx].capture_us;
		double   alpha = (t1 > t0) ?
		                 (double)(target_us - t0) / (double)(t1 - t0) : 0.0;
		const imu_tilt_t *a = &g_imu_ring[prev_idx].tilt;
		const imu_tilt_t *b = &g_imu_ring[next_idx].tilt;

#define LERP(f) (float)((a->f) + alpha * ((b->f) - (a->f)))
		out->pitch       = LERP(pitch);
		out->roll        = LERP(roll);
		out->yaw         = LERP(yaw);
		out->gyro[0]     = LERP(gyro[0]);
		out->gyro[1]     = LERP(gyro[1]);
		out->gyro[2]     = LERP(gyro[2]);
		out->temperature = LERP(temperature);
#undef LERP
		out->status   = a->status;   /* discrete — nearest (prev) */
		out->has_accel = a->has_accel;
		out->has_quat  = a->has_quat;
		*residual_us  = 0;
	}
	return 0;
}

static void sig_handle(int signo)
{
	(void)signo;
	signal(SIGINT,  SIG_IGN);
	signal(SIGTERM, SIG_IGN);
	g_exit_flag = 1;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint64_t get_time_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000;
}

/**
 * @brief Print decoded imu tilt packet.
 * @param t_pkt Input imu packet.
 * @param t_user User context pointer.
 * @return None.
 */
static void imu_packet_print(const imu_packet_t *t_pkt, void *t_user)
{
	imu_tilt_t tilt;

	(void)t_user;

	if (!t_pkt)
		return;

	if (imu_decode_tilt(t_pkt, &tilt) != 0)
		return;

	/* Convert IMU system_time to CLOCK_MONOTONIC µs via calibrated offset.
	 * On the first packet, compute offset = mono_now - imu_now. */
	{
		uint64_t imu_us   = (uint64_t)(tilt.system_time * 1e6);
		uint64_t mono_now = get_time_us();
		uint64_t capture_us;

		/* Dynamic EMA offset: tracks clock drift between IMU oscillator
		 * and Linux CLOCK_MONOTONIC.  Both devices start from 0 at power-on
		 * so the raw difference is approximately the UART pipeline delay.
		 * α = 1/100 ≈ 0.01  →  time constant ≈ 100 packets (~1 s at 100 Hz).
		 * Integer-only arithmetic: new = old + (raw - old) / 100            */
		{
			int64_t raw_offset = (int64_t)mono_now - (int64_t)imu_us;
			if (!g_imu_clk_offset_valid) {
				g_imu_clk_offset_us    = raw_offset;
				g_imu_clk_offset_valid = 1;
			} else {
				g_imu_clk_offset_us = g_imu_clk_offset_us
				                    + (raw_offset - g_imu_clk_offset_us) / 100;
			}
		}
		capture_us = (uint64_t)((int64_t)imu_us + g_imu_clk_offset_us);
		imu_ring_push(&tilt, capture_us);

		/* Clock-domain diagnostics: once per second at DEBUG level. */
		LOGD_RL(1000000ULL,
		        "clk mono=%llu imu=%llu raw_diff=%lld ema_off=%lld us\n",
		        (unsigned long long)mono_now,
		        (unsigned long long)imu_us,
		        (long long)((int64_t)mono_now - (int64_t)imu_us),
		        (long long)g_imu_clk_offset_us);
	}
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
		g_imu_parser = imu_parser_create(imu_packet_print, NULL);
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

static int is_h265_vcl_nalu(H265E_NALU_TYPE_E enType)
{
	return enType == H265E_NALU_BSLICE ||
	       enType == H265E_NALU_PSLICE ||
	       enType == H265E_NALU_ISLICE ||
	       enType == H265E_NALU_IDRSLICE;
}

/*
 * Build H.265 prefix SEI carrying an IMU snapshot interpolated to frame_pts_us.
 *
 * frame_pts_us: hardware frame PTS in CLOCK_MONOTONIC µs (from pstPack[0].u64PTS).
 *
 * Annex-B NAL layout:
 *   [0..3]   start code (0x00 0x00 0x00 0x01)
 *   [4]      nal_type byte: (39 << 1) = 0x4E (prefix_sei)
 *   [5]      temporal_id_plus1 = 0x01
 *   [6..8]   SEI message header:
 *     [6]    payload_type = 5 (user_data_unregistered)
 *     [7]    payload_size = 0xFF (+ [8]=0x01 → 256 bytes)
 *   [9..264] SEI payload (256 bytes):
 *     [0..3]    frame_pts_ms      uint32 BE  — frame hardware PTS (ms)
 *     [4]       imu_valid         uint8      — 1 if interpolation succeeded
 *     [5..8]    imu_residual_us   uint32 BE  — bracket residual (µs)
 *     [9..12]   pitch             float32 LE (deg)
 *     [13..16]  roll              float32 LE (deg)
 *     [17..20]  yaw               float32 LE (deg)
 *     [21..24]  gyro[0]           float32 LE (deg/s)
 *     [25..28]  gyro[1]           float32 LE (deg/s)
 *     [29..32]  gyro[2]           float32 LE (deg/s)
 *     [33..36]  temperature       float32 LE (°C)
 *     [37]      status            uint8
 *     [38..255] reserved          (zeros)
 */
static size_t build_h265_prefix_sei(uint8_t *dst, size_t dst_size,
                                    uint64_t frame_pts_us)
{
	uint8_t payload[256];
	uint8_t sei_rbsp[1 + 4 + 16 + 256 + 1]; /* type + size(<=4 bytes enough) + data + rbsp_trailing_bits */
	static const uint8_t k_sei_uuid[16] = {
		0xAA, 0xAA, 0xAA, 0xAA, 0xBB, 0xBB, 0xBB, 0xBB,
		0xCC, 0xCC, 0xCC, 0xCC, 0xDD, 0xDD, 0xDD, 0xDD
	};
	size_t out = 0;
	size_t rbsp_len = 0;
	size_t i;
	int zero_count = 0;
	int payload_size = 16 + (int)sizeof(payload);
	int remain;

	if (!dst || dst_size < 64)
		return 0;

	memset(payload, 0, sizeof(payload));
	memset(sei_rbsp, 0, sizeof(sei_rbsp));

	/* Build payload using TH264SEI layout (249 bytes), remaining bytes keep zero.
	 * struct layout:
	 *   [0..3]   Head    = 0x01000000 (little-endian bytes: 00 00 00 01)
	 *   [4]      sei     = 0x06
	 *   [5]      seitype = 0x05
	 *   [6]      len     = 242 (uuid + type + union(224) + ends)
	 *   [7..22]  uuid[16]
	 *   [23]     type    = 2 (SEI_PAYLOAD_TYPE_XYZ)
	 *   [24..247] union  = jStr[224]
	 *   [248]    ends    = 0x80 */
	payload[0] = 0x00;
	payload[1] = 0x00;
	payload[2] = 0x00;
	payload[3] = 0x01;
	payload[4] = 0x06;
	payload[5] = 0x05;
	payload[6] = 242;
	memcpy(&payload[7], k_sei_uuid, sizeof(k_sei_uuid));
	payload[23] = 2;

	/* Fill union.jStr[224] with frame/IMU fields.
	 * jStr offsets below are relative to payload[24]. */
	{
		imu_tilt_t imu;
		uint32_t   residual_us = 0;
		uint32_t pts_ms = (uint32_t)(frame_pts_us / 1000ULL);

		/* Verify PTS vs ring head at TRACE level, once per second. */
#if LOG_LEVEL >= LOG_LEVEL_TRACE
		if (g_imu_ring_count > 0) {
			int newest = (g_imu_ring_head - 1 + IMU_RING_SIZE) % IMU_RING_SIZE;
			uint64_t newest_us = g_imu_ring[newest].capture_us;
			LOGT_RL(1000000ULL,
			        "sei frame_pts=%llu newest_imu=%llu diff=%lld us\n",
			        (unsigned long long)frame_pts_us,
			        (unsigned long long)newest_us,
			        (long long)((int64_t)newest_us - (int64_t)frame_pts_us));
		}
#endif

		if (imu_ring_interpolate(frame_pts_us, &imu, &residual_us) == 0) {
			payload[24] = (uint8_t)((pts_ms >> 24) & 0xFF);
			payload[25] = (uint8_t)((pts_ms >> 16) & 0xFF);
			payload[26] = (uint8_t)((pts_ms >>  8) & 0xFF);
			payload[27] = (uint8_t)( pts_ms        & 0xFF);
			payload[28] = 1;  /* imu_valid */
			payload[29] = (uint8_t)((residual_us >> 24) & 0xFF);
			payload[30] = (uint8_t)((residual_us >> 16) & 0xFF);
			payload[31] = (uint8_t)((residual_us >>  8) & 0xFF);
			payload[32] = (uint8_t)( residual_us        & 0xFF);
			memcpy(&payload[33], &imu.pitch,       4);
			memcpy(&payload[37], &imu.roll,        4);
			memcpy(&payload[41], &imu.yaw,         4);
			memcpy(&payload[45], &imu.gyro[0],     4);
			memcpy(&payload[49], &imu.gyro[1],     4);
			memcpy(&payload[53], &imu.gyro[2],     4);
			memcpy(&payload[57], &imu.temperature, 4);
			payload[61] = imu.status;
		}
	}
	payload[248] = 0x80;

	/* Build RBSP (without start code / NAL header). */
	sei_rbsp[rbsp_len++] = 5;  /* payload_type: user_data_unregistered */

	remain = payload_size;
	while (remain >= 0xFF) {
		sei_rbsp[rbsp_len++] = 0xFF;
		remain -= 0xFF;
	}
	sei_rbsp[rbsp_len++] = (uint8_t)remain;

	memcpy(&sei_rbsp[rbsp_len], k_sei_uuid, sizeof(k_sei_uuid));
	rbsp_len += sizeof(k_sei_uuid);
	memcpy(&sei_rbsp[rbsp_len], payload, sizeof(payload));
	rbsp_len += sizeof(payload);
	sei_rbsp[rbsp_len++] = 0x80; /* rbsp_trailing_bits */

	/* Annex-B start code + NAL header (must not be emulation-protected). */
	if (dst_size < 6)
		return 0;
	dst[out++] = 0x00;
	dst[out++] = 0x00;
	dst[out++] = 0x00;
	dst[out++] = 0x01;
	dst[out++] = 39 << 1; /* nal_unit_type = 39 (prefix_sei), nuh_layer_id = 0 */
	dst[out++] = 0x01;    /* temporal_id_plus1 = 1 */

	/* Emulation prevention over RBSP bytes only. */
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

/* Send one VENC frame via RTSP and inject one prefix SEI before the first VCL NAL. */
static void send_venc_stream(VENC_STREAM_S *pstStream)
{
	CVI_U32 i;
	int sei_sent = 0;
	uint8_t sei_nal[512];
	/* Use the hardware PTS of the first pack as the frame capture time.
	 * u64PTS is in µs on the same CLOCK_MONOTONIC domain as get_time_us(). */
	uint64_t frame_pts_us = (pstStream->u32PackCount > 0)
	                        ? pstStream->pstPack[0].u64PTS
	                        : get_time_us();
	size_t sei_len = build_h265_prefix_sei(sei_nal, sizeof(sei_nal), frame_pts_us);

	if (pstStream->u32PackCount == 0)
		return;

	/* Send each NAL unit separately so the RTSP H265 source sees
	 * one NAL per call (start-code Annex-B data per pack). */
	for (i = 0; i < pstStream->u32PackCount; i++) {
		VENC_PACK_S *p = &pstStream->pstPack[i];
		int len = (int)(p->u32Len - p->u32Offset);

		if (!sei_sent && is_h265_vcl_nalu(p->DataType.enH265EType) && sei_len > 0) {
			rtsp_send_h265_data(sei_nal, sei_len);
			sei_sent = 1;
		}

		if (len > 0)
			rtsp_send_h265_data(p->pu8Addr + p->u32Offset, (size_t)len);
	}

	if (!sei_sent && sei_len > 0)
		rtsp_send_h265_data(sei_nal, sei_len);
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

	/* 4. Encode → stream loop */
	{
		VENC_CHN       VencChn  = 0;
		CVI_S32        venc_fd  = CVI_VENC_GetFd(VencChn);
		int            uart_fd  = -1;
		uint64_t       frame_count = 0;
		uint64_t       last_us    = get_time_us();

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

		while (!g_exit_flag) {
			struct timeval tv;
			tv.tv_sec  = 1;
			tv.tv_usec = 0;
			fd_set read_fds;
			int max_fd;
			int sel;
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

			if (sel == 0)
				continue;
			if (!FD_ISSET(venc_fd, &read_fds))
				continue;

			/* Query how many packs are ready */
			VENC_CHN_STATUS_S stStat = {};
			if (CVI_VENC_QueryStatus(VencChn, &stStat) != CVI_SUCCESS)
				continue;
			if (stStat.u32CurPacks == 0)
				continue;

			VENC_STREAM_S stStream = {};
			stStream.pstPack = (VENC_PACK_S *)malloc(
			    sizeof(VENC_PACK_S) * stStat.u32CurPacks);
			if (!stStream.pstPack)
				continue;

			if (CVI_VENC_GetStream(VencChn, &stStream, 1000) == CVI_SUCCESS) {
				if (frame_count == 0) {
					LOGI("first H265 frame: %u packs\n",
					     stStream.u32PackCount);
				}
				send_venc_stream(&stStream);
				CVI_VENC_ReleaseStream(VencChn, &stStream);
				frame_count++;
			}
			free(stStream.pstPack);

			uint64_t now = get_time_us();
			if (frame_count % 100 == 0 && frame_count > 0) {
				LOGI("frame %llu  fps=%.1f\n",
				     (unsigned long long)frame_count,
				     100.0 * 1e6 / (double)(now - last_us));
				last_us = now;
			}
		}

		LOGI("shutting down after %llu frames\n",
		     (unsigned long long)frame_count);

		uart1_deinit(uart_fd);
	}

err_venc:
	sys_venc_deinit();
err_vi:
	sys_vi_deinit();
err_rtsp:
	rtsp_server_deinit();
	return ret;
}
