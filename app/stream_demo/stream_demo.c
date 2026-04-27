/*
 * stream_demo.c - H.265 RTSP streaming demo
 *
 * Pipeline: VI (GC4653) -> ISP -> VPSS -> H.265 VENC -> RTSP server
 * Mode: VI_OFFLINE_VPSS_ONLINE + VPSS_INPUT_ISP (VPSS_MODE_SINGLE)
 *
 * Usage: stream_demo [enc_width enc_height]
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

#include "imu.h"
#include "sample_comm.h"
#include "rtsp-server.h"

/* ------------------------------------------------------------------ */
/* Global state                                                        */
/* ------------------------------------------------------------------ */

static volatile int            g_exit_flag = 0;
static SAMPLE_VI_CONFIG_S      g_stViConfig;
static SAMPLE_INI_CFG_S        g_stIniCfg;
static uint8_t                 g_sei_fill_value = 0;
static imu_parser_t           *g_imu_parser = NULL;

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

	printf("[imu] time=%.3f status=%u\n",
	       tilt.system_time, tilt.status);
	printf("[imu] gyro=%.4f %.4f %.4f deg/s\n",
	       tilt.gyro[0], tilt.gyro[1], tilt.gyro[2]);
	if (tilt.has_accel) {
		printf("[imu] accel=%.4f %.4f %.4f m/s^2\n",
		       tilt.accel[0], tilt.accel[1], tilt.accel[2]);
	}
	printf("[imu] euler pitch=%.4f roll=%.4f yaw=%.4f deg\n",
	       tilt.pitch, tilt.roll, tilt.yaw);
	if (tilt.has_quat) {
		printf("[imu] temp=%.3f C quat=%.6f %.6f %.6f %.6f\n",
		       tilt.temperature,
		       tilt.quat[0], tilt.quat[1], tilt.quat[2], tilt.quat[3]);
	} else {
		printf("[imu] temp=%.3f C\n", tilt.temperature);
	}
	fflush(stdout);
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
			perror("[stream_demo] uart1 read");
		return;
	}

	printf("[stream_demo] uart1 rx %zd bytes\r\n", rx_len);
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

static size_t build_h265_prefix_sei(uint8_t *dst, size_t dst_size, uint8_t fill_value)
{
	uint8_t rbsp[260];
	size_t rbsp_len = 0;
	size_t out = 0;
	uint32_t now_ms = (uint32_t)(get_time_us() / 1000ULL);
	int zero_count = 0;
	int i;

	if (dst_size < 512)
		return 0;

	/* payload_type = 5 (user_data_unregistered style custom payload) */
	rbsp[rbsp_len++] = 5;
	/* payload_size = 256 => 0xFF, 0x01 */
	rbsp[rbsp_len++] = 0xFF;
	rbsp[rbsp_len++] = 0x01;
	rbsp[rbsp_len++] = (uint8_t)((now_ms >> 24) & 0xFF);
	rbsp[rbsp_len++] = (uint8_t)((now_ms >> 16) & 0xFF);
	rbsp[rbsp_len++] = (uint8_t)((now_ms >> 8) & 0xFF);
	rbsp[rbsp_len++] = (uint8_t)(now_ms & 0xFF);
	for (i = 4; i < 256; i++)
		rbsp[rbsp_len++] = fill_value;
	/* rbsp_trailing_bits() */
	rbsp[rbsp_len++] = 0x80;

	/* Annex-B start code */
	dst[out++] = 0x00;
	dst[out++] = 0x00;
	dst[out++] = 0x00;
	dst[out++] = 0x01;
	/* nal_unit_type = 39 (prefix_sei), nuh_layer_id = 0, temporal_id_plus1 = 1 */
	dst[out++] = 39 << 1;
	dst[out++] = 0x01;

	for (i = 0; i < (int)rbsp_len; i++) {
		uint8_t b = rbsp[i];

		if (zero_count >= 2 && b <= 0x03) {
			dst[out++] = 0x03;
			zero_count = 0;
		}

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
	size_t sei_len = build_h265_prefix_sei(sei_nal, sizeof(sei_nal), g_sei_fill_value);

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

	g_sei_fill_value++;
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
		printf("[stream_demo] sensor_cfg.ini parsed OK\n");

	CVI_VI_SetDevNum(stIniCfg.devNum);

	/* Step 2: build VI config from ini */
	s32Ret = SAMPLE_COMM_VI_IniToViCfg(&stIniCfg, &stViConfig);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] IniToViCfg failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	/* Step 3: get native sensor frame size */
	s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stIniCfg.enSnsType[0], &enPicSize);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] GetSizeBySensor failed 0x%x\n", s32Ret);
		return s32Ret;
	}
	s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSensorSize);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] GetPicSize failed 0x%x\n", s32Ret);
		return s32Ret;
	}
	printf("[stream_demo] sensor size: %ux%u\n",
	       stSensorSize.u32Width, stSensorSize.u32Height);

	/* Step 4: init VB pools sized for sensor frame */
	s32Ret = SAMPLE_PLAT_SYS_INIT(stSensorSize);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] SYS_INIT failed 0x%x\n", s32Ret);
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
			printf("[stream_demo] SetVIVPSSMode failed 0x%x\n", s32Ret);
			return s32Ret;
		}
		s32Ret = CVI_SYS_SetVPSSModeEx(&stVPSSMode);
		if (s32Ret != CVI_SUCCESS) {
			printf("[stream_demo] SetVPSSModeEx failed 0x%x\n", s32Ret);
			return s32Ret;
		}
	}

	/* Step 6: start sensor, MIPI, ISP, VI pipe & channels */
	s32Ret = SAMPLE_PLAT_VI_INIT(&stViConfig);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] PLAT_VI_INIT failed 0x%x\n", s32Ret);
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
			printf("[stream_demo] VPSS_Init failed 0x%x\n", s32Ret);
			return s32Ret;
		}
		s32Ret = SAMPLE_COMM_VPSS_Start(0, abChnEnable, &stGrpAttr, stChnAttr);
		if (s32Ret != CVI_SUCCESS) {
			printf("[stream_demo] VPSS_Start failed 0x%x\n", s32Ret);
			return s32Ret;
		}
	}

	/* Step 8: bind VI pipe 0 channel 0 -> VPSS group 0 */
	s32Ret = SAMPLE_COMM_VI_Bind_VPSS(0, 0, 0);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] VI_Bind_VPSS failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	printf("[stream_demo] VI+VPSS init OK, encode %ux%u\n",
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
		printf("[stream_demo] CVI_VENC_CreateChn failed 0x%x\n", s32Ret);
		return s32Ret;
	}

	/* Bind VPSS group 0 channel 0 -> VENC channel 0 */
	s32Ret = SAMPLE_COMM_VPSS_Bind_VENC(0, 0, 0);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] VPSS_Bind_VENC failed 0x%x\n", s32Ret);
		CVI_VENC_DestroyChn(0);
		return s32Ret;
	}

	stRecvParam.s32RecvPicNum = -1;   /* encode indefinitely */
	s32Ret = CVI_VENC_StartRecvFrame(0, &stRecvParam);
	if (s32Ret != CVI_SUCCESS) {
		printf("[stream_demo] StartRecvFrame failed 0x%x\n", s32Ret);
		SAMPLE_COMM_VPSS_UnBind_VENC(0, 0, 0);
		CVI_VENC_DestroyChn(0);
		return s32Ret;
	}

	printf("[stream_demo] VENC H265 init OK\n");
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
			fprintf(stderr, "Usage: %s [enc_width enc_height]\n", argv[0]);
			return 1;
		}
	} else if (argc != 1) {
		fprintf(stderr, "Usage: %s [enc_width enc_height]\n", argv[0]);
		return 1;
	}

	signal(SIGINT,  sig_handle);
	signal(SIGTERM, sig_handle);
	signal(SIGPIPE, SIG_IGN);

	printf("[stream_demo] starting, encode %dx%d H265\n", enc_w, enc_h);
	fflush(stdout);

	/* 1. Start RTSP server */
	if (rtsp_server_init(NULL, 8554) != 0) {
		fprintf(stderr, "[stream_demo] rtsp_server_init failed\n");
		return 1;
	}
	if (rtsp_server_start() != 0) {
		fprintf(stderr, "[stream_demo] rtsp_server_start failed\n");
		rtsp_server_deinit();
		return 1;
	}
	printf("[stream_demo] RTSP URL: rtsp://%s:%d/live\n",
	       rtsp_get_server_ip(), rtsp_get_server_port());
	fflush(stdout);

	/* 2. Init VI + VPSS */
	if (sys_vi_init(enc_w, enc_h) != 0) {
		fprintf(stderr, "[stream_demo] sys_vi_init failed\n");
		ret = 1;
		goto err_rtsp;
	}

	/* 3. Init VENC H265 */
	if (sys_venc_h265_init(enc_w, enc_h) != 0) {
		fprintf(stderr, "[stream_demo] sys_venc_h265_init failed\n");
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
			fprintf(stderr, "[stream_demo] CVI_VENC_GetFd failed %d\n", venc_fd);
			ret = 1;
			goto err_venc;
		}

		if (uart1_init(&uart_fd) == 0)
			printf("[stream_demo] uart1 ready: /dev/ttyS1 460800 8N1 (A18/A19)\n");
		else
			fprintf(stderr, "[stream_demo] uart1 init failed, continue without uart\n");

		printf("[stream_demo] entering encode/stream loop (fd=%d)\n", venc_fd);
		fflush(stdout);

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
				perror("[stream_demo] select");
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
					printf("[stream_demo] first H265 frame: %u packs\n",
					       stStream.u32PackCount);
					fflush(stdout);
				}
				send_venc_stream(&stStream);
				CVI_VENC_ReleaseStream(VencChn, &stStream);
				frame_count++;
			}
			free(stStream.pstPack);

			uint64_t now = get_time_us();
			if (frame_count % 100 == 0 && frame_count > 0) {
				printf("[stream_demo] frame %llu  fps=%.1f\n",
				       (unsigned long long)frame_count,
				       100.0 * 1e6 / (double)(now - last_us));
				fflush(stdout);
				last_us = now;
			}
		}

		printf("[stream_demo] shutting down after %llu frames\n",
		       (unsigned long long)frame_count);
		fflush(stdout);

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
