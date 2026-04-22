/*
 * stream_demo.c - LicheeRV Nano H.265 UDP MPEG-TS streaming demo
 *
 * Pipeline: VI (GC4653) -> ISP -> VPSS -> H.265 VENC -> MPEG-TS -> UDP
 * Mode: VI_OFFLINE_VPSS_ONLINE + VPSS_INPUT_ISP (VPSS_MODE_SINGLE)
 *
 * Usage: stream_demo [enc_width enc_height [host port]]
 *   Default encode resolution: 2560x1440 (GC4653 max)
 *   Default target: 192.168.2.255:1234 (UDP broadcast)
 *   Play on host: ffplay udp://@:1234
 *              or ffplay udp://192.168.2.18:1234  (direct)
 *
 * Note: For unicast, run:  stream_demo 2560 1440 192.168.2.X 1234
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sample_comm.h"
#include "mpeg-ts.h"
#include "mpeg-proto.h"

/* ------------------------------------------------------------------ */
/* Global state                                                        */
/* ------------------------------------------------------------------ */

static volatile int            g_exit_flag = 0;
static SAMPLE_VI_CONFIG_S      g_stViConfig;
static SAMPLE_INI_CFG_S        g_stIniCfg;

/* MPEG-TS / UDP output state */
static int                     g_udp_sock       = -1;
static struct sockaddr_in      g_udp_dst;
static void                   *g_ts_muxer       = NULL;
static int                     g_ts_vid_stream  = -1;
static int64_t                 g_ts_pts         = 0;   /* 90 kHz clock */

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

/* TS alloc/free/write callbacks required by libmpeg. */
static void *ts_alloc_cb(void *param, size_t bytes)
{
	(void)param;
	return malloc(bytes);
}

static void ts_free_cb(void *param, void *packet)
{
	(void)param;
	free(packet);
}

static int ts_write_cb(void *param, const void *data, size_t bytes)
{
	(void)param;
	sendto(g_udp_sock, data, bytes, 0,
	       (struct sockaddr *)&g_udp_dst, sizeof(g_udp_dst));
	return 0;
}

/* Initialise UDP socket and MPEG-TS muxer. */
static int ts_udp_init(const char *host, int port)
{
	static const struct mpeg_ts_func_t ts_func = { ts_alloc_cb, ts_free_cb, ts_write_cb };
	int on = 1;

	g_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (g_udp_sock < 0) {
		perror("[stream_demo] socket");
		return -1;
	}
	/* Allow broadcast addresses */
	setsockopt(g_udp_sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));

	memset(&g_udp_dst, 0, sizeof(g_udp_dst));
	g_udp_dst.sin_family = AF_INET;
	g_udp_dst.sin_port   = htons((uint16_t)port);
	if (inet_aton(host, &g_udp_dst.sin_addr) == 0) {
		fprintf(stderr, "[stream_demo] invalid host: %s\n", host);
		close(g_udp_sock);
		g_udp_sock = -1;
		return -1;
	}

	g_ts_muxer = mpeg_ts_create(&ts_func, NULL);
	if (!g_ts_muxer) {
		fprintf(stderr, "[stream_demo] mpeg_ts_create failed\n");
		close(g_udp_sock);
		g_udp_sock = -1;
		return -1;
	}

	g_ts_vid_stream = mpeg_ts_add_stream(g_ts_muxer, PSI_STREAM_H265, NULL, 0);
	if (g_ts_vid_stream <= 0) {
		fprintf(stderr, "[stream_demo] mpeg_ts_add_stream failed\n");
		mpeg_ts_destroy(g_ts_muxer);
		g_ts_muxer = NULL;
		close(g_udp_sock);
		g_udp_sock = -1;
		return -1;
	}

	return 0;
}

static void ts_udp_deinit(void)
{
	if (g_ts_muxer) {
		mpeg_ts_destroy(g_ts_muxer);
		g_ts_muxer = NULL;
	}
	if (g_udp_sock >= 0) {
		close(g_udp_sock);
		g_udp_sock = -1;
	}
}

/*
 * Mux one VENC frame into MPEG-TS and send via UDP.
 * All packs are concatenated into one Annex-B access unit,
 * which is what mpeg_ts_write() expects.
 */
static void send_venc_stream(VENC_STREAM_S *pstStream)
{
	CVI_U32  i;
	size_t   total = 0;
	size_t   off   = 0;
	int      is_idr = 0;
	uint8_t *buf;

	if (pstStream->u32PackCount == 0)
		return;

	/* Calculate total size and detect IDR */
	for (i = 0; i < pstStream->u32PackCount; i++) {
		VENC_PACK_S *p = &pstStream->pstPack[i];
		total += (size_t)(p->u32Len - p->u32Offset);
		if (p->DataType.enH265EType == H265E_NALU_IDRSLICE)
			is_idr = 1;
	}

	buf = (uint8_t *)malloc(total);
	if (!buf)
		return;

	/* Copy all NALs (each already has Annex-B start code from VENC) */
	for (i = 0; i < pstStream->u32PackCount; i++) {
		VENC_PACK_S *p = &pstStream->pstPack[i];
		size_t len = (size_t)(p->u32Len - p->u32Offset);
		memcpy(buf + off, p->pu8Addr + p->u32Offset, len);
		off += len;
	}

	/* flags: 0x0001 = IDR/random-access point */
	mpeg_ts_write(g_ts_muxer, g_ts_vid_stream,
	              is_idr ? 0x0001 : 0,
	              g_ts_pts, g_ts_pts,
	              buf, total);
	g_ts_pts += 3000; /* 90000 Hz / 30 fps */

	free(buf);
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

	SAMPLE_COMM_VI_UnBind_VPSS(0, 0, 0);
	SAMPLE_COMM_VPSS_Stop(0, abChnEnable);
	SAMPLE_COMM_VI_DestroyIsp(&g_stViConfig);
	SAMPLE_COMM_VI_DestroyVi(&g_stViConfig);
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
	stChnAttr.stRcAttr.stH265Cbr.u32BitRate      = 4096;  /* 4 Mbps */
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
	int         enc_w = 2560;
	int         enc_h = 1440;
	const char *host  = "192.168.2.255";
	int         port  = 1234;
	int         ret   = 0;

	if (argc == 3 || argc == 5) {
		enc_w = atoi(argv[1]);
		enc_h = atoi(argv[2]);
		if (enc_w <= 0 || enc_h <= 0) {
			fprintf(stderr, "Usage: %s [enc_w enc_h [host port]]\n", argv[0]);
			return 1;
		}
		if (argc == 5) {
			host = argv[3];
			port = atoi(argv[4]);
			if (port <= 0 || port > 65535) {
				fprintf(stderr, "Invalid port: %s\n", argv[4]);
				return 1;
			}
		}
	} else if (argc != 1) {
		fprintf(stderr, "Usage: %s [enc_w enc_h [host port]]\n", argv[0]);
		return 1;
	}

	signal(SIGINT,  sig_handle);
	signal(SIGTERM, sig_handle);
	signal(SIGPIPE, SIG_IGN);

	printf("[stream_demo] starting, encode %dx%d H265 -> udp://%s:%d\n",
	       enc_w, enc_h, host, port);
	printf("[stream_demo] play with: ffplay udp://@:%d\n", port);
	fflush(stdout);

	/* 1. Init UDP + MPEG-TS muxer */
	if (ts_udp_init(host, port) != 0) {
		fprintf(stderr, "[stream_demo] ts_udp_init failed\n");
		return 1;
	}

	/* 2. Init VI + VPSS */
	if (sys_vi_init(enc_w, enc_h) != 0) {
		fprintf(stderr, "[stream_demo] sys_vi_init failed\n");
		ret = 1;
		goto err_ts;
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
		uint64_t       frame_count = 0;
		uint64_t       last_us    = get_time_us();

		if (venc_fd <= 0) {
			fprintf(stderr, "[stream_demo] CVI_VENC_GetFd failed %d\n", venc_fd);
			ret = 1;
			goto err_venc;
		}

		printf("[stream_demo] entering encode/stream loop (fd=%d)\n", venc_fd);
		fflush(stdout);

		while (!g_exit_flag) {
			struct timeval tv;
			tv.tv_sec  = 1;
			tv.tv_usec = 0;
			fd_set read_fds;
			FD_ZERO(&read_fds);
			FD_SET(venc_fd, &read_fds);

			int sel = select(venc_fd + 1, &read_fds, NULL, NULL, &tv);
			if (sel < 0) {
				if (g_exit_flag) break;
				perror("[stream_demo] select");
				break;
			}
			if (sel == 0)   /* timeout — no frame yet, loop */
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
	}

err_venc:
	sys_venc_deinit();
err_vi:
	sys_vi_deinit();
err_ts:
	ts_udp_deinit();
	return ret;
}
