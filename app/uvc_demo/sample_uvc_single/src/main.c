#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <semaphore.h>
#include <errno.h>
#include <glob.h>
#include <string.h>

#include "cvi_uvc.h"
#include "cvi_uvc_gadget.h"
#include "cvi_system.h"
#include "sample_comm.h"
#include "hid_ctl.h"

static sem_t s_ExitSem;/** EXIT semaphore */
static SAMPLE_VI_CONFIG_S stViConfig;

static int find_uvc_video_node(char *out, size_t out_len)
{
	glob_t g;
	int i;
	const char *base;

	if (out == NULL || out_len == 0)
		return -1;

	/* Prefer the node exported by the active UDC gadget path. */
	if (glob("/sys/class/udc/*/device/gadget/video4linux/video*", 0, NULL, &g) == 0 && g.gl_pathc > 0) {
		base = strrchr(g.gl_pathv[0], '/');
		if (base != NULL && *(base + 1) != '\0') {
			snprintf(out, out_len, "/dev/%s", base + 1);
			globfree(&g);
			return 0;
		}
		globfree(&g);
	}

	/* Fallback scan in case sysfs path is unavailable. */
	for (i = 0; i < 10; i++) {
		snprintf(out, out_len, "/dev/video%d", i);
		if (access(out, F_OK) == 0)
			return 0;
	}

	out[0] = '\0';
	return -1;
}

void sig_handler(int signo)
{
    if (SIGTERM == signo || SIGINT == signo) {
        if (UVC_Stop() != 0) {
            printf("UVC_Stop Failed !");
        }
        if (UVC_Deinit() != 0) {
            printf("UVC_Deinit Failed !");
        }
    }
	exit(0);
}

static int32_t _CVI_Init_Vi(SIZE_S *pstSize)
{
	MMF_VERSION_S stVersion;
	SAMPLE_INI_CFG_S	   stIniCfg = {0};

	PIC_SIZE_E enPicSize;
	int32_t s32Ret = CVI_SUCCESS;
	//int op;
	LOG_LEVEL_CONF_S log_conf;

	CVI_SYS_GetVersion(&stVersion);
	printf("MMF Version:%s\n", stVersion.version);

	log_conf.enModId = CVI_ID_LOG;
	log_conf.s32Level = CVI_DBG_INFO;
	CVI_LOG_SetLevelConf(&log_conf);

	// Get config from ini if found.
	if (SAMPLE_COMM_VI_ParseIni(&stIniCfg)) {
		printf("Parse complete\n");
	}

	//Set sensor number
	CVI_VI_SetDevNum(stIniCfg.devNum);

	/************************************************
	 * step1:  Config VI
	 ************************************************/
	s32Ret = SAMPLE_COMM_VI_IniToViCfg(&stIniCfg, &stViConfig);
	printf("stage IniToViCfg ret=0x%x\n", s32Ret);
	if (s32Ret != CVI_SUCCESS)
		return s32Ret;

	/************************************************
	 * step2:  Get input size
	 ************************************************/
	s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stIniCfg.enSnsType[0], &enPicSize);
	printf("stage GetSizeBySensor ret=0x%x\n", s32Ret);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_VI_GetSizeBySensor failed with %#x\n", s32Ret);
		return s32Ret;
	}

	s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, pstSize);
	printf("stage GetPicSize ret=0x%x\n", s32Ret);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_SYS_GetPicSize failed with %#x\n", s32Ret);
		return s32Ret;
	}

	printf("SAMPLE_COMM_SYS_GetPicSize %dx%d\n", pstSize->u32Width, pstSize->u32Height);

	/************************************************
	 * step3:  Init VB pool
	 ************************************************/
	s32Ret = SAMPLE_PLAT_SYS_INIT(*pstSize);
	printf("stage PLAT_SYS_INIT ret=0x%x\n", s32Ret);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "sys init failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	/* Align VI/VPSS pipeline mode with known working configuration on SG2002. */
	{
		VI_VPSS_MODE_S stVIVPSSMode = {0};
		VPSS_MODE_S stVPSSMode = {0};

		stVIVPSSMode.aenMode[0] = VI_OFFLINE_VPSS_ONLINE;
		stVIVPSSMode.aenMode[1] = VI_OFFLINE_VPSS_ONLINE;

		stVPSSMode.enMode = VPSS_MODE_SINGLE;
		stVPSSMode.aenInput[0] = VPSS_INPUT_ISP;
		stVPSSMode.ViPipe[0] = 0;

		s32Ret = CVI_SYS_SetVIVPSSMode(&stVIVPSSMode);
		printf("stage SetVIVPSSMode ret=0x%x\n", s32Ret);
		if (s32Ret != CVI_SUCCESS) {
			CVI_TRACE_LOG(CVI_DBG_ERR, "SetVIVPSSMode failed. s32Ret: 0x%x !\n", s32Ret);
			return s32Ret;
		}

		s32Ret = CVI_SYS_SetVPSSModeEx(&stVPSSMode);
		printf("stage SetVPSSModeEx ret=0x%x\n", s32Ret);
		if (s32Ret != CVI_SUCCESS) {
			CVI_TRACE_LOG(CVI_DBG_ERR, "SetVPSSModeEx failed. s32Ret: 0x%x !\n", s32Ret);
			return s32Ret;
		}
	}

	/************************************************
	 * step4:  Init VI modules
	 ************************************************/
	if (stIniCfg.enSource == VI_PIPE_FRAME_SOURCE_DEV) {
		s32Ret = SAMPLE_PLAT_VI_INIT(&stViConfig);
		printf("stage PLAT_VI_INIT ret=0x%x\n", s32Ret);
		if (s32Ret != CVI_SUCCESS) {
			CVI_TRACE_LOG(CVI_DBG_ERR, "vi init failed. s32Ret: 0x%x !\n", s32Ret);
			return s32Ret;
		}
	}

	// VI_CHN_ATTR_S stChnAttr;
	// CVI_VI_GetChnAttr(0, 0, &stChnAttr);
	// stChnAttr.stSize.u32Width = 1280;
	// stChnAttr.stSize.u32Height = 720;
	// CVI_VI_SetChnAttr(0, 0, &stChnAttr);

	// CVI_VI_GetChnAttr(1, 1, &stChnAttr);
	// stChnAttr.stSize.u32Width = 1280;
	// stChnAttr.stSize.u32Height = 720;
	// CVI_VI_SetChnAttr(1, 1, &stChnAttr);

	// CVI_VI_SetChnRotation(0, 0, ROTATION_90);
	// CVI_VI_SetChnRotation(1, 1, ROTATION_90);

	//system("stty erase ^H");
	return s32Ret;
}

static CVI_S32  _CVI_Init_Vpss(VI_CHN ViChn, VPSS_GRP VpssGrp, VPSS_GRP_ATTR_S *stVpssGrpAttr)
{
	/* Defensive cleanup: destroy any leftover group from a prior run */
	CVI_VPSS_StopGrp(VpssGrp);
	CVI_VPSS_DestroyGrp(VpssGrp);
	SAMPLE_COMM_VI_UnBind_VPSS(0, ViChn, VpssGrp);

	VPSS_CHN VpssChn = 0;
	CVI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = { 0 };
	VPSS_CHN_ATTR_S astVpssChnAttr[VPSS_MAX_PHY_CHN_NUM];
	CVI_S32 s32Ret = CVI_SUCCESS;
	CVI_S32 chanel_num = (VpssGrp == 0) ? 2 : 1;

	for (VpssChn = 0; VpssChn < chanel_num; VpssChn++) {
		abChnEnable[VpssChn] = CVI_TRUE;
		astVpssChnAttr[VpssChn].u32Width                    = 1280;
		astVpssChnAttr[VpssChn].u32Height                   = 720;
		astVpssChnAttr[VpssChn].enVideoFormat               = VIDEO_FORMAT_LINEAR;
		astVpssChnAttr[VpssChn].enPixelFormat               = PIXEL_FORMAT_NV21;
		astVpssChnAttr[VpssChn].stFrameRate.s32SrcFrameRate = 30;
		astVpssChnAttr[VpssChn].stFrameRate.s32DstFrameRate = 30;
		astVpssChnAttr[VpssChn].u32Depth                    = 1;
		astVpssChnAttr[VpssChn].bMirror                     = CVI_FALSE;
		astVpssChnAttr[VpssChn].bFlip                       = CVI_FALSE;
		astVpssChnAttr[VpssChn].stAspectRatio.enMode        = ASPECT_RATIO_AUTO;
		astVpssChnAttr[VpssChn].stAspectRatio.bEnableBgColor = CVI_TRUE;
		astVpssChnAttr[VpssChn].stNormalize.bEnable         = CVI_FALSE;
	}

	/*start vpss*/
	s32Ret = SAMPLE_COMM_VPSS_Init(VpssGrp, abChnEnable, stVpssGrpAttr, astVpssChnAttr);
	if (s32Ret != CVI_SUCCESS) {
		SAMPLE_PRT("init vpss group failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	s32Ret = SAMPLE_COMM_VPSS_Start(VpssGrp, abChnEnable, stVpssGrpAttr, astVpssChnAttr);
	if (s32Ret != CVI_SUCCESS) {
		SAMPLE_PRT("start vpss group failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	s32Ret = SAMPLE_COMM_VI_Bind_VPSS(0, ViChn, VpssGrp);
	if (s32Ret != CVI_SUCCESS) {
		SAMPLE_PRT("vi bind vpss failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	return s32Ret;
}

int main(int argc, char const *argv[])
{
	int i;
    CVI_S32 s32Ret = CVI_SUCCESS;
	SIZE_S stSize;
	VPSS_GRP_ATTR_S stVpssGrpAttr;
	CVI_UVC_DEVICE_CAP_S stDeviceCap = {0};
	CVI_UVC_DATA_SOURCE_S stDataSource = {0};
	CVI_UVC_BUFFER_CFG_S stBuffer = {0};
	char uvc_devname[UVC_CAMERA_NUM_MAX][32] = {{0}};

	UNUSED(argc);
	UNUSED(argv);

	/* Disable stdio buffering so log output appears immediately */
	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	/* Stop default UVC server to avoid /dev/video0 ownership conflict. */
	system("fuser -k /etc/init.d/uvc-gadget-server.elf >/dev/null 2>&1");

	stDataSource.AcapHdl = 0;
	stDataSource.VcapHdl = 0;
	for (i = 0; i < UVC_CAMERA_NUM_MAX; i++) {
		stDataSource.VprocHdl[i] = i;
		stDataSource.VencHdl[i] = i;
	}
	stDataSource.VprocChnId = 0;

	/* Open UVC device early, otherwise host may connect before userspace owns /dev/video0. */
	if (UVC_Init(&stDeviceCap, &stDataSource, &stBuffer) != 0) {
		printf("UVC_Init Failed !");
		return -1;
	}

	for (i = 0; i < 50; i++) {
		if (find_uvc_video_node(uvc_devname[0], sizeof(uvc_devname[0])) == 0) {
			break;
		}
		usleep(100 * 1000);
	}
	if (uvc_devname[0][0] == '\0') {
		printf("No UVC video node found!\n");
		UVC_Deinit();
		return -1;
	}
	printf("Using UVC device node: %s\n", uvc_devname[0]);

	if (UVC_Start((const char *)uvc_devname) != 0) {
		printf("UVC_Start Failed !");
		UVC_Deinit();
		return -1;
	}

	// start Vi
	s32Ret = _CVI_Init_Vi(&stSize);
	if (s32Ret != CVI_SUCCESS) {
		printf("_CVI_Init_Vi fail, %d\n", s32Ret);
		goto EXIT_UVC;
	}

	// start Vpss
	stVpssGrpAttr.stFrameRate.s32SrcFrameRate = -1;
	stVpssGrpAttr.stFrameRate.s32DstFrameRate = -1;
	stVpssGrpAttr.enPixelFormat = VI_PIXEL_FORMAT;
	stVpssGrpAttr.u32MaxW = 1280;
	stVpssGrpAttr.u32MaxH = 720;
	/// only for test here. u8VpssDev should be decided by VPSS_MODE and usage.
	stVpssGrpAttr.u8VpssDev = 0;

	s32Ret = _CVI_Init_Vpss(0, 0, &stVpssGrpAttr);
	if (s32Ret != CVI_SUCCESS) {
		SAMPLE_PRT("CVI_Init_Video_Process Grp0 failed with %d\n", s32Ret);
		goto EXIT_UVC;
	}

	/* No rotation: VPSS outputs 1280x720 landscape matching gadget configfs */

	// hid_init();

    sem_init(&s_ExitSem, 0, 0);

    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    while((0 != sem_wait(&s_ExitSem)) && (errno == EINTR));

    return 0;

EXIT_UVC:
	UVC_Stop();
	UVC_Deinit();
	return -1;
}
