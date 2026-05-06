#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <semaphore.h>
#include <errno.h>

#include "cvi_uvc.h"
#include "cvi_uvc_gadget.h"
#include "cvi_system.h"
#include "sample_comm.h"
#include "hid_ctl.h"

static sem_t s_ExitSem;/** EXIT semaphore */
static SAMPLE_VI_CONFIG_S stViConfig;

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
	SAMPLE_INI_CFG_S	   stIniCfg = {
		.enSource  = VI_PIPE_FRAME_SOURCE_DEV,
		.devNum    = 1,
		.enSnsType[0] = SONY_IMX327_MIPI_2M_30FPS_12BIT,
		.enWDRMode[0] = WDR_MODE_NONE,
		.s32BusId[0]  = 3,
		.s32SnsI2cAddr[0] = -1,
		.MipiDev[0]   = 0xFF,
		.enSnsType[1] = SONY_IMX327_SLAVE_MIPI_2M_30FPS_12BIT,
		.s32BusId[1]  = 3,
		.s32SnsI2cAddr[1] = -1,
		.MipiDev[1]   = 0xFF,
		.u8UseMultiSns = 0,
	};

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
	if (s32Ret != CVI_SUCCESS)
		return s32Ret;

	/************************************************
	 * step2:  Get input size
	 ************************************************/
	s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stIniCfg.enSnsType[0], &enPicSize);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_VI_GetSizeBySensor failed with %#x\n", s32Ret);
		return s32Ret;
	}

	s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, pstSize);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_SYS_GetPicSize failed with %#x\n", s32Ret);
		return s32Ret;
	}

	printf("SAMPLE_COMM_SYS_GetPicSize %dx%d\n", pstSize->u32Width, pstSize->u32Height);

	/************************************************
	 * step3:  Init VB pool
	 ************************************************/
	VB_CONFIG_S stVbConf;
	// CVI_U8 i = 0;
	CVI_U32 u32BlkSize = 0;

	memset(&stVbConf, 0, sizeof(VB_CONFIG_S));
	stVbConf.u32MaxPoolCnt = 1;

	u32BlkSize = COMMON_GetPicBufferSize(ALIGN(pstSize->u32Width, 64), ALIGN(pstSize->u32Height, 64), SAMPLE_PIXEL_FORMAT,
		DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
	stVbConf.astCommPool[0].u32BlkCnt = 18;
	stVbConf.astCommPool[0].enRemapMode = VB_REMAP_MODE_CACHED;

	// u32BlkSize = COMMON_GetPicBufferSize(1280, 768, SAMPLE_PIXEL_FORMAT,
	// 	DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	// stVbConf.astCommPool[1].u32BlkSize = u32BlkSize;
	// stVbConf.astCommPool[1].u32BlkCnt = 12;
	// stVbConf.astCommPool[1].enRemapMode = VB_REMAP_MODE_CACHED;


	// for (int e = 0, i = 1; e < CUR_ENCODE_NUM; i++, e++) {
	// 	SIZE_S stSize;
	// 	CVI_BOOL bRepeated = CVI_FALSE;

	// 	s32Ret = SAMPLE_COMM_SYS_GetPicSize(mySize[e], &stSize);
	// 	if (s32Ret != CVI_SUCCESS) {
	// 		CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_SYS_GetPicSize failed with %#x\n", s32Ret);
	// 		return s32Ret;
	// 	}
	// 	printf("SAMPLE_COMM_SYS_GetPicSize %dx%d\n", stSize.u32Width, stSize.u32Height);
	// 	u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height, PIXEL_FORMAT_YUV_PLANAR_420,
	// 		DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	// 	for (int j = 0; j < i; j++) {
	// 		if (u32BlkSize == stVbConf.astCommPool[j].u32BlkSize) {
	// 			stVbConf.astCommPool[j].u32BlkCnt += 2;
	// 			bRepeated = CVI_TRUE;
	// 			break;
	// 		}
	// 	}
	// 	if (bRepeated) {
	// 		i--;
	// 		continue;
	// 	}
	// 	stVbConf.astCommPool[i].u32BlkSize = u32BlkSize;
	// 	stVbConf.astCommPool[i].u32BlkCnt = 2;
	// 	stVbConf.astCommPool[i].enRemapMode = VB_REMAP_MODE_CACHED;
	// 	stVbConf.u32MaxPoolCnt++;
	// }
	// for (i = 0; i < stVbConf.u32MaxPoolCnt; i++) {
	// 	printf("common pool[%d] BlkSize %d * %d\n"
	// 		, i, stVbConf.astCommPool[i].u32BlkSize, stVbConf.astCommPool[i].u32BlkCnt);
	// }

	// VI_CROP_INFO_S stCropInfo;
	// stCropInfo.bEnable = 1;
	// stCropInfo.enCropCoordinate = VI_CROP_ABS_COOR;
	// stCropInfo.stCropRect.s32X = (1920 - 1280) / 2; 
	// stCropInfo.stCropRect.s32Y = (1080 - 720) / 2; 
	// stCropInfo.stCropRect.u32Width = 1280;
	// stCropInfo.stCropRect.u32Height = 720;

	// CVI_VI_SetChnCrop(0, 0, &stCropInfo);
	// CVI_VI_SetChnCrop(1, 1, &stCropInfo);

	s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "sys init failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	/************************************************
	 * step4:  Init VI modules
	 ************************************************/
	if (stIniCfg.enSource == VI_PIPE_FRAME_SOURCE_DEV) {
		s32Ret = SAMPLE_PLAT_VI_INIT(&stViConfig);
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

	UNUSED(argc);
	UNUSED(argv);

	// start Vi
	s32Ret = _CVI_Init_Vi(&stSize);
	if (s32Ret != CVI_SUCCESS) {
		printf("_CVI_Init_Vi fail, %d\n", s32Ret);
		return -1;
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
		return -1;
	}

	s32Ret = _CVI_Init_Vpss(1, 1, &stVpssGrpAttr);
	if (s32Ret != CVI_SUCCESS) {
		SAMPLE_PRT("CVI_Init_Video_Process Grp0 failed with %d\n", s32Ret);
		return -1;
	}

	CVI_VPSS_SetChnRotation(0, 0, ROTATION_270);
	CVI_VPSS_SetChnRotation(0, 1, ROTATION_270);
	CVI_VPSS_SetChnRotation(1, 0, ROTATION_270);

	CVI_UVC_DEVICE_CAP_S stDeviceCap = {0};
	CVI_UVC_DATA_SOURCE_S stDataSource = {0};
	CVI_UVC_BUFFER_CFG_S stBuffer = {0};
	stDataSource.AcapHdl = 0;
	stDataSource.VcapHdl = 0;
	for (i = 0; i < UVC_CAMERA_NUM_MAX; i++)
	{
		stDataSource.VprocHdl[i] = i;
		stDataSource.VencHdl[i] = i;
	}
	stDataSource.VprocChnId = 0;

    if (UVC_Init(&stDeviceCap, &stDataSource, &stBuffer) != 0) {
        printf("UVC_Init Failed !");
        return -1;
    }

    const char uvc_devname[UVC_CAMERA_NUM_MAX][32] = {
		{"/dev/video0"},
		// {"/dev/video1"},
		{ '\0' },
	};

    if (UVC_Start((const char *)uvc_devname) != 0) {
        printf("UVC_Start Failed !");
        return -1;
    }

	// hid_init();

    sem_init(&s_ExitSem, 0, 0);

    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    while((0 != sem_wait(&s_ExitSem)) && (errno == EINTR));

    return 0;
}
