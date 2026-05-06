#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/prctl.h>

#include "cvi_uvc.h"
#include "cvi_uvc_gadget.h"
#include "cvi_system.h"
#include "sample_comm.h"

#define     CVI_KOMOD_PATH              "/mnt/system/ko"
#define     CVI_UVC_SCRIPTS_PATH        "/etc"

CVI_U8 dataptr[2000000];

/** UVC Stream Context */
typedef struct tagUVC_STREAM_CONTEXT_S {
    CVI_UVC_DEVICE_CAP_S stDeviceCap;
    CVI_UVC_DATA_SOURCE_S stDataSource;
    UVC_STREAM_ATTR_S stStreamAttr; /**<stream attribute, update by uvc driver */
    bool bVcapsStart;
    bool bVencStart;
    bool bFirstFrame;
    bool bInited[UVC_CAMERA_NUM_MAX];
} UVC_STREAM_CONTEXT_S;
static UVC_STREAM_CONTEXT_S s_stUVCStreamCtx;

int32_t UVC_STREAM_SetAttr(UVC_STREAM_ATTR_S *pstAttr, uint8_t fd_index)
{
    s_stUVCStreamCtx.stStreamAttr = *pstAttr;
    printf("Format: %d, Resolution: %ux%u, FPS: %u, BitRate: %u", pstAttr->enFormat, pstAttr->u32Width,
             pstAttr->u32Height, pstAttr->u32Fps, pstAttr->u32BitRate);

    s_stUVCStreamCtx.bInited[fd_index] = false;
    s_stUVCStreamCtx.bVcapsStart = false;
    s_stUVCStreamCtx.bVencStart = false;
    s_stUVCStreamCtx.bFirstFrame = false;

    return 0;
}

static int init_vproc(int fd_index) {
    UVC_STREAM_ATTR_S *pAttr = &s_stUVCStreamCtx.stStreamAttr;
    CVI_UVC_DATA_SOURCE_S *pstSrc = &s_stUVCStreamCtx.stDataSource;
    VPSS_CHN_ATTR_S stChnAttr;

	CVI_VPSS_GetChnAttr(pstSrc->VprocHdl[fd_index], pstSrc->VprocChnId, &stChnAttr);
	// stChnAttr.u32Width = pAttr->u32Width;
	// stChnAttr.u32Height = pAttr->u32Height;
    stChnAttr.u32Width = pAttr->u32Height;
	stChnAttr.u32Height = pAttr->u32Width;
	CVI_VPSS_SetChnAttr(pstSrc->VprocHdl[fd_index], pstSrc->VprocChnId, &stChnAttr);

    return 0;
}

static void _initInputCfg(chnInputCfg *ipIc)
{
	memset(ipIc, 0, sizeof(chnInputCfg));
	ipIc->rcMode = -1;
	ipIc->iqp = -1;
	ipIc->pqp = -1;
	ipIc->gop = -1;
	ipIc->bitrate = -1;
	ipIc->firstFrmstartQp = -1;
	ipIc->num_frames = -1;
	ipIc->framerate = 30;
	ipIc->maxQp = -1;
	ipIc->minQp = -1;
	ipIc->maxIqp = -1;
	ipIc->minIqp = -1;
}

static int init_venc(uint8_t fd_index) {
    UVC_STREAM_ATTR_S *pAttr = &s_stUVCStreamCtx.stStreamAttr;
    CVI_UVC_DATA_SOURCE_S *pstSrc = &s_stUVCStreamCtx.stDataSource;

	int32_t s32Ret = 0;
	PIC_SIZE_E enSize = PIC_1440P;
    chnInputCfg pIc;
	VENC_GOP_MODE_E enGopMode = VENC_GOPMODE_NORMALP;
	VENC_GOP_ATTR_S stGopAttr;
	SAMPLE_RC_E enRcMode;
	CVI_U32 u32Profile = 0;
	PAYLOAD_TYPE_E enPayLoad = PT_H264;

    _initInputCfg(&pIc);
    if (pAttr->enFormat == CVI_UVC_STREAM_FORMAT_MJPEG) {
        enPayLoad = PT_MJPEG;
    } else if (pAttr->enFormat == CVI_UVC_STREAM_FORMAT_H264) {
        enPayLoad = PT_H264;
    }

    // if (pAttr->u32Height == 1440) {
    //     enSize = PIC_1440P;
    //     pIc.quality = 20;
    // } else if (pAttr->u32Height == 1080) {
    //     enSize = PIC_1080P;
    //     pIc.quality = 40;
    // } else if (pAttr->u32Height == 720) {
    //     enSize = PIC_720P;
    //     pIc.quality = 60;
    // } else if (pAttr->u32Height == 360) {
    //     enSize = PIC_CUSTOMIZE;
    //     pIc.width = 640;
    //     pIc.height = 360;
    //     pIc.quality = 70;
    // } else {
    //     printf("\n todo venc size\n %s %d\n", __func__, __LINE__);
    // }

    enSize = PIC_CUSTOMIZE;
    pIc.width = 720;
    pIc.height = 1280;
    pIc.quality = 60;
    printf("[%s]enSize:%d\n", __func__, enSize);

	strcpy(pIc.codec, (enPayLoad == PT_MJPEG) ? "mjp" : (enPayLoad == PT_H264) ? "264" : "265");
	// pIc.rcMode = (enPayLoad == PT_MJPEG) ? SAMPLE_RC_FIXQP : SAMPLE_RC_CBR;
    pIc.rcMode = SAMPLE_RC_CBR;
	enRcMode = (SAMPLE_RC_E) pIc.rcMode;

    if(enPayLoad == PT_MJPEG){
        pIc.iqp = 38;
        pIc.pqp = 38;
        pIc.gop = 60;
        pIc.bitrate = 20000;
        pIc.firstFrmstartQp = 26;
        pIc.num_frames = -1;
        pIc.srcFramerate = 30;
        pIc.framerate = 30;
        pIc.maxQp = 42;
        pIc.minQp = 26;
        pIc.maxIqp = 42;
        pIc.minIqp = 26;
        pIc.minIprop = 1;
        pIc.maxIprop = 100;
        pIc.initialDelay = 1000;
        pIc.bitstreamBufSize = 1024 * 1024;
    }
    else{
        pIc.iqp = 38;
        pIc.pqp = 38;
        pIc.gop = 50;
        pIc.statTime = 2;
        pIc.s32IPQpDelta = 2;
        pIc.bitrate = 4000;
        pIc.firstFrmstartQp = 35;
        pIc.num_frames = -1;
        pIc.srcFramerate = 30;
        pIc.framerate = 30;
        pIc.maxQp = 51;
        pIc.minQp = 20;
        pIc.maxIqp = 51;
        pIc.minIqp = 20;
        pIc.minIprop = 1;
        pIc.maxIprop = 10;
        pIc.s32BgDeltaQp = 2;
        pIc.u32RowQpDelta = CVI_H26X_ROW_QP_DELTA_DEFAULT;
        pIc.u32ThrdLv = 2;
        pIc.initialDelay = 1000;
    }

    // crop 768 to 720
    pIc.posX = ALIGN(720, 64) - 720;
	pIc.posY = 0;
	pIc.width = 720;
	pIc.height = 1280;

	s32Ret = SAMPLE_COMM_VENC_GetGopAttr(enGopMode, &stGopAttr);
	if (s32Ret != 0) {
		printf("[Err]Venc Get GopAttr for %#x!\n", s32Ret);
		return CVI_FAILURE;
	}

	s32Ret = SAMPLE_COMM_VENC_Start(
			&pIc,
			pstSrc->VencHdl[fd_index],
			enPayLoad,
			enSize,
			enRcMode,
			u32Profile,
			CVI_FALSE,
			&stGopAttr);
	if (s32Ret != 0) {
		printf("[Err]Venc Start failed for %#x!\n", s32Ret);
		return CVI_FAILURE;
	}

	return s32Ret;
}

int32_t UVC_STREAM_Start(int fd_index) {

    struct timeval tv;
    

    printf("UVC_STREAM_Start....\n");
    if (!s_stUVCStreamCtx.bInited[fd_index])
	{   
        printf("init_vproc....\n");
        if (0 != init_vproc(fd_index))
		{
            printf("init_vproc failed !");
            return -1;
        }
        gettimeofday(&tv, NULL);
        printf("init_vproc done...., timestap: %lds-%ldus\n", tv.tv_sec, tv.tv_usec);

        printf("init_venc....\n");
        if (0 != init_venc(fd_index))
		{
            printf("init_venc failed !");
            return -1;
        }
        gettimeofday(&tv, NULL);
        printf("init_venc done...., timestap: %lds-%ldus\n", tv.tv_sec, tv.tv_usec);
        s_stUVCStreamCtx.bInited[fd_index] = true;
    }

    return 0;
}

int32_t UVC_STREAM_Stop(int fd_index)
{
    if (s_stUVCStreamCtx.bInited[fd_index])
	{
        CVI_UVC_DATA_SOURCE_S *pstSrc = &s_stUVCStreamCtx.stDataSource;
        SAMPLE_COMM_VENC_Stop(pstSrc->VencHdl[fd_index]);
        printf("SAMPLE_COMM_VENC_Stop %d\n", fd_index);

        s_stUVCStreamCtx.bInited[fd_index] = false;
    }
    return 0;
}

int32_t UVC_STREAM_CopyBitStream(void *dst, uint8_t fd_index) {
    int32_t s32Ret = 0;
    CVI_UVC_DATA_SOURCE_S *pstSrc = &s_stUVCStreamCtx.stDataSource;
    VIDEO_FRAME_INFO_S venc_frame;
    VENC_STREAM_S stStream = {0};

    // VIDEO_FRAME_INFO_S vpss_frame1, vpss_frame2;
    // s32Ret = CVI_VPSS_GetChnFrame(0, 1, &vpss_frame1, -1);
    // if (s32Ret != 0) {
    //     printf("CVI_VPSS_GetChnFrame1 failed! %d\n", s32Ret);
    //     return -1;
    // }

    // s32Ret = CVI_VPSS_GetChnFrame(1, 0, &vpss_frame2, -1);
    // if (s32Ret != 0) {
    //     printf("CVI_VPSS_GetChnFrame2 failed! %d\n", s32Ret);
    //     return -1;
    // }

    s32Ret = CVI_VPSS_GetChnFrame(pstSrc->VprocHdl[fd_index], pstSrc->VprocChnId, &venc_frame, -1);
    if (s32Ret != 0) {
        printf("CVI_VPSS_GetChnFrame failed! %d\n", s32Ret);
        return -1;
    }

    int32_t s32SetFrameMilliSec = 20000;
    VENC_CHN_ATTR_S stVencChnAttr;
    VENC_CHN_STATUS_S stStat;

    s32Ret = CVI_VENC_SendFrame(pstSrc->VencHdl[fd_index], &venc_frame, s32SetFrameMilliSec);
    if (s32Ret != 0) {
        printf("CVI_VENC_SendFrame failed! %d\n", s32Ret);
        return -1;
    }

    s32Ret = CVI_VENC_GetChnAttr(pstSrc->VencHdl[fd_index], &stVencChnAttr);
    if (s32Ret != 0) {
        printf("CVI_VENC_GetChnAttr, VencChn[%d], s32Ret = %d\n", pstSrc->VprocHdl[fd_index], s32Ret);
        return -1;
    }

    s32Ret = CVI_VENC_QueryStatus(pstSrc->VencHdl[fd_index], &stStat);
    if (s32Ret != 0) {
        printf("CVI_VENC_QueryStatus failed with %#x!\n", s32Ret);
        return -1;
    }

    if (!stStat.u32CurPacks) {
        printf("NOTE: Current frame is NULL!\n");
        return -1;
    }

    stStream.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
    if (stStream.pstPack == NULL) {
        printf("malloc memory failed!\n");
        return -1;
    }

    s32Ret = CVI_VENC_GetStream(pstSrc->VencHdl[fd_index], &stStream, -1);
    if (s32Ret != 0) {
        printf("CVI_VENC_GetStream failed with %#x!\n", s32Ret);
        free(stStream.pstPack);
        stStream.pstPack = NULL;
        return -1;
    }

    uint32_t bitstream_size = 0;
    for (unsigned i = 0; i < stStream.u32PackCount; i++) {
        VENC_PACK_S *ppack;
        ppack = &stStream.pstPack[i];

        memcpy(dst + bitstream_size, ppack->pu8Addr + ppack->u32Offset, ppack->u32Len - ppack->u32Offset);
        bitstream_size += ppack->u32Len - ppack->u32Offset;
    }

    s32Ret = CVI_VENC_ReleaseStream(pstSrc->VencHdl[fd_index], &stStream);
    if (s32Ret != 0) {
        printf("CVI_VENC_ReleaseStream, s32Ret = %d\n", s32Ret);
        free(stStream.pstPack);
        stStream.pstPack = NULL;
        return -1;
    }

    free(stStream.pstPack);
    stStream.pstPack = NULL;

    CVI_VPSS_ReleaseChnFrame(pstSrc->VprocHdl[fd_index], pstSrc->VprocChnId, &venc_frame);

    // int i;
    // int count = 0;
    // for (i = 0; i < 2; i++) {
    //     vpss_frame1.stVFrame.pu8VirAddr[i] = (CVI_U8*)CVI_SYS_Mmap(vpss_frame1.stVFrame.u64PhyAddr[i], vpss_frame1.stVFrame.u32Length[i]);
    //     memcpy(&dataptr[count], vpss_frame1.stVFrame.pu8VirAddr[i], vpss_frame1.stVFrame.u32Length[i]);
    //     count = count + vpss_frame1.stVFrame.u32Length[i];
    //     CVI_SYS_Munmap(vpss_frame1.stVFrame.pu8VirAddr[i], vpss_frame1.stVFrame.u32Length[i]);
    // }

    // count = 0;
    // for (i = 0; i < 2; i++) {
    //     vpss_frame2.stVFrame.pu8VirAddr[i] = (CVI_U8*)CVI_SYS_Mmap(vpss_frame2.stVFrame.u64PhyAddr[i], vpss_frame2.stVFrame.u32Length[i]);
    //     memcpy(&dataptr[count], vpss_frame2.stVFrame.pu8VirAddr[i], vpss_frame2.stVFrame.u32Length[i]);
    //     count = count + vpss_frame2.stVFrame.u32Length[i];
    //     CVI_SYS_Munmap(vpss_frame2.stVFrame.pu8VirAddr[i], vpss_frame2.stVFrame.u32Length[i]);
    // }

    // CVI_VPSS_ReleaseChnFrame(0, 1, &vpss_frame1);
    // CVI_VPSS_ReleaseChnFrame(1, 0, &vpss_frame2);

    return bitstream_size;
}

int32_t UVC_STREAM_ReqIDR(void) {
    // TODO: implement
    return 0;
}

/** UVC Context */
static UVC_CONTEXT_S s_stUVCCtx = {.bRun = false, .bPCConnect = false, .TskId = {(pthread_t)-1}};

bool g_bPushVencData = false;

static void *UVC_CheckTask(void *pvArg) {
    int fd_index = (uintptr_t)pvArg;
    int32_t ret = 0;
    
    prctl(PR_SET_NAME, "cvitask_uvc", 0, 0, 0);

    while (s_stUVCCtx.bRun) {
        ret = UVC_GADGET_DeviceCheck(fd_index);

        if (ret < 0) {
            printf("UVC_GADGET_DeviceCheck %x\n", ret);
            break;
        } else if (ret == 0) {
            printf("Timeout Do Nothing\n");
            if (false != g_bPushVencData) {
                g_bPushVencData = false;
            }
        }
    }


    return NULL;
}

static int32_t UVC_LoadMod(void) {
    static bool first = true;
    if(first == false) {
        return 0;
    }
    first = false;
    printf("Uvc insmod ko successfully!");
    // cvi_insmod(CVI_KOMOD_PATH"/videobuf2-memops.ko", NULL);
    cvi_system("echo 449 >/sys/class/gpio/export");
    cvi_system("echo 450 >/sys/class/gpio/export");

    cvi_system("echo \"out\" >/sys/class/gpio/gpio449/direction");
    cvi_system("echo \"out\" >/sys/class/gpio/gpio450/direction");

    cvi_system("echo 0 >/sys/class/gpio/gpio449/value");
    cvi_system("echo 1 >/sys/class/gpio/gpio450/value");

    cvi_insmod(CVI_KOMOD_PATH"/usbcore.ko", NULL);
    cvi_insmod(CVI_KOMOD_PATH"/dwc2.ko", NULL);
    cvi_insmod(CVI_KOMOD_PATH"/configfs.ko", NULL);
    cvi_insmod(CVI_KOMOD_PATH"/libcomposite.ko", NULL);
    cvi_insmod(CVI_KOMOD_PATH"/videobuf2-vmalloc.ko", NULL);
    cvi_insmod(CVI_KOMOD_PATH"/usb_f_uvc.ko", NULL);
    cvi_insmod(CVI_KOMOD_PATH"/u_audio.ko", NULL);
    cvi_insmod(CVI_KOMOD_PATH"/usb_f_uac1.ko", NULL);
    cvi_system("echo device > /proc/cviusb/otg_role");
    cvi_system(CVI_UVC_SCRIPTS_PATH"/run_usb.sh probe uvc");
    // cvi_system(CVI_UVC_SCRIPTS_PATH"/run_usb.sh probe hid");
    cvi_system("./ConfigUVC.sh");
    cvi_system(CVI_UVC_SCRIPTS_PATH"/run_usb.sh start");
    cvi_system("devmem 0x030001DC 32 0x8844");
    return 0;
}

static int32_t UVC_UnLoadMod(void) {
    // printf("Do nothing now, due to the ko can NOT rmmod successfully!");
    // cvi_rmmod(CVI_KOMOD_PATH "/videobuf2-memops.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/videobuf2-vmalloc.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/configfs.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/libcomposite.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/u_serial.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_acm.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/cvi_usb_f_cvg.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_uvc.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/u_audio.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_uac1.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_serial.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_mass_storage.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/u_ether.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_ecm.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_eem.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/usb_f_rndis.ko");
    // cvi_rmmod(CVI_KOMOD_PATH "/cv183x_usb_gadget.ko");

    return 0;
}

int32_t UVC_Init(const CVI_UVC_DEVICE_CAP_S *pstCap, const CVI_UVC_DATA_SOURCE_S *pstDataSrc,
                 CVI_UVC_BUFFER_CFG_S *pstBufferCfg) {

    UVC_LoadMod();

    s_stUVCStreamCtx.stDeviceCap = *pstCap;
    s_stUVCStreamCtx.stDataSource = *pstDataSrc;
    UVC_GADGET_Init(pstCap, pstBufferCfg->u32BufSize);

    // TODO: Do we need handle CVI_UVC_BUFFER_CFG_S?
    return 0;
}

int32_t UVC_Deinit(void) {
    UVC_UnLoadMod(); // TODO, Not work right now

    return 0;
}

int32_t UVC_Start(const char *pDevPath)
{
	uint8_t i;
    uint8_t dev_num = 0;
	const char *devpath = pDevPath;

    if (false == s_stUVCCtx.bRun) {

		for (i = 0; i < UVC_CAMERA_NUM_MAX; i++)
		{
			if (*devpath == '\0')
				break;

			strcpy(s_stUVCCtx.szDevPath, devpath);

			if (UVC_GADGET_DeviceOpen(devpath, i)) {
				printf("UVC_GADGET_DeviceOpen Failed!");
				return -1;
			}
			devpath += 32;
            dev_num += 1;
		}

		s_stUVCCtx.bPCConnect = false;
		s_stUVCCtx.bRun = true;

        printf("dev_num: %d\n", dev_num);

		for (i = 0; i < dev_num; ++i) {
            if (pthread_create(&s_stUVCCtx.TskId[i], NULL, UVC_CheckTask, (void *)(uintptr_t)i)) {
                printf("UVC_CheckTask create thread failed!\n");
                s_stUVCCtx.bRun = false;
                return -1;
            }

            usleep(100 * 1000);

            struct sched_param param;
            param.sched_priority = 95;
            printf("sched_priority = %d\n", param.sched_priority);
            if (pthread_setschedparam(s_stUVCCtx.TskId[i], SCHED_RR, &param) != 0) {
                printf("pthread_setschedparam failed\n");
                return -1;
            }
        }
		printf("UVC_CheckTask create thread successful\n");
    }
	else
	{
        printf("UVC already started\n");
    }

    return 0;
}

int32_t UVC_Stop(void) {
    int i;
    if (false == s_stUVCCtx.bRun) {
        printf("UVC not run\n");
        return 0;
    }

    s_stUVCCtx.bRun = false;
    for (i = 0; i < UVC_CAMERA_NUM_MAX; ++i) {
        if (s_stUVCCtx.TskId[i] != (pthread_t)-1) {
            pthread_join(s_stUVCCtx.TskId[i], NULL);
        }
    }
    
    return UVC_GADGET_DeviceClose();
}

int32_t UVC_SetRotation(int value, int fd_index) {
    UVC_STREAM_ATTR_S *pAttr = &s_stUVCStreamCtx.stStreamAttr;
    CVI_UVC_DATA_SOURCE_S *pstSrc = &s_stUVCStreamCtx.stDataSource;
    VPSS_CHN_ATTR_S vpssChnAttr;
    VENC_CHN_ATTR_S vencChnAttr;
    VENC_CHN_PARAM_S vencChnParam;
    ROTATION_E enrotation = UVC_ROTATION_0;
    bool mirror = false;
    bool flip = false;
    int32_t width = pAttr->u32Width;
    int32_t height = pAttr->u32Height;
    int32_t crop_x = 0;
    int32_t crop_y = 0;
    int32_t ret = 0;

    switch (value)
    {
    case UVC_ROTATION_0:
        enrotation = ROTATION_0;
        break;
    case UVC_ROTATION_90:
        //change H × W to W × H
        height = pAttr->u32Width;
        width = pAttr->u32Height;
        enrotation = ROTATION_90;

        crop_x = ALIGN(width, 64) - width;
        break;
    case UVC_ROTATION_180:
        mirror = true;
        flip = true;
        enrotation = ROTATION_0;
        break;
    case UVC_ROTATION_270:
        height = pAttr->u32Width;
        width = pAttr->u32Height;
        enrotation = ROTATION_270;

        crop_y = ALIGN(height, 64) - height;
        break;
    
    default:
        return -1;
    }

    printf("UVC rotation: %d\n", enrotation);
	ret = CVI_VPSS_SetChnRotation(pstSrc->VprocHdl[fd_index], pstSrc->VprocChnId, enrotation);
    if (ret != CVI_SUCCESS) {
        printf("UVC rotation failed: %x\n", ret);
    }

    // printf("UVC rotation: %d\n", enrotation);
	// ret = CVI_VI_SetChnRotation(pstSrc->VcapHdl, fd_index, enrotation);
    // if (ret != CVI_SUCCESS) {
    //     printf("UVC rotation failed: %x\n", ret);
    // }

    CVI_VPSS_GetChnAttr(pstSrc->VprocHdl[fd_index], pstSrc->VprocChnId, &vpssChnAttr);
	vpssChnAttr.bMirror = mirror;
	vpssChnAttr.bFlip = flip;
    // vpssChnAttr.u32Width = ALIGN(width, 64);
	// vpssChnAttr.u32Height = ALIGN(height, 64);
	CVI_VPSS_SetChnAttr(pstSrc->VprocHdl[fd_index], pstSrc->VprocChnId, &vpssChnAttr);

    CVI_VENC_GetChnAttr(pstSrc->VencHdl[fd_index], &vencChnAttr);
    vencChnAttr.stVencAttr.u32PicWidth = width;
    vencChnAttr.stVencAttr.u32PicHeight = height;
    vencChnAttr.stVencAttr.u32MaxPicWidth = width;
    vencChnAttr.stVencAttr.u32MaxPicHeight = height;
    CVI_VENC_SetChnAttr(pstSrc->VencHdl[fd_index], &vencChnAttr);

    CVI_VENC_GetChnParam(pstSrc->VencHdl[fd_index], &vencChnParam);
    VENC_CROP_INFO_S * pcropCfg = &vencChnParam.stCropCfg;
    pcropCfg->bEnable = true;
    pcropCfg->stRect.s32X = crop_x;
    pcropCfg->stRect.s32Y = crop_y;
    pcropCfg->stRect.u32Width = width;
    pcropCfg->stRect.u32Height = height;
    CVI_VENC_SetChnParam(pstSrc->VencHdl[fd_index], &vencChnParam);

    return 0;
}

int32_t UVC_SetFrameRate(int value, int fd_index) {
    ISP_PUB_ATTR_S pubAttr;;
    int32_t fps = 30;
    int32_t ret = 0;

    switch (value)
    {
    case UVC_FRAME_RATE_5:
        fps = 5;
        break;
    case UVC_FRAME_RATE_10:
        fps = 10;
        break;
    case UVC_FRAME_RATE_15:
        fps = 15;;
        break;
    case UVC_FRAME_RATE_20:
        fps = 20;
        break;
    case UVC_FRAME_RATE_25:
        fps = 25;
        break;
    default:
        return -1;
    }

    printf("UVC set frame rate: %d fps\n", fps);

	CVI_ISP_GetPubAttr(fd_index, &pubAttr);
	pubAttr.f32FrameRate = fps;
	ret = CVI_ISP_SetPubAttr(fd_index, &pubAttr);
    if (ret != CVI_SUCCESS) {
        printf("UVC rotation failed: %x\n", ret);
    }

    return 0;
}

UVC_CONTEXT_S *UVC_GetCtx(void) { return &s_stUVCCtx; }
