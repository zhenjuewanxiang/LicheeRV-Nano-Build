/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: sample_hid.h
 * Description:
 */
#include <linux/cvi_common.h>
#include <linux/videodev2.h>
#include "uvc.h"

#ifndef __HID_CTL_H__
#define __HID_CTL_H__

#define HID_VENDOR_ID       0x3346
#define HID_PRODUCT_ID      0x100A 

#define UVC_ROTATION 		1
#define UVC_SET_FRAME_RATE 		2


typedef struct {
    __u32 type;
    __u32 uvc_fd_index;
    __u32 value;
} hid_uvc_event;

enum hid_response_status {
    SUCCESS = 0,
    FAILED
};

#define HID_DEV_NAME  "/dev/hidg0"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

CVI_VOID hid_init(CVI_VOID);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* End of #ifndef __SAMPLE_HID_H__*/

