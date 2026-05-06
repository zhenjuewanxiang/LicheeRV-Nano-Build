#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
// #include <unistd.h>
#include <fcntl.h>

#include "sample_comm.h"
#include "hid_ctl.h"
#include "cvi_uvc.h"

void deal_uvc_event(hid_uvc_event event) {
    switch (event.type) {
    case UVC_ROTATION:
        UVC_SetRotation(event.value, event.uvc_fd_index);
        break;
    case UVC_SET_FRAME_RATE:
        UVC_SetFrameRate(event.value, event.uvc_fd_index);
        break;
    default:
        break;
    }
}

/***
 * Send data to host.
 * @hid_fd: Hid device file.
 * @hid_user_layer_message: Data buff.
 * @msg_size: Data size.
 * @return: Bytes recerived.
 * ***/
CVI_S32 hid_recv_data(int hid_fd, hid_uvc_event* hid_user_layer_message, unsigned long msg_size) {
    CVI_S32 count = 0;

    count = read(hid_fd, hid_user_layer_message, msg_size);
    printf("hid received %d bytes data.\n", count);
    return count;
} 

CVI_S32 hid_response_data(int hid_fd, void *resp, int size) {
    CVI_S32 count = 0;

    count = write(hid_fd, resp, size);
    printf("hid sent %d bytes data.\n", count);
    return count;
} 

void* hid_receive_func(CVI_VOID* arg) {
    hid_uvc_event hid_receive_uvc_event;
    CVI_S32 s32_ret = CVI_SUCCESS;
    int hid_fd = 0;

    UNUSED(arg);

    hid_fd = open(HID_DEV_NAME, O_RDWR);
    if (hid_fd < 0) {
        fprintf(stderr, "Can't open file %s.\n", HID_DEV_NAME);
        return NULL;
    }

    // // set no buffer for hid character device, it is very very important!!!
    // setvbuf(p_hid_file, NULL, _IONBF, 0);
    
    while (1) {
        s32_ret = hid_recv_data(hid_fd, &hid_receive_uvc_event, sizeof(hid_receive_uvc_event));
        if (s32_ret == 0) {
            printf("HID none data!\n");
            continue;
        } else if (s32_ret != sizeof(hid_receive_uvc_event)) {
            fprintf(stderr, "HID recv data failed!\n");
            continue;
        }
        printf("process hid_receive_uvc_event...\n");    
        deal_uvc_event(hid_receive_uvc_event);
        printf("process hid_receive_uvc_event done!\n");
    }

    return NULL;
}


CVI_VOID hid_init(CVI_VOID) {
    CVI_S32 s32_ret = CVI_SUCCESS;
    pthread_t s_device_recv_thread;

    s32_ret = pthread_create(&s_device_recv_thread, NULL, hid_receive_func, NULL);
    if (s32_ret != CVI_SUCCESS) {
        return;
    }
    printf("hid_init.\n");

    pthread_join(s_device_recv_thread, NULL);
}