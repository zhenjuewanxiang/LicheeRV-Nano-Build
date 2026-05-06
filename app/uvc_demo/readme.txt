以1811c_spinand为例，在build/boards/cv181x/cv1811c_wevb_0006a_spinand/linux/cvitek_cv1811c_wevb_0006a_spinand_defconfig配置文件添加以下配置；
注意文件里若有同一个配置项的请删掉；

# uvc support
CONFIG_USB_GADGET=y
CONFIG_USB_CONFIGFS=y
CONFIG_USB_LIBCOMPOSITE=y
CONFIG_USB_ROLE_SWITCH=y

CONFIG_VIDEO_DEV=y
CONFIG_VIDEO_V4L2=y
CONFIG_MEDIA_CAMERA_SUPPORT=y
CONFIG_MEDIA_SUPPORT=y
CONFIG_VIDEOBUF2_CORE=y
CONFIG_VIDEOBUF2_MEMOPS=y
CONFIG_VIDEOBUF2_V4L2=y
CONFIG_VIDEOBUF2_VMALLOC=y

CONFIG_USB_F_UVC=y
CONFIG_USB_CONFIGFS_F_UVC=y

进入系统后，配置device
echo device > /proc/cviusb/otg_role

执行sample_uvc_single