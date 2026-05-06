#!/bin/sh

cd /tmp/usb/usb_gadget/cvitek/functions/uvc.usb0
mkdir control/header/h/
echo "0x0100" > control/header/h/bcdUVC
echo "48000000" > control/header/h/dwClockFrequency
ln -s control/header/h/ control/class/fs/
ln -s control/header/h/ control/class/ss/

echo 3072 > streaming_maxpacket
echo 1 > streaming_interval

#YUV
# mkdir -p streaming/uncompressed/u/360p
# echo -e "666666\n1000000\n5000000\n" > streaming/uncompressed/u/360p/dwFrameInterval

# mkdir -p streaming/uncompressed/u/720p
# echo -e "5000000" > streaming/uncompressed/u/720p/dwFrameInterval

#MJPEG
mkdir streaming/mjpeg/m/
mkdir streaming/mjpeg/m/1440p/
echo "400000" > streaming/mjpeg/m/1440p/dwFrameInterval
echo "400000" > streaming/mjpeg/m/1440p/dwDefaultFrameInterval
echo "663552000" > streaming/mjpeg/m/1440p/dwMinBitRate
echo "3981312000" > streaming/mjpeg/m/1440p/dwMaxBitRate
echo "16588800" > streaming/mjpeg/m/1440p/dwMaxVideoFrameBufferSize
echo "2560" > streaming/mjpeg/m/1440p/wWidth
echo "1440" > streaming/mjpeg/m/1440p/wHeight

mkdir streaming/mjpeg/m/1080p/
echo "400000" > streaming/mjpeg/m/1080p/dwFrameInterval
echo "400000" > streaming/mjpeg/m/1080p/dwDefaultFrameInterval
echo "663552000" > streaming/mjpeg/m/1080p/dwMinBitRate
echo "3981312000" > streaming/mjpeg/m/1080p/dwMaxBitRate
echo "16588800" > streaming/mjpeg/m/1080p/dwMaxVideoFrameBufferSize
echo "1920" > streaming/mjpeg/m/1080p/wWidth
echo "1080" > streaming/mjpeg/m/1080p/wHeight

mkdir streaming/mjpeg/m/720p/
echo "400000" > streaming/mjpeg/m/720p/dwFrameInterval
echo "400000" > streaming/mjpeg/m/720p/dwDefaultFrameInterval
echo "663552000" > streaming/mjpeg/m/720p/dwMinBitRate
echo "3981312000" > streaming/mjpeg/m/720p/dwMaxBitRate
echo "16588800" > streaming/mjpeg/m/720p/dwMaxVideoFrameBufferSize
echo "1280" > streaming/mjpeg/m/720p/wWidth
echo "720" > streaming/mjpeg/m/720p/wHeight

mkdir streaming/mjpeg/m/360p/
echo "400000" > streaming/mjpeg/m/360p/dwFrameInterval
echo "400000" > streaming/mjpeg/m/360p/dwDefaultFrameInterval
echo "663552000" > streaming/mjpeg/m/360p/dwMinBitRate
echo "3981312000" > streaming/mjpeg/m/360p/dwMaxBitRate
echo "16588800" > streaming/mjpeg/m/360p/dwMaxVideoFrameBufferSize
echo "640" > streaming/mjpeg/m/360p/wWidth
echo "360" > streaming/mjpeg/m/360p/wHeight

#H264
mkdir streaming/framebased/fb/
mkdir streaming/framebased/fb/1440p/
echo "400000" > streaming/framebased/fb/1440p/dwFrameInterval
echo "400000" > streaming/framebased/fb/1440p/dwDefaultFrameInterval
echo "15360000" > streaming/framebased/fb/1440p/dwMinBitRate
echo "15360000" > streaming/framebased/fb/1440p/dwMaxBitRate
echo "2560" > streaming/framebased/fb/1440p/wWidth
echo "1440" > streaming/framebased/fb/1440p/wHeight

mkdir streaming/framebased/fb/1080p/
echo "400000" > streaming/framebased/fb/1080p/dwFrameInterval
echo "400000" > streaming/framebased/fb/1080p/dwDefaultFrameInterval
echo "15360000" > streaming/framebased/fb/1080p/dwMinBitRate
echo "15360000" > streaming/framebased/fb/1080p/dwMaxBitRate
echo "1920" > streaming/framebased/fb/1080p/wWidth
echo "1080" > streaming/framebased/fb/1080p/wHeight

mkdir streaming/framebased/fb/720p/
echo "400000" > streaming/framebased/fb/720p/dwFrameInterval
echo "400000" > streaming/framebased/fb/720p/dwDefaultFrameInterval
echo "15360000" > streaming/framebased/fb/720p/dwMaxBitRate
echo "15360000" > streaming/framebased/fb/720p/dwMinBitRate
echo "1280" > streaming/framebased/fb/720p/wWidth
echo "720" > streaming/framebased/fb/720p/wHeight

mkdir streaming/framebased/fb/360p/
echo "400000" > streaming/framebased/fb/360p/dwFrameInterval
echo "400000" > streaming/framebased/fb/360p/dwDefaultFrameInterval
echo "15360000" > streaming/framebased/fb/360p/dwMinBitRate
echo "15360000" > streaming/framebased/fb/360p/dwMaxBitRate
echo "640" > streaming/framebased/fb/360p/wWidth
echo "360" > streaming/framebased/fb/360p/wHeight

mkdir streaming/header/h/
# ln -s streaming/uncompressed/u streaming/header/h/
ln -s streaming/mjpeg/m/ streaming/header/h/
ln -s streaming/framebased/fb/ streaming/header/h/

ln -s streaming/header/h/ streaming/class/fs/
ln -s streaming/header/h/ streaming/class/hs/
ln -s streaming/header/h/ streaming/class/ss/

#-Create and setup configuration
cd ../../
echo "0x01" > bDeviceProtocol
echo "0x02" > bDeviceSubClass
echo "0xEF" > bDeviceClass