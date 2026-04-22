SHELL = /bin/bash

# Point to middleware/v2/Makefile.param (two levels up from app/stream_demo/)
ifeq ($(PARAM_FILE), )
	PARAM_FILE := ../../middleware/v2/Makefile.param
	include $(PARAM_FILE)
endif

SDIR := $(CURDIR)
MW_SAMPLE := $(realpath $(SDIR)/../../middleware/v2/sample)
MEDIA_SERVER := $(MW_SAMPLE)/test_mmf/media_server-1.0.1
MAIX_MMF_DIR := $(MW_SAMPLE)/test_mmf

TARGET := stream_demo

# Source files
SRCS := \
	$(SDIR)/stream_demo.c \
	$(MEDIA_SERVER)/libmov/test/mov-file-buffer.c

# http-parser compiled separately with NDEBUG to disable asserts
HTTP_PARSER_SRC := $(MEDIA_SERVER)/sdk/libhttp/source/http-parser.c
HTTP_PARSER_OBJ := $(SDIR)/http-parser.o

# Recompile h265-camera-source and rtsp-server from source
SRCS_CPP := \
	$(MEDIA_SERVER)/media/h265-camera-source.cpp \
	$(MEDIA_SERVER)/rtsp-server.cpp

# Prebuilt .o files from test_mmf (matched to prebuilt libs)
PREBUILT_OBJS := \
	$(MEDIA_SERVER)/rtp-udp-transport.o \
	$(MEDIA_SERVER)/media/h265-file-reader.o \
	$(MEDIA_SERVER)/media/h265-file-source.o \
	$(MEDIA_SERVER)/media/h264-file-reader.o \
	$(MEDIA_SERVER)/media/h264-file-source.o \
	$(MEDIA_SERVER)/media/mp4-file-reader.o \
	$(MEDIA_SERVER)/media/mp4-file-source.o \
	$(MEDIA_SERVER)/media/ps-file-source.o \
	$(MEDIA_SERVER)/media/pcm-file-source.o \
	$(MEDIA_SERVER)/media/avpacket-queue.o

OBJS     := $(SRCS:.c=.o)
OBJS_CPP := $(SRCS_CPP:.cpp=.o)
DEPS     := $(SRCS:.c=.d)

# Include paths
INCS := \
	-I$(MW_INC) \
	-I$(ISP_INC) \
	-I$(KERNEL_INC) \
	-I$(MW_INC)/linux \
	-I$(MW_SAMPLE)/common \
	-I$(MEDIA_SERVER) \
	-I$(MEDIA_SERVER)/include \
	-I$(MEDIA_SERVER)/librtsp/source/server \
	-I$(MEDIA_SERVER)/libmov/include \
	-I$(MEDIA_SERVER)/libmov/test \
	-I$(MEDIA_SERVER)/sdk/include \
	-I$(MEDIA_SERVER)/sdk/libhttp/include \
	-I$(MEDIA_SERVER)/librtp/include \
	-I$(MEDIA_SERVER)/libmpeg/include \
	-I$(MEDIA_SERVER)/libflv/include \
	-I$(MEDIA_SERVER)/libmkv/include \
	-I$(MEDIA_SERVER)/media \
	-I$(MEDIA_SERVER)/avcodec/avcodec/include \
	-I$(MEDIA_SERVER)/avcodec/avbsf/include \
	-I$(MEDIA_SERVER)/libmov/include

CPPFLAGS += $(INCS) -D__ERROR__=00*10000000+__LINE__*1000
EXTRA_CFLAGS := $(INCS)

ifeq ($(DEBUG), 1)
CFLAGS += -g -O0
endif

# Library paths
PKG_CONFIG_PATH := $(MW_PATH)/pkgconfig
REQUIRES := cvi_common cvi_sample
MW_LIBS := $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) \
	pkg-config --libs --define-variable=mw_dir=$(MW_PATH) $(REQUIRES))

EXTRA_LDFLAGS := \
	$(MW_LIBS) \
	-L$(MEDIA_SERVER)/libs \
	-lrtsp -lhttp -lflv -lmov -lrtp -lmpeg -lsdk -lavcodec -lavbsf \
	-lcvi_audio -ltinyalsa \
	-latomic -lsys -lpthread -lm -lini

.PHONY: all clean

all: $(TARGET)

# Compile http-parser.c with NDEBUG (disables assert)
$(HTTP_PARSER_OBJ): $(HTTP_PARSER_SRC)
	@$(CC) $(CFLAGS) $(EXTRA_CFLAGS) -DNDEBUG -o $@ -c $<
	@echo [$(notdir $(CC))] $(notdir $@)

# rtsp-server.cpp needs include/ before the top-level media_server dir so that
# the full rtsp-server.h (with rtsp_handler_t) is found instead of the wrapper.
# Also compile with -DNDEBUG to disable asserts (e.g. in rtsp_onsetup).
RTSP_SERVER_INCS := \
	-I$(MEDIA_SERVER)/include \
	$(INCS)

$(MEDIA_SERVER)/rtsp-server.o: $(MEDIA_SERVER)/rtsp-server.cpp
	@$(CXX) $(CFLAGS) $(RTSP_SERVER_INCS) -D__ERROR__=00*10000000+__LINE__*1000 -DNDEBUG -std=gnu++11 -o $@ -c $<
	@echo [$(notdir $(CXX))] $(notdir $@)

$(SDIR)/%.o: $(SDIR)/%.c
	@$(CC) $(DEPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) -o $@ -c $<
	@echo [$(notdir $(CC))] $(notdir $@)

$(TARGET): $(OBJS) $(OBJS_CPP) $(HTTP_PARSER_OBJ) $(PREBUILT_OBJS)
	@$(CXX) -o $@ \
		-Wl,--start-group \
		$(OBJS) $(OBJS_CPP) $(HTTP_PARSER_OBJ) $(PREBUILT_OBJS) \
		-lsys $(MW_LIB)/libsys.a \
		-Wl,--end-group \
		$(ELFFLAGS) $(EXTRA_LDFLAGS)
	@echo -e "\033[34m[LINK]\033[0m[$(notdir $(CXX))] $(notdir $@)"

clean:
	@rm -f $(OBJS) $(OBJS_CPP) $(HTTP_PARSER_OBJ) $(DEPS) $(TARGET)

-include $(DEPS)
