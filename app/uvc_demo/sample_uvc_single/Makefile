SHELL = /bin/bash
ifeq ($(PARAM_FILE), )
	PARAM_FILE:=../../../middleware/v2/Makefile.param
	include $(PARAM_FILE)
endif

MW_SAMPLE := $(realpath $(CURDIR)/../../../middleware/v2/sample)
COMMON_DIR := $(MW_SAMPLE)/common
PANEL_INC := $(MW_PATH)/component/panel/$(shell echo $(CHIP_ARCH) | tr A-Z a-z)

include $(MW_SAMPLE)/sample.mk

SDIR = $(PWD)/src
LOCAL_SRCS = $(wildcard $(SDIR)/*.c) $(wildcard $(SDIR)/src/*.c)
COMMON_SRCS = $(COMMON_DIR)/sample_common_venc.c	\
		$(COMMON_DIR)/sample_common_vdec.c	\
		$(COMMON_DIR)/sample_common_vo.c	\
		$(COMMON_DIR)/md5sum.c	\
		$(COMMON_DIR)/sample_common_sys.c	\
		$(COMMON_DIR)/sample_common_bin.c	\
		$(COMMON_DIR)/sample_common_platform.c	\
		$(COMMON_DIR)/sample_common_sensor.c	\
		$(COMMON_DIR)/sample_common_vi.c	\
		$(COMMON_DIR)/sample_common_isp.c	\
		$(COMMON_DIR)/sample_common_vpss.c
SRCS = $(LOCAL_SRCS) $(COMMON_SRCS)
INCS = -I$(MW_INC) -I$(ISP_INC) -I$(COMMON_DIR) -I$(KERNEL_INC) -I./include
INCS += -I$(PANEL_INC)
INCS += $(if $(SENSOR_LIST_INC),-I$(SENSOR_LIST_INC))
# GC4653 sensor object must be compiled in for SAMPLE_COMM_SNS_GetSnsObj to work
CFLAGS += -DSENSOR_GCORE_GC4653
OBJS = $(SRCS:.c=.o)
DEPS = $(LOCAL_SRCS:.c=.d)

TARGET = sample_uvc_single

PKG_CONFIG_PATH = $(MW_PATH)/pkgconfig
REQUIRES = cvi_common cvi_sample

MW_LIBS = $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --libs --define-variable=mw_dir=$(MW_PATH) $(REQUIRES))

LIBS = $(MW_LIBS)

ifeq ($(MULTI_PROCESS_SUPPORT), 1)
DEFS += -DRPC_MULTI_PROCESS
LIBS += -lnanomsg
endif

EXTRA_CFLAGS = $(INCS) $(DEFS)
EXTRA_LDFLAGS = $(LIBS) -lpthread -lm -lini

.PHONY : clean all
all: $(TARGET)

$(COMMON_DIR)/%.o: $(COMMON_DIR)/%.c
	@$(CC) $(CFLAGS) $(EXTRA_CFLAGS) -o $@ -c $<
	@echo [$(notdir $(CC))] $(notdir $@)

$(SDIR)/%.o: $(SDIR)/%.c
	@$(CC) $(DEPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) -o $@ -c $<
	@echo [$(notdir $(CC))] $(notdir $@)

$(TARGET): $(OBJS)
	$(CXX) -o $@ -Wl,--start-group $(OBJS) -Wl,--end-group $(ELFFLAGS) $(EXTRA_LDFLAGS)
	@echo -e $(BLUE)[LINK]$(END)[$(notdir $(CXX))] $(notdir $@)

clean:
	@rm -f $(OBJS) $(DEPS) $(TARGET)

-include $(DEPS)