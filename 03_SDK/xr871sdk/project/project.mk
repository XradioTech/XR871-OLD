#
# Rules for building project
#

# ----------------------------------------------------------------------------
# project common config
# ----------------------------------------------------------------------------
include $(ROOT_PATH)/project/prjconfig.mk

# ----------------------------------------------------------------------------
# library
# ----------------------------------------------------------------------------
LIBRARY_PATHS := -L$(ROOT_PATH)/lib

# There are strong and weak symbols in "lchip", it may link to the weak symbol
# as they are statc libraries, so use --whole-archive to solve this problem.
LIBRARIES := -Wl,--whole-archive -lchip -Wl,--no-whole-archive

# add extra libs from specific project
LIBRARIES += $(PRJ_EXTRA_LIBS)

ifneq ($(__CONFIG_BOOTLOADER), y)

LIBRARIES += -lota

# audio player libs
ifeq ($(__PRJ_CONFIG_XPLAYER), y)
  LIBRARIES += \
            -lcedarx \
            -lmp3 \
            -lamr \
            -lamren \
            -lwav  \
            -laac \
            -lcedarx
endif

ifneq ($(__CONFIG_CHIP_SERIES_XR32), y)
# network libs
LIBRARIES += -lmqtt \
	-lnopoll \
	-llibwebsockets \
	-lhttpd \
	-lhttpcli \
	-lmbedtls \
	-lsntp \
	-lping \
	-ludhcpd \
	-lxrsys \
	-lwlan \
	-lsmartlink \
	-lairkiss_aes \
	-lsc_assistant \
	-llwip \
	-lxrsys
endif

LIBRARIES += -lcjson -lfs -lconsole -lcomponent -lefpg -lpm -laudmgr -lpcm \
	-ladt -lutil -lreverb

endif # __CONFIG_BOOTLOADER

ifeq ($(__CONFIG_BIN_COMPRESS), y)
LIBRARIES += -lxz
endif

LIBRARIES += -limage -los

ifeq ($(__CONFIG_OS_FREERTOS), y)
  LIBRARIES += -lfreertos
endif

LIBRARIES += $(LD_SYS_LIBS) -lxrc

# ----------------------------------------------------------------------------
# extra include path
# ----------------------------------------------------------------------------
ifeq ($(__PRJ_CONFIG_XPLAYER), y)
  CEDARX_INC_DIRS := $(shell find $(ROOT_PATH)/include/cedarx -type d)
  INCLUDE_PATHS += $(foreach dir, $(CEDARX_INC_DIRS), -I$(dir))
endif

INCLUDE_PATHS += -I$(ROOT_PATH)/project

ifneq ($(__PRJ_CONFIG_BOARD),)
  INCLUDE_PATHS += -I$(ROOT_PATH)/project/common/board/$(__PRJ_CONFIG_BOARD)
else
  $(error board is not defined!)
endif

# ----------------------------------------------------------------------------
# include config header for all project
# ----------------------------------------------------------------------------
CC_FLAGS += -include common/prj_conf_opt.h

# ----------------------------------------------------------------------------
# common suffix
# ----------------------------------------------------------------------------
ifeq ($(__PRJ_CONFIG_WLAN_STA_AP), y)
  SUFFIX_WLAN := _sta_ap
endif

ifeq ($(__PRJ_CONFIG_XIP), y)
  SUFFIX_XIP := _xip
endif

ifeq ($(__CONFIG_BIN_COMPRESS), y)
  SUFFIX_XZ := _xz
endif

ifeq ($(__PRJ_CONFIG_RAM_EXT), y)
  SUFFIX_RAME := _ext
endif

ifeq ($(__CONFIG_XIP_SECTION_FUNC_LEVEL), y)
  LINKER_SCRIPT_SUFFIX := _max
endif

# ----------------------------------------------------------------------------
# linker script
# ----------------------------------------------------------------------------
# linker script, maybe override by the specific project
LINKER_SCRIPT_PATH ?= $(ROOT_PATH)/project/linker_script/gcc/$(CONFIG_CHIP_NAME)
LINKER_SCRIPT ?= $(LINKER_SCRIPT_PATH)/appos$(SUFFIX_XIP)$(LINKER_SCRIPT_SUFFIX).ld

# ----------------------------------------------------------------------------
# image
# ----------------------------------------------------------------------------
# original bin path, files and names
BIN_PATH := $(ROOT_PATH)/bin/$(CONFIG_CHIP_NAME)
BIN_FILES := $(wildcard $(BIN_PATH)/*.bin)
BIN_NAMES := $(notdir $(BIN_FILES))

ifeq ($(__CONFIG_BIN_COMPRESS), y)

# xz is a tool used to compress bins
XZ_CHECK ?= none
XZ_LZMA2_DICT_SIZE ?= 8KiB
XZ := xz -f -k --no-sparse --armthumb --check=$(XZ_CHECK) \
         --lzma2=preset=6,dict=$(XZ_LZMA2_DICT_SIZE),lc=3,lp=1,pb=1

XZ_DEFAULT_BINS := app.bin
ifeq ($(__CONFIG_CHIP_SERIES_XR32), y)
  ifeq ($(__PRJ_CONFIG_RAM_EXT), y)
    XZ_DEFAULT_BINS += app_ext.bin
  endif
else
  ifeq ($(__CONFIG_CHIP_XR871), y)
    XZ_DEFAULT_BINS += net.bin net_ap.bin net_wps.bin
  endif
endif
XZ_BINS ?= $(XZ_DEFAULT_BINS)

endif # __CONFIG_BIN_COMPRESS

# output image path
IMAGE_PATH := ../image/$(CONFIG_CHIP_NAME)

# $(IMAGE_TOOL) is relative to $(IMAGE_PATH)
IMAGE_TOOL := ../$(ROOT_PATH)/tools/$(MKIMAGE)

# image config file, maybe override by the specific project
# $(IMAGE_CFG_PATH) is relative to $(IMAGE_PATH)
IMAGE_CFG_PATH ?= ../$(ROOT_PATH)/project/image_cfg/$(CONFIG_CHIP_NAME)
IMAGE_CFG ?= $(IMAGE_CFG_PATH)/image$(SUFFIX_WLAN)$(SUFFIX_XIP)$(SUFFIX_XZ).cfg

# image tool's options to enable/disable OTA
ifeq ($(__PRJ_CONFIG_OTA), y)
  IMAGE_TOOL_OPT := -O
else
  IMAGE_TOOL_OPT :=
endif

# image name, maybe override by the specific project
IMAGE_NAME ?= xr_system

# ----------------------------------------------------------------------------
# common targets and building rules
# ----------------------------------------------------------------------------
CC_SYMBOLS += $(PRJ_CONFIG_SYMBOLS) $(PRJ_EXTRA_CONFIG_SYMBOLS)

ifeq ($(MDK_DBG_EN), y)
  ELF_EXT = axf
else
  ELF_EXT = elf
endif

ifeq ($(__PRJ_CONFIG_XIP), y)
  OBJCOPY_R_XIP := -R .xip
  OBJCOPY_J_XIP := -j .xip
endif

ifeq ($(__PRJ_CONFIG_RAM_EXT), y)
  OBJCOPY_R_EXT := -R .text_ext -R .data_ext
  OBJCOPY_J_EXT := -j .text_ext -j .data_ext
endif

.PHONY: all $(PROJECT).$(ELF_EXT) objdump size clean lib lib_clean \
	lib_install_clean install image image_clean build build_clean

all: $(PROJECT).bin size

$(PROJECT).$(ELF_EXT): $(OBJS)
	$(Q)$(CC) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $(OBJS) $(LIBRARIES)

%.bin: %.$(ELF_EXT)
	$(Q)$(OBJCOPY) -O binary $(OBJCOPY_R_XIP) $(OBJCOPY_R_EXT) $< $@
ifeq ($(__PRJ_CONFIG_XIP), y)
	$(Q)$(OBJCOPY) -O binary $(OBJCOPY_J_XIP) $< $(basename $@)$(SUFFIX_XIP).bin
endif
ifeq ($(__PRJ_CONFIG_RAM_EXT), y)
	$(Q)$(OBJCOPY) -O binary $(OBJCOPY_J_EXT) $< $(basename $@)$(SUFFIX_RAME).bin
endif

%.objdump: %.$(ELF_EXT)
	$(Q)$(OBJDUMP) -Sdh $< > $@

objdump: $(PROJECT).objdump

size:
	$(Q)$(SIZE) $(PROJECT).$(ELF_EXT)

clean:
	$(Q)-rm -f $(PROJECT).* *.bin $(OBJS) $(DEPS)

lib:
	$(Q)$(MAKE) $(S) -C $(ROOT_PATH)/src install

lib_clean:
	$(Q)$(MAKE) $(S) -C $(ROOT_PATH)/src clean

lib_install_clean:
	$(Q)$(MAKE) $(S) -C $(ROOT_PATH)/src install_clean

ifeq ($(__CONFIG_BOOTLOADER), y)

install:
	$(Q)$(CP) $(PROJECT).bin $(ROOT_PATH)/bin/$(CONFIG_CHIP_NAME)/boot.bin

build: lib all install

build_clean: clean lib_clean lib_install_clean

else # __CONFIG_BOOTLOADER

ifneq ($(__PRJ_CONFIG_ETF), y)

install:
	@mkdir -p $(IMAGE_PATH)
	$(Q)$(CP) $(PROJECT).bin $(IMAGE_PATH)/app.bin
ifeq ($(__PRJ_CONFIG_XIP), y)
	$(Q)$(CP) $(PROJECT)$(SUFFIX_XIP).bin $(IMAGE_PATH)/app$(SUFFIX_XIP).bin
endif
ifeq ($(__PRJ_CONFIG_RAM_EXT), y)
	$(Q)$(CP) $(PROJECT)$(SUFFIX_RAME).bin $(IMAGE_PATH)/app$(SUFFIX_RAME).bin
endif

image: install
	$(Q)$(CP) -t $(IMAGE_PATH) $(BIN_FILES)
ifeq ($(__CONFIG_BIN_COMPRESS), y)
	cd $(IMAGE_PATH) && \
	$(Q)$(XZ) $(XZ_BINS)
endif
	cd $(IMAGE_PATH) && \
	chmod a+r *.bin && \
	$(IMAGE_TOOL) $(IMAGE_TOOL_OPT) -c $(IMAGE_CFG) -o $(IMAGE_NAME).img

image_clean:
	cd $(IMAGE_PATH) && \
	rm -f $(BIN_NAMES) app*.bin *.xz *.img

endif # __PRJ_CONFIG_ETF

build: lib all image

build_clean: image_clean clean lib_clean lib_install_clean

endif # __CONFIG_BOOTLOADER

# ----------------------------------------------------------------------------
# dependent rules
# ----------------------------------------------------------------------------
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
