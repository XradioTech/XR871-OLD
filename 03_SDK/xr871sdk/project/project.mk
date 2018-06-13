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
LIBRARIES := -Wl,--whole-archive -lchip -Wl,--no-whole-archive -lota -limage

# add extra libs from specific project
LIBRARIES += $(PRJ_EXTRA_LIBS)

ifneq ($(__CONFIG_BOOTLOADER), y)

# audio player libs
ifeq ($(__PRJ_CONFIG_XPLAYER), y)
  LIBRARIES += -lcedarx -lmp3 -lamr -lamren -lcedarx
endif

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

LIBRARIES += -lcjson -lfs -lconsole -lcomponent -lefpg -lpm -laudmgr -lpcm \
	-luncompress -ladt -lutil

endif # __CONFIG_BOOTLOADER

LIBRARIES += -los

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
# original path of bin files
BIN_PATH := $(ROOT_PATH)/bin/$(CONFIG_CHIP_NAME)

# output image path
IMAGE_PATH := ../image/$(CONFIG_CHIP_NAME)

# $(IMAGE_TOOL) is relative to $(IMAGE_PATH)
IMAGE_TOOL := ../$(ROOT_PATH)/tools/$(MKIMAGE)

# image config file, maybe override by the specific project
# $(IMAGE_CFG_PATH) is relative to $(IMAGE_PATH)
IMAGE_CFG_PATH ?= ../$(ROOT_PATH)/project/image_cfg/$(CONFIG_CHIP_NAME)
IMAGE_CFG ?= $(IMAGE_CFG_PATH)/image$(SUFFIX_WLAN)$(SUFFIX_XIP).cfg

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
CC_SYMBOLS += $(PRJ_CONFIG_SYMBOLS)

ifeq ($(MDK_DBG_EN), y)
  ELF_EXT = axf
else
  ELF_EXT = elf
endif

.PHONY: all $(PROJECT).$(ELF_EXT) objdump size clean lib lib_clean \
	lib_install_clean install image image_clean build build_clean

all: $(PROJECT).bin size

$(PROJECT).$(ELF_EXT): $(OBJS)
	$(Q)$(CC) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $(OBJS) $(LIBRARIES)

%.bin: %.$(ELF_EXT)
ifeq ($(__PRJ_CONFIG_XIP), y)
	$(Q)$(OBJCOPY) -O binary -R .xip $< $@
	$(Q)$(OBJCOPY) -O binary -j .xip $< $(basename $@)$(SUFFIX_XIP).bin
else
	$(Q)$(OBJCOPY) -O binary $< $@
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

install:
	@mkdir -p $(IMAGE_PATH)
	$(Q)$(CP) $(PROJECT).bin $(IMAGE_PATH)/app.bin
ifeq ($(__PRJ_CONFIG_IMG_COMPRESS), y)
	rm -f $(PROJECT).bin.xz
	xz -k --check=crc32 --lzma2=preset=6e,dict=32KiB $(PROJECT).bin
	$(Q)$(CP) $(PROJECT).bin.xz $(IMAGE_PATH)/app.bin.xz
	rm -f $(PROJECT).bin.xz
endif
ifeq ($(__PRJ_CONFIG_XIP), y)
	$(Q)$(CP) $(PROJECT)$(SUFFIX_XIP).bin $(IMAGE_PATH)/app$(SUFFIX_XIP).bin
endif

image: install
	$(Q)$(CP) -t $(IMAGE_PATH) $(BIN_PATH)/*.bin
ifeq ($(__PRJ_CONFIG_IMG_COMPRESS), y)
	rm -f $(BIN_PATH)/*.bin.xz
	xz -k --check=crc32 --lzma2=preset=6e,dict=32KiB $(BIN_PATH)/net.bin
	xz -k --check=crc32 --lzma2=preset=6e,dict=32KiB $(BIN_PATH)/net_ap.bin
	$(Q)$(CP) -t $(IMAGE_PATH) $(BIN_PATH)/*.bin.xz
	rm -f $(BIN_PATH)/*.bin.xz
endif
	cd $(IMAGE_PATH) && \
	chmod a+rw *.bin && \
	$(IMAGE_TOOL) $(IMAGE_TOOL_OPT) -c $(IMAGE_CFG) -o $(IMAGE_NAME).img

image_clean:
	-rm -f $(IMAGE_PATH)/*.bin $(IMAGE_PATH)/*.xz $(IMAGE_PATH)/*.img

build: lib all image

build_clean: image_clean clean lib_clean lib_install_clean

endif # __CONFIG_BOOTLOADER

# ----------------------------------------------------------------------------
# dependent rules
# ----------------------------------------------------------------------------
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
