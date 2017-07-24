# Rules for building application

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
PROJECT = audio_player

__CONFIG_XPLAYER = 1
__CONFIG_OTA_UPDATE = 0
__CONFIG_XIP_ENABLE = 1
__CONFIG_SOUND_CARD = 1

COMMON_DIR = $(ROOT_PATH)/project/common
DIRS = .. $(COMMON_DIR)/startup/gcc $(COMMON_DIR)/board \
	$(COMMON_DIR)/ctrl_msg $(COMMON_DIR)/cmd $(COMMON_DIR)/ctrl_img \
	$(COMMON_DIR)/sysinfo

ifeq ($(__CONFIG_ARCH_DUAL_CORE), 1)
DIRS += $(COMMON_DIR)/net_ctrl $(COMMON_DIR)/iperf
endif

SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))
OBJS = $(addsuffix .o,$(SRCS))

INCLUDE_PATHS += -I$(ROOT_PATH)/project

ifeq ($(__CONFIG_XPLAYER), 1)
  CC_SYMBOLS += -D__CONFIG_XPLAYER
  INC_DIRS = $(shell find $(ROOT_PATH)/src/cedarx -maxdepth 4 -type d)
  INCLUDE_PATHS += $(foreach n, $(INC_DIRS), -I$(n))
  LIBS_XPLAYER = -lcedarx -lmp3 -lamr -lamren -lcedarx
endif

ifeq ($(__CONFIG_OTA_UPDATE), 1)
CC_SYMBOLS += -D__CONFIG_OTA_UPDATE
endif

ifeq ($(__CONFIG_XIP_ENABLE), 1)
  CC_SYMBOLS += -D__CONFIG_XIP_ENABLE
endif

LIBRARY_PATHS = -L$(ROOT_PATH)/lib

LIBS_DUAL_CORE =

ifeq ($(__CONFIG_ARCH_DUAL_CORE), 1)
LIBS_DUAL_CORE += -lxrsys -lwlan -lairkiss_aes -lping -lmqtt -lhttpcli \
	-lmbedtls -lhttpd -lsntp -ludhcpd -llwip -lxrsys
endif

# There are strong and weak symbols in "lchip", it may link to the weak symbol
# as they are statc libraries, so use --whole-archive to solve this problem.
LIBRARIES = -Wl,--whole-archive -lchip -Wl,--no-whole-archive \
	$(LIBS_XPLAYER) $(LIBS_DUAL_CORE) -lfs -lota -limage -lpm -lefpg -lconsole -lcomponent -los


ifeq ($(__CONFIG_SOUND_CARD), 1)
  CC_SYMBOLS += -D__CONFIG_SOUNDCARD0_ENABLE
  LIBRARIES += -lpcm -laudmgr
endif

ifeq ($(__CONFIG_OS_USE_FREERTOS), 1)
LIBRARIES += -lfreertos
endif

LIBRARIES += $(LD_SYS_LIBS) -lxrc

CC_SYMBOLS += -D__CONFIG_BOARD_XR871_EVB_AUDIO
CC_FLAGS += -Werror

################################################################################
LINKER_SCRIPT_PATH = $(ROOT_PATH)/project/linker_script/gcc/$(CONFIG_CHIP_NAME)
BIN_PATH = $(ROOT_PATH)/bin/$(CONFIG_CHIP_NAME)
IMAGE_PATH = ../image/$(CONFIG_CHIP_NAME)

# $(IMAGE_CFG_PATH) is relative to $(IMAGE_PATH)
IMAGE_CFG_PATH = ../$(ROOT_PATH)/project/image_cfg/$(CONFIG_CHIP_NAME)

# $(IMAGE_TOOL) is relative to $(IMAGE_PATH)
IMAGE_TOOL = ../$(ROOT_PATH)/tools/$(MKIMAGE)

ifeq ($(__CONFIG_XIP_ENABLE), 1)
  ifeq ($(__CONFIG_OTA_UPDATE), 1)
    IMAGE_CFG = ./image_xip_ota.cfg
  else
    IMAGE_CFG = ./image_xip.cfg
  endif
  LINKER_SCRIPT = ./xr871_appos_xip.ld
else
  ifeq ($(__CONFIG_OTA_UPDATE), 1)
    IMAGE_CFG = $(IMAGE_CFG_PATH)/image_ota.cfg
  else
    IMAGE_CFG = $(IMAGE_CFG_PATH)/image.cfg
  endif
  LINKER_SCRIPT = $(LINKER_SCRIPT_PATH)/appos.ld
endif

ifeq ($(MDK_DEBUG_ENABLE),1)
	ELF_EXT = axf
else
	ELF_EXT = elf
endif

.PHONY: all $(PROJECT).$(ELF_EXT) objdump size clean lib lib_clean install image image_clean

all: $(PROJECT).bin size

$(PROJECT).$(ELF_EXT): $(OBJS)
	$(CC) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $(OBJS) $(LIBRARIES)

%.bin: %.$(ELF_EXT)
ifeq ($(__CONFIG_XIP_ENABLE), 1)
	$(OBJCOPY) -O binary -R .xip $< $@
	$(OBJCOPY) -O binary -j .xip $< $(basename $@)_xip.bin
else
	$(OBJCOPY) -O binary $< $@
endif

%.objdump: %.$(ELF_EXT)
	$(OBJDUMP) -Sdh $< > $@

objdump: $(PROJECT).objdump

size:
	$(SIZE) $(PROJECT).$(ELF_EXT)

clean:
	-rm -f $(PROJECT).* *.bin $(OBJS) $(DEPS)

lib:
	$(MAKE) -C $(ROOT_PATH)/src install

lib_clean:
	$(MAKE) -C $(ROOT_PATH)/src clean

install:
	@mkdir -p $(IMAGE_PATH)
ifeq ($(__CONFIG_OTA_UPDATE), 1)
	$(CP) $(PROJECT).bin $(IMAGE_PATH)/app_ota.bin
else
	$(CP) $(PROJECT).bin $(IMAGE_PATH)/app.bin
endif
ifeq ($(__CONFIG_XIP_ENABLE), 1)
	$(CP) $(PROJECT)_xip.bin $(IMAGE_PATH)/app_xip.bin
endif

image: install
	$(CP) -t $(IMAGE_PATH) $(BIN_PATH)/*.bin && \
	cd $(IMAGE_PATH) && \
	chmod a+rw *.bin && \
	$(IMAGE_TOOL) -c $(IMAGE_CFG) -o xr_system.img

image_clean:
	-rm -f $(IMAGE_PATH)/*.bin $(IMAGE_PATH)/*.img

build: lib all install image

################################################################################
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
