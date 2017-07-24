# Rules for building bootloader

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
PROJECT = boot

__CONFIG_OTA_UPDATE = 0

COMMON_DIR = $(ROOT_PATH)/project/common
DIRS = .. $(COMMON_DIR)/startup/gcc $(COMMON_DIR)/board $(COMMON_DIR)/ctrl_img

SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))
OBJS = $(addsuffix .o,$(SRCS))

INCLUDE_PATHS += -I$(ROOT_PATH)/project

LIBRARY_PATHS = -L$(ROOT_PATH)/lib

# There are strong and weak symbols in "lchip", it may link to the weak symbol
# as they are statc libraries, so use --whole-archive to solve this problem.
LIBRARIES = -Wl,--whole-archive -lchip -Wl,--no-whole-archive \
	-lota -limage -los

ifeq ($(__CONFIG_OS_USE_FREERTOS), 1)
LIBRARIES += -lfreertos
endif

LIBRARIES += $(LD_SYS_LIBS) -lxrc

ifeq ($(__CONFIG_OTA_UPDATE), 1)
CC_SYMBOLS += -D__CONFIG_OTA_UPDATE
endif

CC_SYMBOLS += -D__CONFIG_BOARD_XR871_EVB_MAIN

CC_FLAGS += -Werror

################################################################################
LINKER_SCRIPT = ./bootloader_$(CONFIG_CHIP_NAME).ld

ifeq ($(MDK_DEBUG_ENABLE),1)
	ELF_EXT = axf
else
	ELF_EXT = elf
endif

.PHONY: all $(PROJECT).$(ELF_EXT) objdump size clean lib lib_clean install

all: $(PROJECT).bin size

$(PROJECT).$(ELF_EXT): $(OBJS)
	$(CC) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $(OBJS) $(LIBRARIES)

%.bin: %.$(ELF_EXT)
	$(OBJCOPY) -O binary $< $@

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
ifeq ($(__CONFIG_OTA_UPDATE), 1)
	$(CP) $(PROJECT).bin $(ROOT_PATH)/bin/$(CONFIG_CHIP_NAME)/boot_ota.bin
else
	$(CP) $(PROJECT).bin $(ROOT_PATH)/bin/$(CONFIG_CHIP_NAME)/boot.bin
endif

build: lib all install

################################################################################
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
