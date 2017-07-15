# Common rules for GCC Makefile

################################################################################
CC_DIR = ~/tools/gcc-arm-none-eabi-4_9-2015q2/bin
CC_PREFIX = $(CC_DIR)/arm-none-eabi-

DEBUG = 0
HARDFP = 0
MDK_DEBUG_ENABLE = 1

################################################################################
CP = cp -u
ifeq ($(shell uname -o), Cygwin)
MKIMAGE = mkimage.exe
else
MKIMAGE = mkimage
endif

################################################################################
AS = $(CC_PREFIX)as
CC = $(CC_PREFIX)gcc
CPP = $(CC_PREFIX)g++
LD = $(CC_PREFIX)ld
AR = $(CC_PREFIX)ar
OBJCOPY = $(CC_PREFIX)objcopy
OBJDUMP = $(CC_PREFIX)objdump
SIZE = $(CC_PREFIX)size

ifeq ($(HARDFP),1)
	FLOAT_ABI = hard
else
	FLOAT_ABI = softfp
endif

ifeq ($(MDK_DEBUG_ENABLE),1)
	DEBUG_OPT = -gdwarf-2
else
	DEBUG_OPT = -g
endif

# Set the compiler CPU/FPU options.
CPU = -mcpu=cortex-m3 -mthumb
#-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=$(FLOAT_ABI)

CC_FLAGS = $(CPU) -c $(DEBUG_OPT) -fno-common -fmessage-length=0 -Wall \
	-fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer \
	-MMD -MP

CC_SYMBOLS = -DTARGET_M3 -DTARGET_CORTEX_M \
	-DTOOLCHAIN_GCC_ARM -DTOOLCHAIN_GCC \
	-D__CORTEX_M3 -DARM_MATH_CM3 -D__FPU_PRESENT=0

# add config symbols
include $(ROOT_PATH)/config.mk
CC_SYMBOLS += $(CONFIG_SYMBOLS)
AS_SYMBOLS = $(CONFIG_SYMBOLS)

LD_FLAGS = $(CPU) -Wl,--gc-sections --specs=nano.specs \
	-u _printf_float -u _scanf_float \
	-Wl,-Map=$(basename $@).map,--cref

LD_FLAGS += -Wl,--wrap,main
LD_FLAGS += -Wl,--wrap,malloc
LD_FLAGS += -Wl,--wrap,realloc
LD_FLAGS += -Wl,--wrap,free
LD_FLAGS += -Wl,--wrap,_malloc_r
LD_FLAGS += -Wl,--wrap,_realloc_r
LD_FLAGS += -Wl,--wrap,_free_r
ifeq ($(__CONFIG_MALLOC_TRACE), 1)
LD_FLAGS += -Wl,--wrap,calloc
LD_FLAGS += -Wl,--wrap,strdup
endif

LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys

ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -O0
else
  CC_FLAGS += -DNDEBUG -Os
endif

INCLUDE_ROOT_PATH = $(ROOT_PATH)/include
INCLUDE_PATHS = -I$(INCLUDE_ROOT_PATH) \
	-I$(INCLUDE_ROOT_PATH)/libc \
	-I$(INCLUDE_ROOT_PATH)/driver/cmsis

ifeq ($(__CONFIG_OS_USE_FREERTOS), 1)
  INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS \
	-I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/portable
endif

ifeq ($(__CONFIG_ARCH_DUAL_CORE), 1)
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/lwip \
	-I$(INCLUDE_ROOT_PATH)/net/lwip/ipv4
endif

################################################################################
%.o: %.asm
	$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.s
	$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.S
	$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.c
	$(CC) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99 $(INCLUDE_PATHS) -o $@ $<

%.o: %.cpp
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 -fno-rtti $(INCLUDE_PATHS) -o $@ $<

