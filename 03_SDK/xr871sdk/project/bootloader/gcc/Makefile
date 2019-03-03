#
# Rules for building application
#

# ----------------------------------------------------------------------------
# project local config
# ----------------------------------------------------------------------------
include localconfig.mk

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# project and objects
# ----------------------------------------------------------------------------
PROJECT := bootloader

INCLUDE_PATHS += -I$(ROOT_PATH)/project/$(PROJECT)

DIRS := .. \
	$(ROOT_PATH)/project/common/startup/gcc \
	$(ROOT_PATH)/project/common/board \
	$(ROOT_PATH)/project/common/board/$(__PRJ_CONFIG_BOARD)

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# extra libs
# PRJ_EXTRA_LIBS :=

# extra symbols (macros)
# PRJ_EXTRA_CONFIG_SYMBOLS :=

# ----------------------------------------------------------------------------
# override project variables
# ----------------------------------------------------------------------------
# linker script path/file
#   - relative to "./"
#   - define your own "LINKER_SCRIPT_PATH" and/or "LINKER_SCRIPT" to override
#     the default one
LINKER_SCRIPT_PATH := .
LINKER_SCRIPT := ./$(__CONFIG_CHIP_TYPE)_bootloader.ld

# project make rules
include $(PRJ_MAKE_RULES)
