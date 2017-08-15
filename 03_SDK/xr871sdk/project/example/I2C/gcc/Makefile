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
ROOT_PATH := ../../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# project and objects
# ----------------------------------------------------------------------------
PROJECT := i2c

INCLUDE_PATHS += -I$(ROOT_PATH)/project/example/$(PROJECT)

DIRS := .. \
	$(ROOT_PATH)/project/common/startup/gcc \
	$(ROOT_PATH)/project/common/board \
	$(ROOT_PATH)/project/common/board/$(__PRJ_CONFIG_BOARD)

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# extra libs
# PRJ_EXTRA_LIBS :=

# ----------------------------------------------------------------------------
# override project variables
# ----------------------------------------------------------------------------
# linker script path/file
#   - relative to "./"
#   - define your own "LINKER_SCRIPT_PATH" and/or "LINKER_SCRIPT" to override
#     the default one
# LINKER_SCRIPT_PATH := .
# LINKER_SCRIPT :=

# image config path/file
#   - relative to "../image/xxxxx/", eg. "../image/xr871/"
#   - define your own "IMAGE_CFG_PATH" and/or "IMAGE_CFG" to override the
#     default one
# IMAGE_CFG_PATH := .
# IMAGE_CFG :=

# image name, default to xr_system
# IMAGE_NAME :=

# project make rules
include $(PRJ_MAKE_RULES)
