#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# library and objects
# ----------------------------------------------------------------------------
LIBS := libmbedtls.a

DIRS := $(shell find ./ -type d | grep -v programs)

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# extra include path
INCLUDE_PATHS += -I$(ROOT_PATH)/include/net/ \
	-I$(ROOT_PATH)/include/net/mbedtls/configs

# extra flags
CC_FLAGS += -DMBEDTLS_CONFIG_FILE='<config-xr-mini-cliserv.h>'

# library make rules
include $(LIB_MAKE_RULES)
