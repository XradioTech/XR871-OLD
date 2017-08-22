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
LIBS := libnopoll.a

DIRS := ./src

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# extra include path
INCLUDE_PATHS += -I$(ROOT_PATH)/include/net/nopoll \
	 -I./src

# library make rules
include $(LIB_MAKE_RULES)
