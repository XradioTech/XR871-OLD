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
LIBS := liblwip.a

DIRS_IGNORE := src/core/ipv6
DIRS_ALL := $(shell find src -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# extra flags: suppress memory address compare warnnings
CC_FLAGS += -Wno-address

# library make rules
include $(LIB_MAKE_RULES)
