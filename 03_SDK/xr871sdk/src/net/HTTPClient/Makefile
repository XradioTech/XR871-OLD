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
LIBS := libhttpcli.a

DIRS_IGNORE := ./Sample
DIRS_ALL := $(shell find ./ -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# extra include path
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/HTTPClient \
	-I$(INCLUDE_ROOT_PATH)/net/HTTPClient/API

# library make rules
include $(LIB_MAKE_RULES)
