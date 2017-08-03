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
LIBS := libfreertos.a

DIRS := ./Source ./Source/portable/GCC/ARM_CM3

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

ifeq ($(__CONFIG_MALLOC_USE_STDLIB), y)
  SRCS += ./Source/portable/MemMang/heap_stdlib
else
  SRCS += ./Source/portable/MemMang/heap_4
endif

OBJS := $(addsuffix .o,$(SRCS))

# library make rules
include $(LIB_MAKE_RULES)
