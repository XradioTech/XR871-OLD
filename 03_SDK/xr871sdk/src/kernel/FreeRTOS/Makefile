# Rules for building library

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libfreertos.a

DIRS = ./Source ./Source/portable/GCC/ARM_CM3
SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

ifeq ($(__CONFIG_ARCH_MEM_PATCH), 1)
  SRCS += ./Source/portable/MemMang/heap_4
else # __CONFIG_ARCH_MEM_PATCH
  ifeq ($(__CONFIG_MALLOC_USE_STDLIB), 1)
    SRCS += ./Source/portable/MemMang/heap_stdlib
  else
    SRCS += ./Source/portable/MemMang/heap_4
  endif
endif # __CONFIG_ARCH_MEM_PATCH

OBJS = $(addsuffix .o,$(SRCS))

CC_FLAGS += -Werror

################################################################################
.PHONY: all install size clean

all: $(LIBS)

$(LIBS): $(OBJS)
	$(AR) -crs $@ $^

install: $(LIBS)
	$(CP) -t $(ROOT_PATH)/lib $^

size:
	$(SIZE) -t $(LIBS)

clean:
	-rm -f $(LIBS) $(OBJS) $(DEPS)


################################################################################
DEPS = $(OBJS:.o=.d)
-include $(DEPS)