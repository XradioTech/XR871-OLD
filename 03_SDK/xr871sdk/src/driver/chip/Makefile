# Rules for building library

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libchip.a

DIRS = $(shell find . -type d)
SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))
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