# Rules for building library

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = liblwip.a

DIRS_IGNORE := ./src/core/ipv6
DIRS_ALL := $(shell find ./src -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.c)))
OBJS := $(addsuffix .o,$(SRCS))

CC_FLAGS += -Werror
CC_FLAGS += -Wno-address	# suppress memory address compare warnnings
#CC_FLAGS += -Wno-format	# suppress printf/scanf format warnnings

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