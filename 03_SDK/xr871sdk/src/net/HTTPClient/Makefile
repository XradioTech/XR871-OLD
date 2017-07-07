# Rules for building library

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libhttpcli.a

DIRS_IGNORE := ./Sample
DIRS_ALL := $(shell find ./ -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.c)))
OBJS := $(addsuffix .o,$(SRCS))

INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net \
	-I$(INCLUDE_ROOT_PATH)/net/lwip \
	-I$(INCLUDE_ROOT_PATH)/net/lwip/ipv4 \
	-I$(INCLUDE_ROOT_PATH)/net/HTTPClient \
	-I$(INCLUDE_ROOT_PATH)/net/HTTPClient/API

CC_FLAGS += -Werror
CC_FLAGS += -Wno-address	# suppress memory address compare warnnings
#CC_FLAGS += -Wno-format	# suppress printf/scanf format warnnings
CC_FLAGS += -DXRADIO_RTOS

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
	-rm -f $(LIBS) $(OBJS) $(DEPS) *.d


################################################################################
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
