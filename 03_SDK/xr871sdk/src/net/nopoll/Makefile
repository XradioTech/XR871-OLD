# Rules for building library

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libnopoll.a

DIRS = ./src
SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))
OBJS = $(addsuffix .o,$(SRCS))

INCLUDE_PATHS += -I./src \
	-I$(ROOT_PATH)/include/net/nopoll

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
