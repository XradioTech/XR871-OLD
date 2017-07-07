# Rules for building library

################################################################################
ROOT_PATH = ../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libcedarx.a

#DIRS = $(shell find . -type d)
DIRS = $(shell find . -maxdepth 4 -type d)
DIRS += .
SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))
OBJS = $(addsuffix .o,$(SRCS))

INCLUDE_PATHS += $(foreach n, $(DIRS), -I$(n))

CC_FLAGS += -Werror

################################################################################
.PHONY: all install size clean

all: $(LIBS)

$(LIBS): $(OBJS)
	#$(AR) -crs $@ $^

install: $(LIBS)
	#$(CP) -t $(ROOT_PATH)/lib $^

size:
	$(SIZE) -t $(LIBS)

clean:
	-rm -f $(LIBS) $(OBJS) $(DEPS)


################################################################################
DEPS = $(OBJS:.o=.d)
-include $(DEPS)