# Rules for building library

################################################################################

ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libping.a

DIRS = .
OBJS = ping.o

INCLUDE_PATHS += -I$(ROOT_PATH)/inc \
	-I$(ROOT_PATH)/include/net/ \
	-I$(ROOT_PATH)/include/net/lwip/ \
	-I$(ROOT_PATH)/include/net/lwip/ipv4
# -DCONFIG_PING
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
