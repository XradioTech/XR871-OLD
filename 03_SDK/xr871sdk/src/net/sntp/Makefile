# Rules for building library

################################################################################

ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libsntp.a

DIRS = .
OBJS = sntp.o

INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net \
	-I$(INCLUDE_ROOT_PATH)/net/lwip \
	-I$(INCLUDE_ROOT_PATH)/net/lwip/ipv4 

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
