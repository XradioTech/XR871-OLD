# Rules for building library

################################################################################
VER := 0.9.8
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libudhcpd.a

DIRS = .
#SRCS := xradio_dhcpd.c, arpping.c, files.c, leases.c, serverpacket.c, options.c, socket.c, packet.c
#SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.c)))
#SRCS := dhcpd.c
#OBJS := $(addsuffix .o,$(SRCS))
OBJS = usr_dhcpd.o  arpping.o  files.o  leases.o  serverpacket.o  options.o  socket.o  packet.o

CC_FLAGS += -Werror -DVERSION='"$(VER)"' -DXR_DHCPD
CC_FLAGS += -DDHCPD_TIMEALT -DDHCPD_LWIP -DDHCPD_LOWMEM -DDHCPD_USRCFG -DDHCPD_FREERTOS

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
