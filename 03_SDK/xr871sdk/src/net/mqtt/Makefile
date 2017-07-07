# Rules for building library

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libmqtt.a

DIRS = ./MQTTPacket ./MQTTClient-C/Xr_RTOS/utest ./MQTTClient-C/Xr_RTOS ./MQTTClient-C
#DIRS_IGNORE := ./MQTTPacket/samples ./MQTTPacket/test
#DIRS_ALL := $(shell find ./src -type d)
#DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))
SRCS = $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.c)))
OBJS = $(addsuffix .o,$(SRCS))

INCLUDE_PATHS += -I$(ROOT_PATH)/include \
	-I$(ROOT_PATH)/include/rtos \
	-I$(ROOT_PATH)/include/net \
	-I$(ROOT_PATH)/include/net/lwip \
	-I$(ROOT_PATH)/include/net/lwip/ipv4 \
	-I$(ROOT_PATH)/include/net/mqtt/MQTTPacket \
	-I$(ROOT_PATH)/include/net/mqtt/MQTTClient-C \
	-I$(ROOT_PATH)/src/net/mqtt/MQTTPacket \
	-I$(ROOT_PATH)/src/net/mqtt/MQTTClient-C/Xr_RTOS/utest \
	-I$(ROOT_PATH)/src/net/mqtt/MQTTClient-C/Xr_RTOS \
	-I$(ROOT_PATH)/src/net/mqtt/MQTTClient-C \
	-I..

DIRS_STM32F411RE = $(shell find $(ROOT_PATH)/src/mbed/targets -type d)
INCLUDE_PATHS += $(foreach dir,$(DIRS_STM32F411RE),-I$(dir))

CC_FLAGS += -Werror
CC_FLAGS += -Wno-address	# suppress memory address compare warnnings
# CC_FLAGS += -Wno-format	# suppress printf/scanf format warnnings
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
	-rm -f $(LIBS) $(OBJS) $(DEPS)


################################################################################
DEPS = $(OBJS:.o=.d)
-include $(DEPS)