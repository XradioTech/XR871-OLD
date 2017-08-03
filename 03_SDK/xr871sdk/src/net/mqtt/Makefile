#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# library and objects
# ----------------------------------------------------------------------------
LIBS := libmqtt.a

DIRS := ./MQTTPacket \
	./MQTTClient-C \
	./MQTTClient-C/Xr_RTOS \
	./MQTTClient-C/Xr_RTOS/utest

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# extra include path
INCLUDE_PATHS += -I$(ROOT_PATH)/include/net/mqtt/MQTTPacket \
	-I$(ROOT_PATH)/include/net/mqtt/MQTTClient-C \
	-I./MQTTPacket \
	-I./MQTTClient-C \
	-I./MQTTClient-C/Xr_RTOS \
	-I./MQTTClient-C/Xr_RTOS/utest

# library make rules
include $(LIB_MAKE_RULES)
