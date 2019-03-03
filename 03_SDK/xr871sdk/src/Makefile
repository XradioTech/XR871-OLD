#
# Rules for building sub-modules
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# sub-modules
# ----------------------------------------------------------------------------
SUBDIRS := driver/chip libc image

ifeq ($(__CONFIG_OS_FREERTOS), y)
SUBDIRS += kernel/FreeRTOS kernel/os/FreeRTOS
endif

ifeq ($(__CONFIG_BIN_COMPRESS), y)
SUBDIRS += xz
endif

ifneq ($(__CONFIG_CHIP_SERIES_XR32), y)
NET_SUBDIRS := 	net/wlan \
	net/$(LWIP_DIR) \
	net/ping \
	net/HTTPClient \
	net/mbedtls-2.2.0 \
	net/nopoll \
	net/libwebsockets/lib \
	net/mqtt \
	net/shttpd-1.42 \
	net/sntp \
	net/udhcp-0.9.8 \
	net/cloud/aliyun \
	smartlink \
	sys

AT_SUBDIRS := atcmd
endif

ifneq ($(__CONFIG_BOOTLOADER), y)
SUBDIRS += driver/component \
	ota \
	pm \
	console \
	efpg \
	fs/fatfs \
	audio/pcm \
	audio/manager \
	$(NET_SUBDIRS) \
	$(AT_SUBDIRS) \
	cjson \
	util
endif

# ----------------------------------------------------------------------------
# building rules
# ----------------------------------------------------------------------------
.PHONY: all install clean install_clean $(SUBDIRS) _all _install _clean _install_clean

all:
	$(Q)$(MAKE) _all TARGET=all

install:
	$(Q)$(MAKE) _install TARGET=install

clean:
	$(Q)$(MAKE) _clean TARGET=clean

install_clean:
	$(Q)$(MAKE) _install TARGET=install_clean

$(SUBDIRS):
	$(Q)$(MAKE) $(S) -C $@ $(TARGET)

_all: $(SUBDIRS)

_install: $(SUBDIRS)

_clean: $(SUBDIRS)

_install_clean: $(SUBDIRS)
