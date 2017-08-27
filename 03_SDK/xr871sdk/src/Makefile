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
SUBDIRS := driver/chip libc ota image

ifeq ($(__CONFIG_OS_FREERTOS), y)
SUBDIRS += kernel/FreeRTOS kernel/os/FreeRTOS
endif

ifneq ($(__CONFIG_BOOTLOADER), y)

SUBDIRS += driver/component \
	pm \
	console \
	efpg \
	fs/fatfs \
	audio/pcm \
	audio/manager \
	cedarx \
	sys \
	net/wlan \
	net/lwip-1.4.1 \
	net/ping \
	net/HTTPClient \
	net/mbedtls-2.2.0 \
	net/nopoll \
	net/mqtt \
	net/shttpd-1.42 \
	net/sntp \
	net/udhcp-0.9.8 \
	cjson \
	atcmd \
	cloud/bbc \
	cloud/alink

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
