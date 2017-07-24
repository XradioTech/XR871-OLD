# Rules for building library

################################################################################
ROOT_PATH = ..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
ifeq ($(__CONFIG_BOOTLOADER), 1)
SUBDIRS = driver/chip driver/component libc image console ota
else
SUBDIRS = driver/chip driver/component libc image pm console fs/fatfs audio/pcm audio/manager ota cedarx efpg atcmd
endif

ifeq ($(__CONFIG_OS_USE_FREERTOS), 1)
SUBDIRS += kernel/FreeRTOS kernel/os/FreeRTOS
endif

ifeq ($(__CONFIG_ARCH_DUAL_CORE), 1)
SUBDIRS += ./sys ./net/wlan ./net/lwip-1.4.1 ./net/ping ./net/HTTPClient \
	./net/mbedtls-2.2.0 ./net/mqtt ./net/shttpd-1.42 ./net/sntp \
	./net/udhcp-0.9.8
endif

################################################################################
.PHONY: all install clean $(SUBDIRS) _all _install _clean

all:
	$(MAKE) _all TARGET=all

install:
	$(MAKE) _install TARGET=install

clean:
	$(MAKE) _clean TARGET=clean

$(SUBDIRS):
	$(MAKE) -C $@ $(TARGET)

_install: $(SUBDIRS)

_clean: $(SUBDIRS)

_all: $(SUBDIRS)
