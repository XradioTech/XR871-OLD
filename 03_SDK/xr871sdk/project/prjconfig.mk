#
# project common config options
#

# ----------------------------------------------------------------------------
# config options
# ----------------------------------------------------------------------------
# support both sta and ap
__PRJ_CONFIG_WLAN_STA_AP ?= n

# support xplayer
__PRJ_CONFIG_XPLAYER ?= n

# enable XIP
__PRJ_CONFIG_XIP ?= n

# enable OTA
__PRJ_CONFIG_OTA ?= n

# enable image compress
__PRJ_CONFIG_IMG_COMPRESS ?= n

# ----------------------------------------------------------------------------
# config symbols
# ----------------------------------------------------------------------------
PRJ_CONFIG_SYMBOLS :=

ifeq ($(__PRJ_CONFIG_WLAN_STA_AP), y)
  PRJ_CONFIG_SYMBOLS += -D__PRJ_CONFIG_WLAN_STA_AP
endif

ifeq ($(__PRJ_CONFIG_XPLAYER), y)
  PRJ_CONFIG_SYMBOLS += -D__PRJ_CONFIG_XPLAYER
endif

ifeq ($(__PRJ_CONFIG_XIP), y)
  PRJ_CONFIG_SYMBOLS += -D__PRJ_CONFIG_XIP
endif

ifeq ($(__PRJ_CONFIG_OTA), y)
  PRJ_CONFIG_SYMBOLS += -D__PRJ_CONFIG_OTA
endif

ifeq ($(__PRJ_CONFIG_IMG_COMPRESS), y)
  PRJ_CONFIG_SYMBOLS += -D__PRJ_CONFIG_IMG_COMPRESS
endif