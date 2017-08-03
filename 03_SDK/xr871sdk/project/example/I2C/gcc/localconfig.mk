#
# project local config options, override the common config options
#

# ----------------------------------------------------------------------------
# board definition
# ----------------------------------------------------------------------------
__PRJ_CONFIG_BOARD := xr871_evb_main

# ----------------------------------------------------------------------------
# override global config options
# ----------------------------------------------------------------------------
# set y to enable bootloader and disable some features, for bootloader only
# export __CONFIG_BOOTLOADER := y

# set n to disable dual core features, for bootloader only
# export __CONFIG_ARCH_DUAL_CORE := n

# ----------------------------------------------------------------------------
# override project common config options
# ----------------------------------------------------------------------------
# support both sta and ap, default to n
# __PRJ_CONFIG_WLAN_STA_AP := y

# support xplayer, default to n
# __PRJ_CONFIG_XPLAYER := y

# enable XIP, default to n
# __PRJ_CONFIG_XIP := y

# enable OTA, default to n
# __PRJ_CONFIG_OTA := y
