#
# project local config options, override the common config options
#

# ----------------------------------------------------------------------------
# board definition
# ----------------------------------------------------------------------------
__PRJ_CONFIG_BOARD := xr871_bl_base

# ----------------------------------------------------------------------------
# override global config options
# ----------------------------------------------------------------------------
# set y to enable bootloader and disable some features, for bootloader only
export __CONFIG_BOOTLOADER := y

# set n to disable dual core features, for bootloader only
export __CONFIG_ARCH_DUAL_CORE := n

# set n to disable printf float variables
export __CONFIG_LIBC_PRINTF_FLOAT := n

# set n to disable scanf float variables
export __CONFIG_LIBC_SCANF_FLOAT := n

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
