#
# project local config options, override the common config options
#

# ----------------------------------------------------------------------------
# board definition
# ----------------------------------------------------------------------------
__PRJ_CONFIG_BOARD := xr871_bl_base
#__PRJ_CONFIG_BOARD := xr32_bl_base

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

# set y to support bin compression
export __CONFIG_BIN_COMPRESS := y

# ----------------------------------------------------------------------------
# override project common config options
# ----------------------------------------------------------------------------
# set chip type: xr871 or xr32
ifeq ($(__PRJ_CONFIG_BOARD), xr32_bl_base)
export __CONFIG_CHIP_TYPE := xr32
endif
