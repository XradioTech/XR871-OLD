#
# global config options
#

# ----------------------------------------------------------------------------
# config options
# ----------------------------------------------------------------------------
# chip and cpu
__CONFIG_CHIP_XR871 ?= y
__CONFIG_CPU_CM4F ?= y

# arch and core
__CONFIG_ARCH_DUAL_CORE ?= y
__CONFIG_ARCH_APP_CORE ?= y
__CONFIG_ARCH_NET_CORE ?= n

# redefine int32_t to signed int, but not signed long
__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE ?= y

# support printf float variables
__CONFIG_LIBC_PRINTF_FLOAT ?= y

# support scanf float variables
__CONFIG_LIBC_SCANF_FLOAT ?= y

# heap managed by stdlib
__CONFIG_MALLOC_USE_STDLIB ?= y

# trace heap memory usage and error when using malloc, free, etc.
__CONFIG_MALLOC_TRACE ?= n

# os
__CONFIG_OS_FREERTOS ?= y

# enable/disable bootloader, y to enable bootloader and disable some features
__CONFIG_BOOTLOADER ?= n

# ----------------------------------------------------------------------------
# config symbols
# ----------------------------------------------------------------------------
CONFIG_SYMBOLS =

ifeq ($(__CONFIG_CHIP_XR871), y)
  CONFIG_SYMBOLS += -D__CONFIG_CHIP_XR871
endif

ifeq ($(__CONFIG_CPU_CM4F), y)
  CONFIG_SYMBOLS += -D__CONFIG_CPU_CM4F
endif

ifeq ($(__CONFIG_ARCH_DUAL_CORE), y)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_DUAL_CORE
endif

ifeq ($(__CONFIG_ARCH_APP_CORE), y)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_APP_CORE
endif

ifeq ($(__CONFIG_ARCH_NET_CORE), y)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_NET_CORE
endif

ifeq ($(__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE
endif

ifeq ($(__CONFIG_MALLOC_USE_STDLIB), y)
  CONFIG_SYMBOLS += -D__CONFIG_MALLOC_USE_STDLIB
endif

ifeq ($(__CONFIG_MALLOC_TRACE), y)
  CONFIG_SYMBOLS += -D__CONFIG_MALLOC_TRACE
endif

ifeq ($(__CONFIG_OS_FREERTOS), y)
  CONFIG_SYMBOLS += -D__CONFIG_OS_FREERTOS
endif

ifeq ($(__CONFIG_BOOTLOADER), y)
  CONFIG_SYMBOLS += -D__CONFIG_BOOTLOADER
endif

# ----------------------------------------------------------------------------
# config chip name
# ----------------------------------------------------------------------------
ifeq ($(__CONFIG_CHIP_XR871), y)
  CONFIG_CHIP_NAME = xr871
else
  error "CONFIG_CHIP_NAME" is not defined!
endif
