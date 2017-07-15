# configuration options

__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE = 1

__CONFIG_OS_USE_FREERTOS = 1

__CONFIG_MALLOC_USE_STDLIB = 1

# trace heap memory usage and error when using malloc, free, etc.
__CONFIG_MALLOC_TRACE = 0

__CONFIG_CHIP_XR871 = 1

__CONFIG_ARCH_DUAL_CORE = 1

__CONFIG_ARCH_APP_CORE = 1

__CONFIG_ARCH_NET_CORE = 0

__CONFIG_BOOTLOADER = 0

ifeq ($(__CONFIG_BOOTLOADER), 1)
  __CONFIG_ARCH_DUAL_CORE = 0
endif

################################################################################
CONFIG_SYMBOLS =

ifeq ($(__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE), 1)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE
endif

ifeq ($(__CONFIG_OS_USE_FREERTOS), 1)
  CONFIG_SYMBOLS += -D__CONFIG_OS_USE_FREERTOS
endif

ifeq ($(__CONFIG_MALLOC_USE_STDLIB), 1)
  CONFIG_SYMBOLS += -D__CONFIG_MALLOC_USE_STDLIB
endif

ifeq ($(__CONFIG_MALLOC_TRACE), 1)
  CONFIG_SYMBOLS += -D__CONFIG_MALLOC_TRACE
endif

ifeq ($(__CONFIG_CHIP_XR871), 1)
  CONFIG_SYMBOLS += -D__CONFIG_CHIP_XR871
endif

ifeq ($(__CONFIG_ARCH_DUAL_CORE), 1)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_DUAL_CORE
endif

ifeq ($(__CONFIG_ARCH_APP_CORE), 1)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_APP_CORE
endif

ifeq ($(__CONFIG_ARCH_NET_CORE), 1)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_NET_CORE
endif

ifeq ($(__CONFIG_BOOTLOADER), 1)
  CONFIG_SYMBOLS += -D__CONFIG_BOOTLOADER
endif

################################################################################
# CONFIG_CHIP_NAME
ifeq ($(__CONFIG_CHIP_XR871), 1)
  CONFIG_CHIP_NAME = xr871
else
  error "CONFIG_CHIP_NAME" is not defined!
endif
