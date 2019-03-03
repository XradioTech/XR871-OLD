#
# global config options
#

# ----------------------------------------------------------------------------
# config options
# ----------------------------------------------------------------------------
# chip and cpu
__CONFIG_CHIP_TYPE ?= xr871
__CONFIG_CPU_CM4F ?= y

ifeq ($(__CONFIG_CHIP_TYPE), xr871)
  __CONFIG_CHIP_XR871 := y
  __CONFIG_CHIP_SERIES_XR32 := n
endif
ifeq ($(__CONFIG_CHIP_TYPE), xr32)
  __CONFIG_CHIP_XR871 := y
  __CONFIG_CHIP_SERIES_XR32 := y
endif

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

# wrap standard input/output/error functions
__CONFIG_LIBC_WRAP_STDIO ?= y

# heap managed by stdlib
__CONFIG_MALLOC_USE_STDLIB ?= y

# trace heap memory usage and error when using malloc, free, etc.
__CONFIG_MALLOC_TRACE ?= n

# os
__CONFIG_OS_FREERTOS ?= y

# lwIP
#   - y: lwIP 1.4.1, support IPv4 stack only
#   - n: lwIP 2.x.x, support dual IPv4/IPv6 stack
__CONFIG_LWIP_V1 ?= y

# mbuf implementation mode
#   - mode 0: continuous memory allocated from net core
#   - mode 1: continuous memory (lwip pbuf) allocated from app core
__CONFIG_MBUF_IMPL_MODE ?= 0

# link function level's text/rodata/data to ".xip" section
__CONFIG_XIP_SECTION_FUNC_LEVEL ?= n

# bin compression
__CONFIG_BIN_COMPRESS ?= n

# enable/disable bootloader, y to enable bootloader and disable some features
__CONFIG_BOOTLOADER ?= n

# ----------------------------------------------------------------------------
# config symbols
# ----------------------------------------------------------------------------
CONFIG_SYMBOLS =

ifeq ($(__CONFIG_CHIP_XR871), y)
  CONFIG_SYMBOLS += -D__CONFIG_CHIP_XR871
endif

ifeq ($(__CONFIG_CHIP_SERIES_XR32), y)
  CONFIG_SYMBOLS += -D__CONFIG_CHIP_SERIES_XR32
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

ifeq ($(__CONFIG_LIBC_PRINTF_FLOAT), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_PRINTF_FLOAT
endif

ifeq ($(__CONFIG_LIBC_SCANF_FLOAT), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_SCANF_FLOAT
endif

ifeq ($(__CONFIG_LIBC_WRAP_STDIO), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_WRAP_STDIO
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

ifeq ($(__CONFIG_LWIP_V1), y)
  CONFIG_SYMBOLS += -D__CONFIG_LWIP_V1
endif

CONFIG_SYMBOLS += -D__CONFIG_MBUF_IMPL_MODE=$(__CONFIG_MBUF_IMPL_MODE)

ifeq ($(__CONFIG_XIP_SECTION_FUNC_LEVEL), y)
  CONFIG_SYMBOLS += -D__CONFIG_XIP_SECTION_FUNC_LEVEL
endif

ifeq ($(__CONFIG_BIN_COMPRESS), y)
  CONFIG_SYMBOLS += -D__CONFIG_BIN_COMPRESS
endif

ifeq ($(__CONFIG_BOOTLOADER), y)
  CONFIG_SYMBOLS += -D__CONFIG_BOOTLOADER
endif

# ----------------------------------------------------------------------------
# config chip name
# ----------------------------------------------------------------------------
CONFIG_CHIP_NAME := $(__CONFIG_CHIP_TYPE)
