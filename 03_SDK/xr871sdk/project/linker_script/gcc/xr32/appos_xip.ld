/* Linker script to configure memory regions. */
MEMORY
{
	RAM     (rwx) : ORIGIN = 0x00010000, LENGTH = 448K
	RAM_EXT (rwx) : ORIGIN = 0x60000000, LENGTH = 384K
	FLASH   (rx)  : ORIGIN = 0x10000000, LENGTH = 1024K
}

__RAM_BASE = ORIGIN(RAM);

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   _estack
 */
ENTRY(Reset_Handler)

SECTIONS
{
	.xip :
	{
		. = ALIGN(4);
		__xip_start = .;

		/* MUST not put IRQ handler/callback in .xip section */
		*libmp3.a: (.text .text.* .rodata .rodata.*)
		*libaac.a: (.text .text.* .rodata .rodata.*)
		*libamr.a: (.text .text.* .rodata .rodata.*)
		*libamren.a: (.text .text.* .rodata .rodata.*)
		*libcedarx.a: (.text .text.* .rodata .rodata.*)
		*libpcm.a: (.text .text.* .rodata .rodata.*)
		*libaudmgr.a: (.text .text.* .rodata .rodata.*)
		*libcjson.a: (.text .text.* .rodata .rodata.*)
		*libfs.a: (.text .text.* .rodata .rodata.*)
		*libefpg.a: (.text .text.* .rodata .rodata.*)
		*libota.a: (.text .text.* .rodata .rodata.*)
		*libxz.a: (.text .text.* .rodata .rodata.*)
		*libutil.a: (.text .text.* .rodata .rodata.*)

		*libm.a: (EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .text
		          EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .text.*
		          EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .rodata
		          EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .rodata.*)

		*project/common/cmd*.o (.text .text.* .rodata .rodata.*)
		*project/common/framework/sysinfo.o (.text .text.* .rodata .rodata.*)
		*project/common/framework/fs_ctrl.o (.text .text.* .rodata .rodata.*)
		*project/common/framework/sys_ctrl/container.o (.text .text.* .rodata .rodata.*)
/*		*project/common/framework/sys_ctrl/event_queue.o (.text .text.* .rodata .rodata.*) */
		*project/common/framework/sys_ctrl/observer.o (.text .text.* .rodata .rodata.*)
		*project/common/framework/sys_ctrl/publisher.o (.text .text.* .rodata .rodata.*)
		*project/common/framework/sys_ctrl/looper.o (.text .text.* .rodata .rodata.*)
/*		*project/common/framework/sys_ctrl/sys_ctrl.o (.text .text.* .rodata .rodata.*) */
		*command.o (.text .text.* .rodata .rodata.*)

		*(.xip_text* .xip_rodata*)

		. = ALIGN(4);
		__xip_end = .;
	} > FLASH

	.text_ext :
	{
		. = ALIGN(4);
		__text_ext_start__ = .;

		/* put text section in the extern RAM
		 * eg. *libfoo.a: (.text* .nonxip_text* .rodata* .nonxip_rodata*)
		 */

		. = ALIGN(4);
		__text_ext_end__ = .;
	} > RAM_EXT

	.data_ext :
	{
		. = ALIGN(4);
		__data_ext_start__ = .;

		/* put data section in the extern RAM
		 * eg. *libfoo.a: (.data* .nonxip_data* vtable)
		 */

		. = ALIGN(4);
		__data_ext_end__ = .;
	} > RAM_EXT

	.bss_ext :
	{
		. = ALIGN(4);
		__bss_ext_start__ = .;

		/* put bss section in the extern RAM
		 * eg. *libfoo.a: (.bss* COMMON)
		 */

		. = ALIGN(4);
		__bss_ext_end__ = .;
	} > RAM_EXT

	.text :
	{
		. = ALIGN(4);
		__text_start__ = .;

		KEEP(*(.isr_vector))
		KEEP(*(.text.Reset_Handler))
		KEEP(*(.cpu_text))
		*(.text*)
		*(.nonxip_text*)
		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
		*crtbegin.o(.dtors)
		*crtbegin?.o(.dtors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
		*(SORT(.dtors.*))
		*(.dtors)

		*(.rodata*)
		*(.nonxip_rodata*)

		KEEP(*(.eh_frame*))

		. = ALIGN(4);
		__text_end__ = .;
	} > RAM

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > RAM

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > RAM
	__exidx_end = .;

	. = ALIGN(4);
	__etext = .;
	_sidata = .;

	.data :
	{
		. = ALIGN(4);
		__data_start__ = .;
		_sdata = .;

		*(vtable)
		*(.data*)
		*(.nonxip_data*)

		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);

		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);

		KEEP(*(.jcr*))

		. = ALIGN(4);
		__data_end__ = .;
		_edata = .;
	} > RAM

	.bss :
	{
		. = ALIGN(4);
		__bss_start__ = .;
		_sbss = .;

		*(.bss*)
		*(COMMON)

		. = ALIGN(4);
		__bss_end__ = .;
		_ebss = .;
	} > RAM

	.heap (COPY):
	{
		__end__ = .;
		end = __end__;
		*(.heap*)
		__HeapLimit = .;
	} > RAM

	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (COPY):
	{
		*(.stack*)
	} > RAM

	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__StackTop = ORIGIN(RAM) + LENGTH(RAM);
	_estack = __StackTop;
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __StackTop);

	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}
