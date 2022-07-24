/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
SEARCH_DIR(.)

VECTOR_TABLE_ALIGNMENT = 0x200; /* vector table must at least be 512-byte aligned */
STACK_SIZE = 0x200; /* 0x100 seems to be to little */

/* Memory Spaces Definitions */
MEMORY
{
	bootloader (rx) : ORIGIN = 0x1FFFB000, LENGTH = 18K
	rom        (rx) : ORIGIN = 0x08000000, LENGTH = 128K
	ram       (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Section Definitions */
SECTIONS
{
	.text :
	{
		_sfixed = .;
		KEEP(*(.vectors .vectors.*))
		*(.text .text.* .gnu.linkonce.t.*)
		*(.glue_7t) *(.glue_7)
		*(.rodata .rodata* .gnu.linkonce.r.*)
		*(.ARM.extab* .gnu.linkonce.armextab.*)

		/* Support C constructors, and C destructors in both user code
		   and the C library. This also provides support for C++ code. */
		. = ALIGN(4);
		KEEP(*(.init))
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;

		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;

		. = ALIGN(4);
		KEEP (*crtbegin.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*crtend.o(.ctors))

		. = ALIGN(4);
		KEEP(*(.fini))

		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;

		KEEP (*crtbegin.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*crtend.o(.dtors))

		. = ALIGN(4);
		_efixed = .;            /* End of text section */
	} > rom



	/* .ARM.exidx is sorted, so has to go in its own output section.  */
	PROVIDE_HIDDEN (__exidx_start = .);
	.ARM.exidx :
	{
	  *(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > rom
	PROVIDE_HIDDEN (__exidx_end = .);

	/* Must align properly so
	 * - CRC computation reads full words
	 * - that the app tag starts at the beginning of the section (for simple processing)
	 */
	. = ALIGN(4);

	.dfutag : {
		KEEP (*(.dfutag));
	} > rom

	. = ALIGN(4);
	_etext = .;

	.relocate : AT (_etext)
	{
		. = ALIGN(4);
		_srelocate = .;
		*(.ramfunc .ramfunc.*);
		*(.data .data.*);
		. = ALIGN(4);
		_erelocate = .;
	} > ram

	/*
	.vectors (NOLOAD):
	{
		. = ALIGN(VECTOR_TABLE_ALIGNMENT);
		_svectors = .;
		. = . + VECTOR_TABLE_ALIGNMENT;
		_evectors = .;
	} > ram
	*/

	/* .bss section which is used for uninitialized data */
	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sbss = . ;
		_szero = .;
		*(.bss .bss.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = . ;
		_ezero = .;
	} > ram

	/* stack section */
	.stack (NOLOAD):
	{
		. = ALIGN(8);
		_sstack = .;
		. = . + STACK_SIZE;
		. = ALIGN(8);
		_estack = .;
	} > ram

	. = ALIGN(4);
	_end = . ;
}

