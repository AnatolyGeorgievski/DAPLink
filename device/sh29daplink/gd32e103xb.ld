/* Memory Spaces Definitions */
_IFLASH_PAGE_SIZE = 1K;

MEMORY
{
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 32K
	CCM (xrw) : ORIGIN = 0x10000000, LENGTH = 0K /* core coupled memory */
	FLASH (rx): ORIGIN = 0x08000000, LENGTH = 128K
	FLASHPATCH (r) : ORIGIN = 0x00000000, LENGTH = 0
	ENDFLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 0
	FLASHB1  (rx)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB0 (rx)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB1 (rx)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB2 (rx)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB3 (rx)  : ORIGIN = 0x00000000, LENGTH = 0
}

/* higher address of the user mode stack */
_estack = 0x20008000;

/* Sections Definitions */

SECTIONS
{
	/* for Cortex devices, the beginning of the startup code is stored in the .isr_vector section, which goes to FLASH */
	.isr_vector :
	{
	. = ALIGN(4);
		KEEP(*(.isr_vector))            /* Startup code */
	. = ALIGN(4);
	} >FLASH

	/* the program code is stored in the .text section, which goes to Flash */
	.text :
	{
		. = ALIGN(4);
		
		*(.text)                   /* remaining code */
		*(.text.*)                   /* remaining code */
		*(.rodata)                 /* read-only data (constants) */
		*(.rodata*)
		*(.glue_7)
		*(.glue_7t)

		. = ALIGN(4);
	 _etext = .;
		/* This is used by the startup in order to initialize the .data secion */
	 _sidata = _etext;
	} >FLASH

	/* This is the initialized data section
	The program executes knowing that the data is in the RAM
	but the loader puts the initial values in the FLASH (inidata).
	It is one task of the startup to copy the initial values from FLASH to RAM. */
	.data  : AT ( _sidata )
	{
		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .data secion */
		_sdata = . ;

		*(.data)
		*(.data.*)
		*(.RAMtext)
		*(.ramfunc) /* Added by iFred for compatibility reasons */

		. = ALIGN(4);

		__start_init_array = .;/* конструкторы */
		*(.init_array)
		__stop_init_array = .;
		__start_ModList = .;
		*(MOD)
		__stop_ModList = .;
		__start_r3_cmd_callbacks = .;
		*(r3_cmd_callbacks)
		__stop_r3_cmd_callbacks = .;
		__start_MIB = .;
		*(MIB)
		__stop_MIB = .;

		. = ALIGN(4);
		__start_config = .;
		*(CONFIG)
		. = ALIGN(4);
		__stop_config = .;
		. += 4;/* CRC32 */
		/* This is used by the startup in order to initialize the .data secion */
	 _edata = . ;
	} >RAM

	/* This is the uninitialized data section */
	.bss :
	{
		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .bss secion */
		_sbss = .;

		*(.bss)
		*(COMMON)

		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .bss secion */
	 _ebss = . ;
	} >RAM
	
	.heap_hope (NOLOAD) :
	{
		. = ALIGN(4);
		__start_heap = . ;
		*(HEAP)
	} >RAM

	.iflash_segment_tail (NOLOAD) : {
		. += SIZEOF(.data);
		_iflash_segment_end = ALIGN(_IFLASH_PAGE_SIZE);
	} >FLASH
	
}
