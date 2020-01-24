
MEMORY
{
     FLASH (rx) : ORIGIN = 0x8000000 LENGTH = 0x20000
     SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000
}

ENTRY(RESET_Handler)

SECTIONS
{
	.text :
	{
	  *(.vectors*)
	  *(.text*)
	  *(.rodata*)
	  _etext = .;
	} > FLASH

	.data : AT (_etext)
	{
          _data = .;
	  *(.data*)
	  _edata = .;
	} > SRAM

	/* zero initialized data */
	.bss :
	{
	  _bss = .;
	  *(.bss*)
	  *(COMMON)
	  _ebss = .;
	} > SRAM
	
	/* Where we put the heap with cr_clib */
	.cr_heap : 
	{
	  end = .;
	  _pvHeapStart = .;
	} > SRAM

/*
	Note: (ref: M0000066)
	Moving the stack down by 16 is to work around a GDB bug.
	This space can be reclaimed for Production Builds.
*/	
	_data_sz   = (_edata - _data);
	_bss_sz   = _ebss - _bss;	
}
