/*
	modified linker script
	
	it leaves empty space for a bootloader (size XX KB)
	
	it puts the header of the application at a fixed address so that the bootloader can check the version of the application
	and locate the address of the entry function.
	
*/



OUTPUT_FORMAT("elf32-msp430","elf32-msp430","elf32-msp430")
OUTPUT_ARCH(msp:16)
MEMORY
{
  text				(rx)   	: ORIGIN = 0x4000,    LENGTH = 0xbfe0
  RAM				(rwx)  	: ORIGIN = 0x1100, 	  LENGTH = 0x2800
  vectors			(rw)  	: ORIGIN = 0xffe0,    LENGTH = 32
  infomem			(rx)	: ORIGIN = 0x1000,    LENGTH = 256
  infomemnobits		(rx)	: ORIGIN = 0x1000,    LENGTH = 256
}

NOCROSSREFS(.bootloader_text .text)

SECTIONS
{
  /* Internal text space.  */
  .text :
  {
 	PROVIDE(__application_start = . );
	PROVIDE(__application_header = . );
	
	/*
	   puts the app_header at the beginngin of the application text.
	   this address is well defined and can be directly accessed from the bootloader
	   the address = __bootloader_end ++, to the next page. in the case when the pages are correctly aligned, no computation has
	   to be executed in the bootloader
	*/
	
	*(.app_header)
    . = ALIGN(2);
    
	*(.delay_function)
    . = ALIGN(2);
    
    *(.init)
    *(.init0)  /* Start here after reset.  */
    *(.init1)
    *(.init2)  /* Copy data loop  */
    *(.init3)
    *(.init4)  /* Clear bss  */
    *(.init5)
    *(.init6)  /* C++ constructors.  */
    *(.init7)
    *(.init8)
    *(.init9)  /* Call main().  */
     __ctors_start = . ;
     *(.ctors)
     __ctors_end = . ;
     __dtors_start = . ;
     *(.dtors)
     __dtors_end = . ;
    . = ALIGN(2);
    *(.text)
    . = ALIGN(2);
    *(.text.*)
    . = ALIGN(2);
    *(.fini9)  /*   */
    *(.fini8)
    *(.fini7)
    *(.fini6)  /* C++ destructors.  */
    *(.fini5)
    *(.fini4)
    *(.fini3)
    *(.fini2)
    *(.fini1)
    *(.fini0)  /* Infinite loop after program termination.  */
    *(.fini)
    
    
    . = ALIGN(2);
    PROVIDE (__commands_start = .);
    *(.commands)                
    PROVIDE (__commands_end = .);    
    . = ALIGN(2);	

    PROVIDE(__application_end = . );

    _etext = .;
  }  > text
  
    .fwdata : 
  {
  												/* the size of reserved RAM area is 16 bytes (0x10 = 0x08 + 0x08) */
  												/* when storing more than one variable in this area, it's important to be careful about alighing of data in memory */
   		PROVIDE(__wd_resetinfo = . );			/* provide address of this location to the firmware so that the __wd_resetinfo can be accessed */
  		. += 0x0008; 							/* reserve space for the __wd_resetinfo, sizeof(__wd_resetinfo) == 8 so it's important to jump by 0x08 */
  		PROVIDE(__rtc_state = . ); 				/* provide address of this location to the firmware so that the __rtc_state can be accessed */
  		. += 0x0008; 							/* skip to the end of the reserved RAM area, as long as the sizeof(__rtc_state) is <= 8 this doesn't need to be modified */
    	. = ALIGN(2);
  }  > RAM
  
  .data   : AT (ADDR (.text) + SIZEOF (.text))
  {
     PROVIDE (__data_start = .) ;
    . = ALIGN(2);
    *(.data)
    . = ALIGN(2);
    *(.gnu.linkonce.d*)
    . = ALIGN(2);
     _edata = . ;
  }  > RAM

  /* Information memory.  */
  .infomem   :
  {
    *(.infomem)
    . = ALIGN(2);
    *(.infomem.*)
  }  > infomem
  /* Information memory (not loaded into MPU).  */
  .infomemnobits   :
  {
    *(.infomemnobits)
    . = ALIGN(2);
    *(.infomemnobits.*)
  }  > infomemnobits
  .bss  SIZEOF(.data) + ADDR(.data) :
  {
     PROVIDE (__bss_start = .) ;
    *(.bss)
    *(COMMON)
     PROVIDE (__bss_end = .) ;
     _end = . ;
  }  > RAM
  .noinit  SIZEOF(.bss) + ADDR(.bss) :
  {
     PROVIDE (__noinit_start = .) ;
    *(.noinit)
    *(COMMON)
     PROVIDE (__noinit_end = .) ;
     _end = . ;
  }  > RAM
  .vectors  :
  {
     PROVIDE (__vectors_start = .) ;
    *(.vectors*)
     _vectors_end = . ;
  }  > vectors
  
  
  
  
  
  
  
  /* Stabs for profiling information*/
  .profiler 0 : { *(.profiler) }
  /* Stabs debugging sections.  */
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info) *(.gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  PROVIDE (__stack = 0x3900) ;
  PROVIDE (__data_start_rom = _etext) ;
  PROVIDE (__data_end_rom   = _etext + SIZEOF (.data)) ;
  PROVIDE (__noinit_start_rom = _etext + SIZEOF (.data)) ;
  PROVIDE (__noinit_end_rom = _etext + SIZEOF (.data) + SIZEOF (.noinit)) ;
  PROVIDE (__subdevice_has_heap = 0) ;
}
