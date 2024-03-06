OUTPUT_ARCH(powerpc:common)
ENTRY(_start)
EXTERN(_start)
SECTIONS
{
  .text      :
  {
    _start = .;
    *(.text)
    *(.fixup)
    _etext = .;
  }
  .data    :
  {
    *(.rodata*)
    *(.data*)
    *(.sdata*)
    *(.got2)
  }
  .dynsym : { *(.dynsym) }
  .dynstr : { *(.dynstr) }
  .dynamic :
  {
    __dynamic_start = .;
    *(.dynamic)
  }
  .hash : { *(.hash) }
  .interp : { *(.interp) }
  .rela.dyn : { *(.rela*) }

  .bss       :
  {
    _edata  =  .;
    __bss_start = .;
    *(.sbss)
    *(.bss)
    *(COMMON)
    _end = . ;
  }
}
