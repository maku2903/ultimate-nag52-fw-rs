MEMORY
{
  # Preloader = 0-8KB
  # Bootloader + info = 8-96KB
  FLASH (rx)    : ORIGIN = 0x00000000 + 8K,        LENGTH = 112K
  BL_INFO       : ORIGIN = 0x00000000 + 120K,      LENGTH = 8K
  CAN           : ORIGIN = 0x20000000,             LENGTH = 64K
  BL_COMM (xrw) : ORIGIN = 0x20000000 + 64K,       LENGTH = 512
  RAM (xrw)     : ORIGIN = 0x20000000 + 64K + 512, LENGTH = 256K - 64K - 512
}

SECTIONS {
  .can (NOLOAD) :
  {
    *(.can .can.*);
  } > CAN
  .bl_info :  {
    KEEP(*(.bl_info));
    . = ALIGN(4);
  } > BL_INFO
  .bl_comm :  {
    *(.bl_comm);
    . = ALIGN(4);
  } > BL_COMM
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);