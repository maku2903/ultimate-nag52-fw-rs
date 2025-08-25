MEMORY
{
  FLASH (rx)    : ORIGIN = 0x00000000, LENGTH = 64K-512
  BL_INFO       : ORIGIN = 0x00000000 + 64K-512, LENGTH = 512
  CAN           : ORIGIN = 0x20000000, LENGTH = 64K
  BL_COMM (xrw) : ORIGIN = 0x20010000, LENGTH = 512
  RAM (xrw)     : ORIGIN = 0x20010200, LENGTH = 256K - 64K - 512
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