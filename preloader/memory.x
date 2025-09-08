MEMORY
{
  # Preloader = 0-8KB
  FLASH (rx)    : ORIGIN = 0x00000000, LENGTH = 8K
  BL_COMM (xrw) : ORIGIN = 0x20000000 + 64K,      LENGTH = 512
  RAM (xrw)     : ORIGIN = 0x20000000 + 64K + 512,LENGTH = 256K - 64K - 512
}

SECTIONS {
  .bl_comm :  {
    *(.bl_comm);
    . = ALIGN(4);
  } > BL_COMM
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);