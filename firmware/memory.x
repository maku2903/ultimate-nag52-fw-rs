MEMORY
{
  # Preloader          = 0-8KB
  # Bootloader + info  = 8-128KB
  FLASH (rx)    : ORIGIN = 0x00000000 + 128K      , LENGTH = 0x00100000 - 128K
  APP_INFO      : ORIGIN = 0x00080000 - 64K - 512 , LENGTH = 512
  CAN           : ORIGIN = 0x20000000             , LENGTH = 64K
  BL_COMM (xrw) : ORIGIN = 0x20010000             , LENGTH = 1K
  RAM (xrw)     : ORIGIN = 0x20010400             , LENGTH = 256K - 64K - 1K
}

SECTIONS {
  .can (NOLOAD) :
  {
    *(.can .can.*);
  } > CAN
  .app_info :  {
    *(.app_info);
    . = ALIGN(4);
  } > APP_INFO
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);