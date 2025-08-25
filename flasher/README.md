# Ultimate-NAG52 V2 flasher

This tool is designed to write/read to the TCUs bootloader

## Note about interface type `can-fast` (Linux Only)

The can-fast interface sets STMIN and BS to 0 for ISO-TP communication.

Therefore, this is designed only for bench-testing purposes only. It is not
designed to be used inside a vehicle with communicating ECUs.