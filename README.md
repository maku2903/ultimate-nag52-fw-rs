# Ultimate-NAG52 firmware V2
Ultimate NAG52 V2 firmware (Rust version)

This is for the Alpha board gen 2.0 (Closing PCB testing), using Atmel SAME54 series processor.


## Startup process

1. CPU jumps to [preloader](preloader/) on power on
    * If bootloader requires updating, the bootloader scratch area is verified, before being copied to the bootloader memory location.
2. Preloader jumps to the [bootloader](bootloader/)
    * Bootloader will **not** jump to the application if one of the following conditions is true:
        1. Magic pin is shorted (SDA + GND on the EEPROM chip)
        2. `stay-in-bootloader` compile flag is set
        3. Application flashing check failed
        4. The application has told the bootloader to launch (Enter reprogramming mode via diagnostics)
        5. Panic, or a hard/bus fault in the application was triggered.
        6. Watchdog in the application was triggered
    * The bootloader allows for updating itself and the application via KWP2000 protocol, over CAN or USB
3. Bootloader jumps to the [application](firmware/)

### Time to boot

This is taken for a nominal boot (No regions are to be flashed)

|Timestamp||
|:-:|:-:|
|0ms|Power on|
|0.5ms|Preloader start|
|2.7ms|Bootloader start|
|2.72ms|Bootloader verify app|
|3ms|Application start|

## Memory map

### Flash (1MB)

|Start address|End address|Size|Usage|
|:-:|:-:|:-:|:-:|
|0x00000000|0x00001FFF|8KB|Preloader|
|0x00002000|0x0001DFFF|112KB|Bootloader|
|0x0001E000|0x00017FFF|8KB|Metadata (Bootloader and preloader)|
|0x00018000|0x00100000|896KB|Application/Bootloader scratch [^1]|

[^1]:- Bootloader update will write into application space, destroying it, as the application has to be updated after updating the bootloader.

---

### RAM (256KB)

|Start address|End address|Size|Usage|
|:-:|:-:|:-:|:-:|
|0x20000000|0x2000FFFF|64KB|MCAN CAN0 Buffer|
|0x20010000|0x200103FF|1KB|Bootloader<->Application communication|
|0x20010400|0x20040000|191KB|RAM
