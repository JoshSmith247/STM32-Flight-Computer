/* STM32F405RG memory layout
 * Flash: 1 MB  @ 0x08000000
 * SRAM1: 112 KB @ 0x20000000
 * SRAM2: 16 KB  @ 0x2001C000
 * CCM:   64 KB  @ 0x10000000 (data only, no DMA)
 */
MEMORY
{
    FLASH  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM    : ORIGIN = 0x20000000, LENGTH = 128K
    CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Stack placed at end of RAM */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
