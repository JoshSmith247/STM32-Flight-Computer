/* STM32H723ZGT6 memory layout
 * Flash:    1 MB   @ 0x08000000
 * ITCM:     64 KB  @ 0x00000000 (instruction tightly coupled, no DMA)
 * DTCM:     128 KB @ 0x20000000 (data tightly coupled, no DMA)
 * AXI SRAM: 320 KB @ 0x24000000 (DMA accessible — use as main RAM)
 * SRAM1:    16 KB  @ 0x30000000
 * SRAM2:    16 KB  @ 0x30004000
 * SRAM4:    16 KB  @ 0x38000000
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   : ORIGIN = 0x24000000, LENGTH = 320K   /* AXI SRAM — DMA accessible */
}

/* Stack placed at end of RAM */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
