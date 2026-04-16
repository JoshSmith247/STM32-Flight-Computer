/* STM32H723ZGT6 memory layout
 * Flash:     1 MB   @ 0x08000000
 * ITCM:      64 KB  @ 0x00000000 (Fastest - Instructions)
 * DTCM:      128 KB @ 0x20000000 (Fastest - Data)
 * AXI SRAM:  320 KB @ 0x24000000 (Main RAM - DMA accessible)
 */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM (rwx)  : ORIGIN = 0x24000000, LENGTH = 320K
}

/* The stack typically resides at the end of the primary RAM region */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);