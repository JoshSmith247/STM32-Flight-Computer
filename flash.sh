#!/bin/sh
# Flash via USB DFU bootloader — no SWD/NRST needed.
#
# Before running: put board in DFU mode —
#   1. Hold BOOT0 button
#   2. Press and release RESET
#   3. Release BOOT0
#   (board should appear as "STM32 BOOTLOADER" in dfu-util -l)
#
# Usage: called by cargo as runner with the ELF path as $1
ELF="$1"

# Convert ELF → raw binary
rust-objcopy -O binary "$ELF" "$ELF.bin"

# Check DFU device is present
if ! dfu-util -l 2>/dev/null | grep -q "STM32\|0483:df11"; then
    echo ""
    echo "ERROR: No DFU device found."
    echo "Put the board in DFU mode first:"
    echo "  1. Hold BOOT0 button"
    echo "  2. Press and release RESET"
    echo "  3. Release BOOT0"
    echo "Then re-run cargo run."
    exit 1
fi

# Flash: alt=0 targets internal flash, :leave resets to user firmware after
dfu-util -a 0 -s 0x08000000:leave -D "$ELF.bin"
