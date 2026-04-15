#!/bin/sh
# Flash script for STM32H723ZG via STM32CubeProgrammer CLI
# Usage: called by cargo as runner with the ELF path as $1
STM32_CLI="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"
rust-objcopy -O ihex "$1" "$1.hex"
"$STM32_CLI" -c port=SWD freq=480 mode=UR -e all
"$STM32_CLI" -c port=SWD freq=480 mode=UR -w "$1.hex" 0x08000000 1 -rst
