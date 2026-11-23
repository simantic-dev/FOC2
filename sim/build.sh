#!/usr/bin/env bash
# Rebuild sim/foc2.elf (CAN ID 1) and sim/foc2_id2.elf (CAN ID 2) from this repo's 01-Firmware/B-G431B-ESC1-v2.
# Upstream is a CubeIDE project with no Makefile; this compiles the same sources the .cproject does, in a
# scratch copy, with FOC2's documented user settings flipped (SENSOR_TYPE -> AS5600_I2C,
# PERFORM_AUTO_CALIBRATION_AT_STARTUP on) and, for joint 2, REG_ID_VALUE 1 -> 2. No logic is changed.
# Usage: sim/build.sh   (needs arm-none-eabi-gcc; tested with 15.3)
set -euo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
WORK=$(mktemp -d); trap 'rm -rf "$WORK"' EXIT
for id in 1 2; do
  rm -rf "$WORK/fw"; cp -R "$HERE/../01-Firmware/B-G431B-ESC1-v2" "$WORK/fw"; cd "$WORK/fw"
  sed -i.bak -e 's|^#define SENSOR_TYPE AS5048A_PWM|#define SENSOR_TYPE AS5600_I2C|' \
    -e 's|^//#define PERFORM_AUTO_CALIBRATION_AT_STARTUP|#define PERFORM_AUTO_CALIBRATION_AT_STARTUP|' Core/Src/main.c
  sed -i.bak -E "s@^(#define REG_ID_VALUE[[:space:]]+)1([[:space:]]|\$)@\1$id\2@" Core/Inc/control_table.h
  grep -q "^#define REG_ID_VALUE[[:space:]]*$id" Core/Inc/control_table.h
  arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32G431xx \
    -O2 -g -ffunction-sections -fdata-sections -ICore/Inc -IDrivers/STM32G4xx_HAL_Driver/Inc \
    -IDrivers/CMSIS/Device/ST/STM32G4xx/Include -IDrivers/CMSIS/Include \
    Core/Src/*.c Drivers/STM32G4xx_HAL_Driver/Src/*.c Core/Startup/startup_stm32g431cbux.s \
    -TSTM32G431CBUX_FLASH.ld -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -lm -o foc2.elf
  [ $id = 1 ] && out=foc2.elf || out=foc2_id2.elf
  arm-none-eabi-strip --strip-debug foc2.elf -o "$HERE/$out"
done
