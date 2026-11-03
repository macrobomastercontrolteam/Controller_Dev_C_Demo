## Overview

STM32 examples for the RoboMaster Development Board Type C (STM32F407IGH6).
Chinese source comments have been translated into English. The original Keil
projects, CubeMX configurations, and PDF tutorials are retained.
Demo folder names use lowercase snake_case and retain the tutorial chapter
number, for example `17_chassis_control`. Letter suffixes distinguish multiple
demos from the same chapter.
Source and text configuration files use UTF-8 without a BOM, not GBK/GB18030.
The [EditorConfig settings](.editorconfig) specify UTF-8 for future edits.

### Original software environment

- Toolchain/IDE: MDK-ARM V5
- STM32F4xx_DFP pack: 2.13.0
- STM32CubeMX: 5.2.1
- STM32Cube firmware: FW_F4 V1.21.1
- FreeRTOS: 10.0.1
- CMSIS-RTOS: 1.02

## Building on a Windows laptop

Install GNU Arm Embedded Toolchain and GNU Make, and put their executable
directories on `PATH`. No Keil license, Python, CubeMX regeneration, or network
access is required to build. These Makefiles follow the structure of the M2006
reference Makefile, but target the F407 rather than the F427.

This laptop uses ARM GCC 10.3.1 (10.3-2021.10). Both `make` (MSYS 3.81)
and `mingw32-make` (native Windows 3.82.90) are supported inside demo folders.

Enter the demo you want to build and run `make`:

```powershell
cd 17_chassis_control
make

# Optional parallel build, image size, or cleanup for this demo only.
make -j4
make size
make clean
```

The 24 focused teaching demos have compact [Makefiles](01_led_on/Makefile)
containing only source lists and project settings. Repeated build rules live in
[common.mk](build_support/common.mk). The complete standard robot retains the
Infantry Common project's self-contained Makefile, startup file, and linker
script. Running `make` inside a demo builds only that demo and produces
`build/<project>.elf`, `.bin`, `.hex`, and `.map`. Names follow the original
Keil output names; for example, the standard robot outputs
`build/standard_robot.elf`. Object paths preserve the source-directory structure
to avoid collisions between identically named files.

If the ARM compiler is not on `PATH`, provide its full prefix:

```powershell
mingw32-make -j4 "PREFIX=C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin\arm-none-eabi-"
```

Optimization defaults to `-O2`. Clean before changing compiler options or
toolchain versions, for example `mingw32-make clean` followed by
`mingw32-make -j4 OPT=-Og`.

### Build implementation

- [Root Makefile](Makefile): builds all demos or a named demo.
- [Shared rules](build_support/common.mk): Cortex-M4, Thumb, FPv4-SP-D16,
  hard-float ABI, dependency tracking, newlib-nano, and ELF/BIN/HEX outputs.
- [Linker script](build_support/STM32F407IGHx_FLASH.ld): 1 MiB flash at
  `0x08000000`, 128 KiB DMA-accessible SRAM at `0x20000000`, and the separately
  declared 64 KiB CCM region. Ordinary data and DMA buffers stay in SRAM, not CCM.
- [GCC startup](build_support/startup_stm32f407xx.s): the same 98 vector entries
  as the supplied Keil startup; calls `SystemInit`, initializes data/BSS, runs
  C runtime initialization, and enters `main`.
- [Heap support](build_support/sysmem.c): bounded newlib heap with recursive
  interrupt-preserving allocator locks. Heap and stack reservations match each
  demo's original Keil startup. FreeRTOS keeps its separate `heap_4` allocator.

Per-demo source lists and include paths follow their `.uvprojx` files. Update the
demo's Makefile when adding source files or regenerating a CubeMX project.
Each project includes the shared rules. The startup, linker script,
heap support, and applicable FreeRTOS/DSP sources also remain in `build_support`,
so keep that directory alongside the demo folders.
Application integer aliases now use `<stdint.h>` and packed structures use CMSIS
`__PACKED_STRUCT`, preserving their wire/flash layout with both GCC and Keil.
Inconsistent static declarations and the Keil-only NOP spelling were corrected.
Strict aliasing is disabled for compatibility with the existing inverse-square-root
implementations.

The GCC builds use these vendored upstream sources, with licenses retained:

- [FreeRTOS GCC Cortex-M4F port](build_support/FreeRTOS/ARM_CM4F):
  [FreeRTOS V10.0.1](https://github.com/FreeRTOS/FreeRTOS/tree/V10.0.1/FreeRTOS/Source/portable/GCC/ARM_CM4F).
  This replaces the RVDS port only in GCC builds; original Keil projects still use RVDS.
- [CMSIS-DSP sine, cosine, and tables](build_support/CMSIS-DSP):
  [CMSIS_4 v4.5.0](https://github.com/ARM-software/CMSIS_4/tree/v4.5.0/CMSIS/DSP_Lib/Source),
  DSP revision 1.4.5/1.4.5a, compatible with the supplied 1.4.5 headers.
  GCC compiles these instead of linking the incompatible Keil DSP archive.

The three larger robot demos retain the supplied `AHRS.lib`: its hard-float
interfaces use scalar numbers and pointers, not `wchar_t`. GNU ld reports its
legacy 2-byte `wchar_t` metadata versus GCC's 4-byte setting. This warning is left
visible; do not extend the binary interface with wide characters or enums without
rebuilding the library for GCC.

### Validation and limitations

All 25 demos have been compiled and linked on this laptop. Validation checks the
ELF/BIN/HEX outputs, vector tables, reset entry, flash/SRAM bounds, heap/stack
reservations, and FreeRTOS handler bindings. Packed protocol/calibration layouts
and heap bounds/allocator locking are also checked.

Builds are not warning-free. Existing application warnings include unused
variables, signedness, potentially uninitialized I2C handles, packed calibration
pointer alignment, and a possible USB formatting-buffer overflow. They are not
silenced or treated as proof of runtime safety. The USB formatting issue and
other application warnings should be addressed before deploying the complete robot.

Hardware execution is **not verified**: STM32CubeProgrammer reported
`No ST-Link detected`, and verification was limited to laptop builds at the user's
request. No board was flashed, erased, or reset. Successful compilation does not
validate sensor readings, timing, motor behavior, or peripheral wiring.

## Optional hardware programming

Install STM32CubeProgrammer separately. Connect ST-Link via SWD and power the board.
To enumerate probes without halting, resetting, or programming the target:

```powershell
mingw32-make probe "PROGRAMMER=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
```

Building never flashes automatically. Flashing replaces the installed firmware
and resets the board. Disconnect motors, servos, heaters, and other actuators
before testing unfamiliar firmware. There is deliberately no aggregate flash
target or mass-erase target.

Only when ready to overwrite firmware, explicitly opt in for a selected demo:

```powershell
mingw32-make -C 02_led_blink flash CONFIRM_FLASH=yes "PROGRAMMER=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
```

This writes the BIN at `0x08000000`, verifies the programmed bytes, and resets the
board. Programmer verification is not a functional test. For the LED demo, also
observe the LEDs on the actual board.

## Included firmware and tutorial

- [Original user firmware](00_cubemx_project_template/user_program.bin)
- [RoboMaster Type C embedded software tutorial (Chinese PDF)](RoboMaster开发板C型嵌入式软件教程文档.pdf)

## Examples

* [0. New CubeMX project](00_cubemx_project_template)
* [1. Turn on an LED](01_led_on)
* [2. Blink LEDs](02_led_blink)
* [3. Timer-driven LED blinking](03_timer_led_blink)
* [4a. PWM LED brightness](04a_pwm_led_dimming)
* [4b. Flowing LED pattern](04b_led_flow_pattern)
* [5a. Buzzer](05a_buzzer)
* [5b. Servo motor](05b_servo_motor)
* [6. Button external interrupt](06_button_external_interrupt)
* [7. ADC supply-voltage measurement](07_adc_supply_voltage)
* [8. UART transmission and reception](08_uart_transmit_receive)
* [9a. Remote-control reception using DMA](09a_remote_control_dma)
* [9b. Print remote-control data over UART](09b_remote_control_uart_output)
* [10. Flash reads and writes](10_flash_read_write)
* [11. IST8310 magnetometer](11_ist8310_magnetometer)
* [12. OLED display](12_oled_display)
* [13. BMI088 sensor data over SPI](13_bmi088_spi)
* [14a. CAN motor control](14a_can_motor_control)
* [14b. PWM snail motor control](14b_pwm_snail_motor_control)
* [15. FreeRTOS LED tasks](15_freertos_led_tasks)
* [16. IMU temperature control](16_imu_temperature_control)
* [17. Chassis control](17_chassis_control)
* [18. Inertial navigation](18_inertial_navigation)
* [19. Gimbal control](19_gimbal_control)
* [20. Complete standard robot](20_standard_robot)

## Infantry reference control migration

The chassis, gimbal, and standard-robot examples use the Infantry 2026 omni
control logic from `C:\RoboMaster\Workspace_Infantry_Reference\Infantry_Common`.

- `17_chassis_control` keeps the chassis lesson isolated. It includes the
  reference chassis behavior, omni-wheel control, MG4010 command handling,
  power limiting, VT13 input, CAN protocol, PID, and fault detection. Interfaces
  owned by the full robot (gimbal, CV, shooter, UI, and referee services) use
  safe demo adapters. Nominal power telemetry lets the standalone demo operate
  until real CAN telemetry is received.
- `19_gimbal_control` keeps the gimbal lesson isolated. It includes the
  reference gimbal behavior, DaMiao motor control, VT13 input, CAN protocol,
  PID, INS feedback, and fault detection. Shooter, CV, chassis, UI, and referee
  dependencies use fail-safe demo adapters.
- `20_standard_robot` mirrors the Infantry Common codebase, excluding only its
  `.git` metadata and generated `build` directory. It uses the reference
  Makefile, startup file, linker script, and default `INFANTRY_2026_OMNI`
  configuration.
