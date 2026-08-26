# Kinetis Simulator

C simulator for NXP Kinetis microcontrollers and their Arm Cortex-M4F cores.

## Features

- CPU Core: Armv7E-M instruction set (Thumb, Thumb-2, DSP, FPv4-SP-D16 floating-point unit).
- Core Peripherals: NVIC, SCB, SysTick, bit-band operations, MPU, breakpoints, and cycle-counted execution.
- Kinetis Peripherals: Flash/SRAM, eDMA, DMAMUX, SIM, MCG, WDOG, PIT, LPTMR, ADC, UART, SPI, I2C, USB, GPIO.
- Device Profiles: MK22FN12810, MK22FN12812, MK22FN25612, MK22FN256CAP12, MK22FN51212, MK22FN1M012, MK22FX51212, and MKV30F12810.
- Device Packages: AK, FM, LF, LH, MP, AH, LK, AP, BP, FX, LL, DC, MC, LQ, and MD variants supported by each profile.

The 50 MHz K22D5 devices use Cortex-M4 cores without floating-point units and a different peripheral map. They are outside this simulator's scope.

## Build

Requires GCC, Meson, and Ninja.

```
meson setup build/simulator --buildtype=release
meson compile -C build/simulator
```

### Build Targets

| Target | Type | Description |
| :--- | :--- | :--- |
| `kinetis_simulator` | Static Library | Kinetis device and Cortex-M4F core simulator. |
| `kinetis_firmware_image` | Static Library | ELF and raw binary image loader. |
| `kinetis_firmware_runner` | Executable | CLI tool to load and run firmware images. |
| `test` | Utility | Run all tests. |

## Run Firmware

```
kinetis_firmware_runner <IMAGE> --profile <DEVICE> --reset-address <ADDRESS> [OPTIONS]
```

### Runner Options

| Option | Description |
| :--- | :--- |
| `--reset-address <ADDR>` | Vector-table base address used for reset (required). |
| `--profile <DEVICE>` | Kinetis device profile (required). |
| `--package <CODE>` | Package code such as `LH`, `LL`, or `LQ`. |
| `--binary-address <ADDR>` | Base address for raw binary images. |
| `--stop-address <ADDR>` | Execution stop address. |
| `--max-instructions <N>` | Maximum instruction count. |
| `--max-cycles <N>` | Maximum clock cycle limit. |

## Use in Meson Projects

```meson
kinetis = subproject('kinetis-sim')
simulator = kinetis.get_variable('kinetis_simulator_dependency')

executable('your_target', 'main.c', dependencies: simulator)
```

## Run Tests

Run all tests:

```
meson test -C build/simulator
```

Run all tests with simulator source coverage:

```
meson setup build/test-coverage --buildtype=debug -Db_coverage=true
meson test -C build/test-coverage --num-processes 1 --no-suite census
meson compile -C build/test-coverage coverage-text
```

The coverage report requires GCC, gcov, and gcovr. The coverage run excludes exhaustive census
tests because their millions of instruction cases distort and can overflow GCC's coverage counters.
Normal test runs still include them.
