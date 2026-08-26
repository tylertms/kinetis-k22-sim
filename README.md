# Kinetis K22 Simulator

C simulator for the 100 MHz and 120 MHz NXP Kinetis K22F microcontroller families and their Arm Cortex-M4 cores with FPv4-SP-D16 floating-point units.

## Features

- CPU Core: Armv7E-M instruction set (Thumb, Thumb-2, DSP, FPv4-SP-D16 floating-point unit).
- Core Peripherals: NVIC, SCB, SysTick, bit-band operations, MPU, breakpoints, and cycle-counted execution.
- Kinetis K22 Peripherals: Flash/SRAM, eDMA, DMAMUX, SIM, MCG, WDOG, PIT, LPTMR, ADC, UART, SPI, I2C, USB, GPIO.
- Device Profiles: MK22F12810, MK22FN12812, MK22FN25612, MK22FN51212, MK22FN1M012, and MK22FX51212.
- Device Packages: AK, LH, MP, AH, LK, AP, BP, FX, LL, DC, MC, LQ, and MD variants supported by each profile.

The 50 MHz K22D5 devices use Cortex-M4 cores without floating-point units and a different peripheral map. They are outside this K22F simulator's scope.

## Build

```
meson setup build/simulator --buildtype=release
meson compile -C build/simulator
```

### Build Targets

| Target | Type | Description |
| :--- | :--- | :--- |
| `kinetis_k22_simulator` | Static Library | K22 device and Cortex-M4F core simulator. |
| `kinetis_k22_firmware_image` | Static Library | ELF and raw binary image loader. |
| `kinetis_k22_firmware_runner` | Executable | CLI tool to load and run firmware images. |
| `test` | Utility | Run all tests. |

## Run Firmware

```
kinetis_k22_firmware_runner <IMAGE> --reset-address <ADDRESS> [OPTIONS]
```

### Runner Options

| Option | Description |
| :--- | :--- |
| `--reset-address <ADDR>` | Entry point / reset address (required). |
| `--profile <DEVICE>` | K22 device profile. Defaults to `MK22FN51212`. |
| `--package <CODE>` | Package code such as `LH`, `LL`, or `LQ`. |
| `--binary-address <ADDR>` | Base address for raw binary images. |
| `--stop-address <ADDR>` | Execution stop address. |
| `--max-instructions <N>` | Maximum instruction count. |
| `--max-cycles <N>` | Maximum clock cycle limit. |

## Use in Meson Projects

```meson
kinetis_k22 = subproject('kinetis-k22-sim')
simulator = kinetis_k22.get_variable('kinetis_k22_simulator_dependency')

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
meson test -C build/test-coverage
meson compile -C build/test-coverage coverage-text
```

The coverage report requires GCC, gcov, and gcovr.
