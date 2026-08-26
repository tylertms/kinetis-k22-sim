project(
  'kinetis-sim',
  'c',
  default_options: [
    'buildtype=release',
    'c_std=c11',
    'warning_level=3',
    'werror=true',
  ],
  meson_version: '>=1.11.0',
)

cc = meson.get_compiler('c')
if cc.get_id() != 'gcc'
  error('GCC is required')
endif
public_include = include_directories('include')
private_include = include_directories('src')
build_by_default = not meson.is_subproject()

simulator_sources = files(
  'src/architecture/cortex_m4/bus.c',
  'src/architecture/cortex_m4/constraints.c',
  'src/architecture/cortex_m4/core.c',
  'src/architecture/cortex_m4/debug.c',
  'src/architecture/cortex_m4/exceptions/advanced.c',
  'src/architecture/cortex_m4/exceptions/base.c',
  'src/architecture/cortex_m4/instructions/dsp.c',
  'src/architecture/cortex_m4/instructions/fpu.c',
  'src/architecture/cortex_m4/instructions/it.c',
  'src/architecture/cortex_m4/instructions/misc.c',
  'src/architecture/cortex_m4/instructions/thumb/extended.c',
  'src/architecture/cortex_m4/instructions/thumb/thumb16.c',
  'src/architecture/cortex_m4/instructions/thumb/thumb32.c',
  'src/architecture/cortex_m4/mpu.c',
  'src/architecture/cortex_m4/system.c',
  'src/architecture/cortex_m4/timing.c',
  'src/device/kinetis/communication/sdhc.c',
  'src/device/kinetis/communication/serial/registers.c',
  'src/device/kinetis/communication/serial/runtime.c',
  'src/device/kinetis/communication/usb_dcd.c',
  'src/device/kinetis/device.c',
  'src/device/kinetis/gpio/api.c',
  'src/device/kinetis/gpio/core.c',
  'src/device/kinetis/gpio/peripherals.c',
  'src/device/kinetis/host.c',
  'src/device/kinetis/memory/analog.c',
  'src/device/kinetis/memory/api.c',
  'src/device/kinetis/memory/dma.c',
  'src/device/kinetis/memory/flash.c',
  'src/device/kinetis/memory/integrity.c',
  'src/device/kinetis/memory/runtime.c',
  'src/device/kinetis/platform/peripherals.c',
  'src/device/kinetis/platform/registers.c',
  'src/device/kinetis/timing/api.c',
  'src/device/kinetis/timing/clocks.c',
  'src/device/kinetis/timing/counters.c',
  'src/device/kinetis/timing/ftm.c',
  'src/device/kinetis/timing/registers.c',
  'src/device/kinetis/timing/watchdogs.c',
  'src/device/kinetis/variants/manifest.c',
  'src/device/kinetis/variants/package.c',
  'src/device/kinetis/variants/profile.c',
  'src/device/kinetis/variants/registers/mk22f12.c',
  'src/device/kinetis/variants/registers/mk22fn12810.c',
  'src/device/kinetis/variants/registers/mk22f25612.c',
  'src/device/kinetis/variants/registers/mk22f51212.c',
  'src/device/kinetis/variants/registers/mkv30f12810.c',
)

simulator = static_library(
  'kinetis_simulator',
  simulator_sources,
  build_by_default: build_by_default,
  c_args: ['-fno-strict-aliasing'],
  include_directories: [public_include, private_include],
)
kinetis_simulator_dependency = declare_dependency(
  include_directories: public_include,
  link_with: simulator,
)

firmware_image = static_library(
  'kinetis_firmware_image',
  'src/image/cortex_m4_firmware_image.c',
  build_by_default: build_by_default,
  dependencies: kinetis_simulator_dependency,
  include_directories: public_include,
)
kinetis_firmware_image_dependency = declare_dependency(
  dependencies: kinetis_simulator_dependency,
  include_directories: public_include,
  link_with: firmware_image,
)

kinetis_firmware_runner = executable(
  'kinetis_firmware_runner',
  'src/runner/main.c',
  build_by_default: build_by_default,
  dependencies: kinetis_firmware_image_dependency,
)

if not meson.is_subproject()
  subdir('tests')
endif
