project(
  'kinetis-k22-sim',
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
  'src/device/kinetis_k22/communication/sdhc.c',
  'src/device/kinetis_k22/communication/serial/registers.c',
  'src/device/kinetis_k22/communication/serial/runtime.c',
  'src/device/kinetis_k22/communication/usb_dcd.c',
  'src/device/kinetis_k22/device.c',
  'src/device/kinetis_k22/gpio/api.c',
  'src/device/kinetis_k22/gpio/core.c',
  'src/device/kinetis_k22/gpio/peripherals.c',
  'src/device/kinetis_k22/host.c',
  'src/device/kinetis_k22/memory/analog.c',
  'src/device/kinetis_k22/memory/api.c',
  'src/device/kinetis_k22/memory/dma.c',
  'src/device/kinetis_k22/memory/flash.c',
  'src/device/kinetis_k22/memory/integrity.c',
  'src/device/kinetis_k22/memory/runtime.c',
  'src/device/kinetis_k22/platform/peripherals.c',
  'src/device/kinetis_k22/platform/registers.c',
  'src/device/kinetis_k22/timing/api.c',
  'src/device/kinetis_k22/timing/clocks.c',
  'src/device/kinetis_k22/timing/counters.c',
  'src/device/kinetis_k22/timing/ftm.c',
  'src/device/kinetis_k22/timing/registers.c',
  'src/device/kinetis_k22/timing/watchdogs.c',
  'src/device/kinetis_k22/variants/manifest.c',
  'src/device/kinetis_k22/variants/package.c',
  'src/device/kinetis_k22/variants/profile.c',
  'src/device/kinetis_k22/variants/registers/mk22f12.c',
  'src/device/kinetis_k22/variants/registers/mk22f12810.c',
  'src/device/kinetis_k22/variants/registers/mk22f25612.c',
  'src/device/kinetis_k22/variants/registers/mk22f51212.c',
)

simulator = static_library(
  'kinetis_k22_simulator',
  simulator_sources,
  build_by_default: build_by_default,
  c_args: cc.get_id() == 'gcc' ? ['-fno-strict-aliasing'] : [],
  include_directories: [public_include, private_include],
)
kinetis_k22_simulator_dependency = declare_dependency(
  include_directories: public_include,
  link_with: simulator,
)

firmware_image = static_library(
  'kinetis_k22_firmware_image',
  'src/image/cortex_m4_firmware_image.c',
  build_by_default: build_by_default,
  dependencies: kinetis_k22_simulator_dependency,
  include_directories: public_include,
)
kinetis_k22_firmware_image_dependency = declare_dependency(
  dependencies: kinetis_k22_simulator_dependency,
  include_directories: public_include,
  link_with: firmware_image,
)

kinetis_k22_firmware_runner = executable(
  'kinetis_k22_firmware_runner',
  'src/runner/main.c',
  build_by_default: build_by_default,
  dependencies: kinetis_k22_firmware_image_dependency,
)

if not meson.is_subproject()
  subdir('tests')
endif
