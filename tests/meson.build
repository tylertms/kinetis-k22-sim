test_include = include_directories('.', 'support', '../src')
test_definitions = [
  {
    'name': 'core_reset',
    'sources': ['architecture/cortex_m4/system/reset_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_api',
    'sources': ['architecture/cortex_m4/api_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_thumb',
    'sources': ['architecture/cortex_m4/execution/thumb_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_thumb16_arithmetic',
    'sources': [
      'architecture/cortex_m4/execution/thumb16_arithmetic_test.c',
    ],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_thumb16_memory',
    'sources': ['architecture/cortex_m4/execution/thumb16_memory_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_thumb16_complete',
    'sources': ['architecture/cortex_m4/execution/thumb16_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_thumb32',
    'sources': ['architecture/cortex_m4/execution/thumb32_smoke_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_thumb32_extended',
    'sources': ['architecture/cortex_m4/execution/thumb32_extended_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_thumb32_complete',
    'sources': ['architecture/cortex_m4/execution/thumb32_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_thumb32_memory_complete',
    'sources': ['architecture/cortex_m4/execution/thumb32_memory_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_memory_faults',
    'sources': ['architecture/cortex_m4/execution/memory_fault_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_power',
    'sources': ['architecture/cortex_m4/system/power_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_processor_state',
    'sources': ['architecture/cortex_m4/system/state_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_constraints_complete',
    'sources': ['architecture/cortex_m4/system/constraints_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_debug_complete',
    'sources': ['architecture/cortex_m4/system/debug_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_debug_boundary',
    'sources': ['architecture/cortex_m4/system/debug_boundary_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_debug_census',
    'sources': ['architecture/cortex_m4/system/debug_census_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_dsp',
    'sources': ['architecture/cortex_m4/execution/dsp_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_fpu',
    'sources': ['architecture/cortex_m4/floating_point/fpu_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_fpu_complete',
    'sources': ['architecture/cortex_m4/floating_point/complete_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_fpu_edge_complete',
    'sources': ['architecture/cortex_m4/floating_point/edge_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_fpu_census',
    'sources': ['architecture/cortex_m4/floating_point/census_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_mpu_complete',
    'sources': ['architecture/cortex_m4/system/mpu_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_bus_protection',
    'sources': ['architecture/cortex_m4/system/bus_protection_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_fpu_encoding_boundaries',
    'sources': [
      'architecture/cortex_m4/floating_point/encoding_boundary_test.c',
    ],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_instruction_census',
    'sources': [
      'architecture/cortex_m4/execution/instruction_census_test.c',
    ],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_encoding_census',
    'sources': ['architecture/cortex_m4/execution/encoding_census_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_it_complete',
    'sources': ['architecture/cortex_m4/execution/it_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_exception',
    'sources': ['architecture/cortex_m4/exceptions/exception_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_exception_advanced',
    'sources': ['architecture/cortex_m4/exceptions/advanced_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_exception_integration_complete',
    'sources': ['architecture/cortex_m4/exceptions/integration_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_fault',
    'sources': ['architecture/cortex_m4/exceptions/fault_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_system_exception',
    'sources': ['architecture/cortex_m4/exceptions/system_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_system_register',
    'sources': ['architecture/cortex_m4/system/register_access_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_system_complete',
    'sources': ['architecture/cortex_m4/system/registers_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'core_system_boundary',
    'sources': ['architecture/cortex_m4/system/boundary_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_system_access_boundary',
    'sources': ['architecture/cortex_m4/system/access_boundary_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_system_exception_census',
    'sources': ['architecture/cortex_m4/system/exception_census_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_timing_complete',
    'sources': ['architecture/cortex_m4/system/timing_test.c'],
    'suite': ['core', 'unit', 'integration'],
  },
  {
    'name': 'core_timing_census',
    'sources': ['architecture/cortex_m4/system/timing_census_test.c'],
    'suite': ['core', 'unit'],
  },
  {
    'name': 'core_trace',
    'sources': ['architecture/cortex_m4/system/trace_test.c'],
    'suite': ['core', 'integration'],
  },
  {
    'name': 'device_memory',
    'sources': ['device/kinetis/memory/memory_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_profile',
    'sources': ['device/kinetis/variants/profile_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_boundary_census',
    'sources': ['device/kinetis/platform/device_boundary_test.c'],
    'suite': ['device', 'unit'],
    'allocation_failure': true,
  },
  {
    'name': 'device_register_manifest',
    'sources': ['device/kinetis/variants/manifest_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_register_access',
    'sources': ['device/kinetis/variants/register_access_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_mkv10_complete',
    'sources': ['device/kinetis/variants/mkv10_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_peripheral_census',
    'sources': ['device/kinetis/variants/peripheral_census_test.c'],
    'suite': ['device', 'unit', 'integration'],
  },
  {
    'name': 'device_stateful_census',
    'sources': ['device/kinetis/platform/stateful_census_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_peripheral_state_census',
    'sources': [
      'device/kinetis/platform/peripheral_state_census_test.c',
    ],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_data_complete',
    'sources': [
      'device/kinetis/memory/data/core.c',
      'device/kinetis/memory/data/flash.c',
      'device/kinetis/memory/data/swap.c',
      'device/kinetis/memory/data/boundary.c',
    ],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_data_census',
    'sources': [
      'device/kinetis/memory/data/census_test.c',
      'device/kinetis/memory/data/core.c',
      'device/kinetis/memory/data/flash.c',
      'device/kinetis/memory/data/command_census.c',
      'device/kinetis/memory/data/flash_state_census.c',
      'device/kinetis/memory/data/state_census.c',
    ],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_fmc_complete',
    'sources': ['device/kinetis/memory/fmc_test.c'],
    'suite': ['device', 'unit', 'integration'],
  },
  {
    'name': 'device_dma_faults',
    'sources': ['device/kinetis/memory/dma_fault_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_flash_faults',
    'sources': ['device/kinetis/memory/flash_fault_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_io_complete',
    'sources': [
      'device/kinetis/platform/io_test.c',
      'device/kinetis/platform/io_boundaries.c',
    ],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_io_state_census',
    'sources': ['device/kinetis/platform/io_state_census_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_package',
    'sources': ['device/kinetis/variants/package_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_serial_complete',
    'sources': ['device/kinetis/communication/serial_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_sdhc_complete',
    'sources': ['device/kinetis/communication/sdhc_test.c'],
    'suite': ['device', 'integration'],
    'allocation_failure': true,
  },
  {
    'name': 'device_sdhc_state_census',
    'sources': ['device/kinetis/communication/sdhc_state_census_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_timing_complete',
    'sources': [
      'device/kinetis/timing/core.c',
      'device/kinetis/timing/ftm.c',
      'device/kinetis/timing/surface.c',
      'device/kinetis/timing/boundaries.c',
    ],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_timing_census',
    'sources': [
      'device/kinetis/timing/census_test.c',
      'device/kinetis/timing/core.c',
      'device/kinetis/timing/ftm_census.c',
      'device/kinetis/timing/state_census.c',
    ],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_watchdog_complete',
    'sources': ['device/kinetis/timing/watchdog_api_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_watchdog_state_census',
    'sources': ['device/kinetis/timing/watchdog_state_census_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_timing_register_state_census',
    'sources': ['device/kinetis/timing/register_state_census_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_ftm_state_census',
    'sources': ['device/kinetis/timing/ftm_state_census_test.c'],
    'suite': ['device', 'unit'],
  },
  {
    'name': 'device_cmt_complete',
    'sources': ['device/kinetis/timing/cmt_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_usbdcd_complete',
    'sources': ['device/kinetis/communication/usb_dcd_test.c'],
    'suite': ['device', 'unit', 'integration'],
  },
  {
    'name': 'device_ftm_synchronization',
    'sources': ['device/kinetis/control/ftm_synchronization_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_ftm_combine',
    'sources': ['device/kinetis/control/ftm_combine_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_ftm_deadtime',
    'sources': ['device/kinetis/control/ftm_deadtime_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_ftm_fault',
    'sources': ['device/kinetis/control/ftm_fault_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_ftm_quadrature',
    'sources': ['device/kinetis/control/ftm_quadrature_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_integration_complete',
    'sources': [
      'device/kinetis/platform/integration/platform.c',
      'device/kinetis/platform/integration/peripherals.c',
    ],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_host_api_complete',
    'sources': ['device/kinetis/platform/host_api_test.c'],
    'suite': ['device', 'integration'],
    'allocation_failure': true,
  },
  {
    'name': 'device_clock_gate_complete',
    'sources': ['device/kinetis/platform/clock_gate_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_sysmpu_integration',
    'sources': ['device/kinetis/platform/sysmpu_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_infrastructure',
    'sources': ['device/kinetis/platform/infrastructure_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_low_power_retention',
    'sources': ['device/kinetis/platform/low_power_retention_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_gpio',
    'sources': ['device/kinetis/platform/gpio_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_pit',
    'sources': ['device/kinetis/timing/pit_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_adc',
    'sources': ['device/kinetis/analog/adc_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_uart',
    'sources': ['device/kinetis/communication/uart_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_spi',
    'sources': ['device/kinetis/communication/spi_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_dma',
    'sources': ['device/kinetis/memory/dma_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_i2c',
    'sources': ['device/kinetis/communication/i2c_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_watchdog',
    'sources': ['device/kinetis/timing/watchdog_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'device_clock_power',
    'sources': ['device/kinetis/timing/clock_power_test.c'],
    'suite': ['device', 'integration'],
  },
  {
    'name': 'firmware_image',
    'sources': ['system/firmware_image_test.c'],
    'suite': ['runner', 'system'],
    'dependencies': [kinetis_firmware_image_dependency],
  },
]

allocation_failure_tests = cc.has_link_argument('-Wl,--wrap=realloc')

foreach definition : test_definitions
  if definition.get('allocation_failure', false) and not allocation_failure_tests
    continue
  endif
  name = 'kinetis_' + definition['name'] + '_test'
  sources = definition['sources']
  suites = definition['suite']
  if definition['name'].contains('census')
    suites += ['census']
  endif
  c_args = []
  link_args = []
  if definition.get('allocation_failure', false)
    sources += ['support/allocation_failure.c']
    c_args += ['-DKINETIS_TEST_ALLOCATION_FAILURE=1']
    link_args += [
      '-Wl,--wrap=malloc',
      '-Wl,--wrap=calloc',
      '-Wl,--wrap=realloc',
    ]
  endif
  if host_machine.system() == 'windows' and definition['name'] == 'device_data_complete'
    link_args += cc.get_supported_link_arguments(
      [
        '-Wl,--stack,33554432',
        '/STACK:33554432',
      ],
    )
  endif
  target = executable(
    name,
    sources,
    c_args: c_args,
    dependencies: definition.get(
      'dependencies',
      [kinetis_simulator_dependency],
    ),
    include_directories: test_include,
    link_args: link_args,
  )
  test(name, target, suite: suites)
  if definition['name'] == 'core_encoding_census'
    foreach shard : range(1, 24)
      test(
        name + '_shard_' + shard.to_string(),
        target,
        args: shard.to_string(),
        suite: suites,
      )
    endforeach
  endif
endforeach

smoke_image_name = 'kinetis_firmware_runner_smoke.bin'
fault_image_name = 'kinetis_firmware_runner_fault.bin'
clock_image_name = 'kinetis_firmware_runner_clock.bin'
smoke_image = meson.current_build_dir() / smoke_image_name
fault_image = meson.current_build_dir() / fault_image_name
clock_image = meson.current_build_dir() / clock_image_name
fixture_generator = executable(
  'kinetis_firmware_runner_fixture',
  'system/runner_fixture_test.c',
  c_args: [
    '-DKINETIS_RUNNER_SMOKE_IMAGE="' + smoke_image + '"',
    '-DKINETIS_RUNNER_FAULT_IMAGE="' + fault_image + '"',
    '-DKINETIS_RUNNER_CLOCK_IMAGE="' + clock_image + '"',
  ],
  include_directories: [test_include, public_include],
)
runner_images = custom_target(
  'kinetis_firmware_runner_images',
  build_by_default: true,
  command: fixture_generator,
  output: [smoke_image_name, fault_image_name, clock_image_name],
)

test(
  'kinetis_firmware_runner_smoke_test',
  kinetis_firmware_runner,
  args: [
    runner_images[0],
    '--profile',
    'MKV30F12810',
    '--reset-address',
    '0',
    '--binary-address',
    '0',
    '--stop-address',
    '10',
    '--max-instructions',
    '10',
  ],
  suite: ['runner', 'system'],
)
test(
  'kinetis_firmware_runner_fault_test',
  kinetis_firmware_runner,
  args: [
    runner_images[1],
    '--profile',
    'MKV30F12810',
    '--reset-address',
    '0',
    '--binary-address',
    '0',
    '--max-instructions',
    '10',
  ],
  expected_fail: true,
  suite: ['runner', 'system'],
)
test(
  'kinetis_firmware_runner_clock_test',
  kinetis_firmware_runner,
  args: [
    runner_images[2],
    '--profile',
    'MK22FN51212',
    '--reset-address',
    '0',
    '--binary-address',
    '0',
    '--max-instructions',
    '20',
  ],
  expected_fail: true,
  suite: ['runner', 'system'],
)

missing_image = meson.current_build_dir() / 'missing.bin'
runner_failure_cases = [
  {'name': 'usage', 'args': []},
  {'name': 'missing_reset_value', 'args': [missing_image, '--reset-address']},
  {
    'name': 'missing_reset_address',
    'args': [missing_image, '--profile', 'MKV30F12810', '--max-instructions', '1'],
  },
  {'name': 'missing_profile', 'args': [missing_image, '--reset-address', '1']},
  {
    'name': 'invalid_profile',
    'args': [missing_image, '--reset-address', '1', '--profile', 'invalid'],
  },
  {
    'name': 'invalid_package',
    'args': [missing_image, '--reset-address', '1', '--package', 'invalid'],
  },
  {
    'name': 'invalid_value',
    'args': [missing_image, '--reset-address', 'invalid'],
  },
  {
    'name': 'unknown_option',
    'args': [missing_image, '--reset-address', '1', '--invalid', '1'],
  },
  {
    'name': 'large_address',
    'args': [
      missing_image,
      '--profile',
      'MKV30F12810',
      '--reset-address',
      '4294967296',
    ],
  },
  {
    'name': 'invalid_device',
    'args': [
      missing_image,
      '--reset-address',
      '1',
      '--profile',
      'MK22FN51212',
      '--package',
      'AK',
    ],
  },
  {
    'name': 'missing_image',
    'args': [
      missing_image,
      '--reset-address',
      '1',
      '--profile',
      'MK22FN51212',
      '--package',
      'DC',
      '--binary-address',
      '0',
      '--max-cycles',
      '1',
      '--stop-address',
      '2',
    ],
  },
]
foreach failure : runner_failure_cases
  test(
    'kinetis_firmware_runner_' + failure['name'] + '_test',
    kinetis_firmware_runner,
    args: failure['args'],
    expected_fail: true,
    suite: ['runner', 'system'],
  )
endforeach
