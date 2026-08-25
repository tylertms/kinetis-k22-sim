cmake_minimum_required(VERSION 3.21)

project(kinetis_k22_sim LANGUAGES C)

if(PROJECT_IS_TOP_LEVEL)
  option(BUILD_TESTING "Build the simulator tests" ON)
  option(KINETIS_K22_ENABLE_COVERAGE "Enable simulator source coverage" OFF)
  set(CMAKE_CTEST_ARGUMENTS --output-on-failure --parallel)
  set(CMAKE_SKIP_TEST_ALL_DEPENDENCY FALSE)
endif()

if(KINETIS_K22_ENABLE_COVERAGE)
  if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "Simulator source coverage requires GCC")
  endif()
  add_compile_options(--coverage)
  add_link_options(--coverage)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Build type" FORCE)
endif()

if(PROJECT_IS_TOP_LEVEL AND NOT KINETIS_K22_ENABLE_COVERAGE)
  find_program(kinetis_k22_coverage_compiler gcc)
  find_program(kinetis_k22_gcov gcov)
  set(kinetis_k22_coverage_build_directory
      "${CMAKE_CURRENT_SOURCE_DIR}/build/test-coverage")

  if(kinetis_k22_coverage_compiler AND kinetis_k22_gcov)
    add_custom_target(
      test-coverage
      COMMAND "${CMAKE_COMMAND}" -E rm -rf
              "${kinetis_k22_coverage_build_directory}"
      COMMAND
        "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_SOURCE_DIR}" -B
        "${kinetis_k22_coverage_build_directory}" -G Ninja
        -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
        -DKINETIS_K22_ENABLE_COVERAGE=ON
        "-DCMAKE_C_COMPILER=${kinetis_k22_coverage_compiler}"
      COMMAND "${CMAKE_COMMAND}" --build
              "${kinetis_k22_coverage_build_directory}" --parallel
      COMMAND
        "${CMAKE_CTEST_COMMAND}" --test-dir
        "${kinetis_k22_coverage_build_directory}" --output-on-failure --parallel 1
      COMMAND
        "${CMAKE_COMMAND}"
        "-DCOVERAGE_BUILD_DIRECTORY=${kinetis_k22_coverage_build_directory}"
        "-DCOVERAGE_SOURCE_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DCOVERAGE_TARGET_PREFIX=kinetis_k22"
        "-DGCOV_EXECUTABLE=${kinetis_k22_gcov}" -P
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Coverage.cmake"
      COMMENT "Testing the Kinetis K22 simulator with source coverage"
      USES_TERMINAL VERBATIM)
  else()
    add_custom_target(
      test-coverage
      COMMAND "${CMAKE_COMMAND}" -E echo
              "Simulator source coverage requires GCC and gcov"
      COMMAND "${CMAKE_COMMAND}" -E false
      USES_TERMINAL VERBATIM)
  endif()
endif()

set(kinetis_k22_simulator_sources
    src/architecture/cortex_m4/core.c
    src/architecture/cortex_m4/bus.c
    src/architecture/cortex_m4/constraints.c
    src/architecture/cortex_m4/debug.c
    src/architecture/cortex_m4/instructions/dsp.c
    src/architecture/cortex_m4/exceptions/base.c
    src/architecture/cortex_m4/exceptions/advanced.c
    src/architecture/cortex_m4/instructions/fpu.c
    src/architecture/cortex_m4/instructions/it.c
    src/architecture/cortex_m4/mpu.c
    src/architecture/cortex_m4/instructions/misc.c
    src/architecture/cortex_m4/system.c
    src/architecture/cortex_m4/instructions/thumb/thumb16.c
    src/architecture/cortex_m4/instructions/thumb/thumb32.c
    src/architecture/cortex_m4/instructions/thumb/extended.c
    src/architecture/cortex_m4/timing.c
    src/device/kinetis_k22/memory/api.c
    src/device/kinetis_k22/memory/analog.c
    src/device/kinetis_k22/memory/dma.c
    src/device/kinetis_k22/memory/flash.c
    src/device/kinetis_k22/memory/integrity.c
    src/device/kinetis_k22/memory/runtime.c
    src/device/kinetis_k22/gpio/api.c
    src/device/kinetis_k22/gpio/core.c
    src/device/kinetis_k22/gpio/peripherals.c
    src/device/kinetis_k22/variants/package.c
    src/device/kinetis_k22/variants/profile.c
    src/device/kinetis_k22/variants/manifest.c
    src/device/kinetis_k22/variants/registers/mk22f12.c
    src/device/kinetis_k22/variants/registers/mk22f12810.c
    src/device/kinetis_k22/variants/registers/mk22f25612.c
    src/device/kinetis_k22/variants/registers/mk22f51212.c
    src/device/kinetis_k22/communication/sdhc.c
    src/device/kinetis_k22/communication/serial/registers.c
    src/device/kinetis_k22/communication/serial/runtime.c
    src/device/kinetis_k22/timing/api.c
    src/device/kinetis_k22/timing/clocks.c
    src/device/kinetis_k22/timing/counters.c
    src/device/kinetis_k22/timing/ftm.c
    src/device/kinetis_k22/timing/registers.c
    src/device/kinetis_k22/timing/watchdogs.c
    src/device/kinetis_k22/communication/usb_dcd.c
    src/device/kinetis_k22/device.c
    src/device/kinetis_k22/host.c
    src/device/kinetis_k22/platform/peripherals.c
    src/device/kinetis_k22/platform/registers.c)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  set_source_files_properties(
    src/device/kinetis_k22/platform/peripherals.c
    src/device/kinetis_k22/platform/registers.c
    PROPERTIES COMPILE_OPTIONS -fno-strict-aliasing)
endif()

add_library(kinetis_k22_simulator STATIC ${kinetis_k22_simulator_sources})
add_library(kinetis_k22::simulator ALIAS kinetis_k22_simulator)

add_library(kinetis_k22_firmware_image STATIC
            src/image/cortex_m4_firmware_image.c)
add_library(kinetis_k22::firmware_image ALIAS kinetis_k22_firmware_image)

add_executable(kinetis_k22_firmware_runner src/runner/main.c)
add_executable(kinetis_k22::firmware_runner ALIAS kinetis_k22_firmware_runner)

target_compile_features(kinetis_k22_simulator PUBLIC c_std_11)
target_include_directories(
  kinetis_k22_simulator
  PUBLIC include
  PRIVATE src)
target_compile_features(kinetis_k22_firmware_runner PRIVATE c_std_11)
target_compile_features(kinetis_k22_firmware_image PUBLIC c_std_11)
target_include_directories(kinetis_k22_firmware_image PUBLIC include)
target_link_libraries(kinetis_k22_firmware_image PUBLIC kinetis_k22_simulator)
target_link_libraries(kinetis_k22_firmware_runner
                      PRIVATE kinetis_k22_firmware_image)

function(kinetis_k22_enable_strict_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()

set(kinetis_k22_targets kinetis_k22_simulator kinetis_k22_firmware_image
                        kinetis_k22_firmware_runner)
foreach(simulator_target IN LISTS kinetis_k22_targets)
  kinetis_k22_enable_strict_warnings(${simulator_target})
endforeach()

include(tests/CMakeLists.txt)
