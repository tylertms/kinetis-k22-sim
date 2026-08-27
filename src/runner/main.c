#include "cortex_m4.h"
#include "kinetis.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cortex_m4_firmware_image.h"

static bool parse_uint64(const char* text, uint64_t* parsed_value) {
    char* end = NULL;
    errno = 0;
    const unsigned long long parsed = strtoull(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0') {
        return false;
    }
    *parsed_value = (uint64_t)parsed;
    return true;
}

static void print_usage(const char* program) {
    fprintf(stderr,
            "usage: %s IMAGE --profile DEVICE --reset-address ADDRESS "
            "[--package CODE] [--binary-address ADDRESS] "
            "[--max-instructions COUNT] "
            "[--max-cycles COUNT] [--stop-address ADDRESS] [--coverage]\n",
            program);
}

int main(int argc, char** argv) {
    if (argc < 4) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    uint64_t reset_address = 0;
    bool reset_address_set = false;
    uint64_t binary_address = 0;
    bool binary_image_loaded = false;
    uint64_t stop_address = 0;
    bool stop_address_set = false;
    bool coverage_requested = false;
    KinetisProfile profile = KINETIS_PROFILE_COUNT;
    bool profile_set = false;
    KinetisPackage package = KINETIS_PACKAGE_DEFAULT;
    CortexM4RunLimits limits = {1000000, 10000000};
    for (int argument_index = 2; argument_index < argc;) {
        if (strcmp(argv[argument_index], "--coverage") == 0) {
            coverage_requested = true;
            argument_index++;
            continue;
        }
        if (argument_index + 1 >= argc) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        if (strcmp(argv[argument_index], "--profile") == 0) {
            if (!kinetis_profile_from_name(argv[argument_index + 1], &profile)) {
                fprintf(stderr, "unknown Kinetis profile: %s\n", argv[argument_index + 1]);
                return EXIT_FAILURE;
            }
            profile_set = true;
            argument_index += 2;
            continue;
        }

        if (strcmp(argv[argument_index], "--package") == 0) {
            if (!kinetis_package_from_code(argv[argument_index + 1], &package)) {
                fprintf(stderr, "unknown Kinetis package: %s\n", argv[argument_index + 1]);
                return EXIT_FAILURE;
            }
            argument_index += 2;
            continue;
        }

        uint64_t parsed_value = 0;
        if (!parse_uint64(argv[argument_index + 1], &parsed_value)) {
            fprintf(stderr, "invalid value: %s\n", argv[argument_index + 1]);
            return EXIT_FAILURE;
        }
        if (strcmp(argv[argument_index], "--reset-address") == 0) {
            reset_address = parsed_value;
            reset_address_set = true;
        } else if (strcmp(argv[argument_index], "--binary-address") == 0) {
            binary_address = parsed_value;
            binary_image_loaded = true;
        } else if (strcmp(argv[argument_index], "--max-instructions") == 0) {
            limits.instruction_limit = parsed_value;
        } else if (strcmp(argv[argument_index], "--max-cycles") == 0) {
            limits.cycle_limit = parsed_value;
        } else if (strcmp(argv[argument_index], "--stop-address") == 0) {
            stop_address = parsed_value;
            stop_address_set = true;
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[argument_index]);
            return EXIT_FAILURE;
        }
        argument_index += 2;
    }
    if (!profile_set || !reset_address_set) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    if (reset_address > UINT32_MAX || binary_address > UINT32_MAX || stop_address > UINT32_MAX) {
        fprintf(stderr, "an address is too large\n");
        return EXIT_FAILURE;
    }
    KinetisConfiguration configuration = kinetis_configuration(profile);
    configuration.package = package;
    configuration.vector_table_address = (uint32_t)reset_address;
    Kinetis* device = kinetis_create(configuration);
    if (device == NULL) {
        fprintf(stderr, "failed to create the device\n");
        return EXIT_FAILURE;
    }
    uint32_t entry_address = 0;
    const bool loaded = binary_image_loaded
                            ? cortex_m4_load_binary(device, argv[1], (uint32_t)binary_address)
                            : cortex_m4_load_elf(device, argv[1], &entry_address);
    if (!loaded) {
        fprintf(stderr, "failed to load the firmware image\n");
        kinetis_destroy(device);
        return EXIT_FAILURE;
    }
    if (!kinetis_reset(device)) {
        fprintf(stderr, "failed to reset the device\n");
        kinetis_destroy(device);
        return EXIT_FAILURE;
    }
    CortexM4Coverage* coverage = NULL;
    if (coverage_requested) {
        coverage = binary_image_loaded ? NULL : cortex_m4_coverage_create_elf(argv[1]);
        if (coverage == NULL) {
            fprintf(stderr, "coverage requires an ELF image with executable sections\n");
            kinetis_destroy(device);
            return EXIT_FAILURE;
        }
        cortex_m4_set_coverage(kinetis_cpu(device), coverage);
    }
    if (stop_address_set &&
        !cortex_m4_set_breakpoint(kinetis_cpu(device), 0, (uint32_t)stop_address, true)) {
        fprintf(stderr, "the stop address is invalid\n");
        kinetis_destroy(device);
        return EXIT_FAILURE;
    }
    const CortexM4Result result = cortex_m4_run(kinetis_cpu(device), limits);
    const uint32_t fault_status = cortex_m4_get_fault_status(kinetis_cpu(device));
    printf("stop=%u pc=0x%08" PRIx32 " opcode=0x%08" PRIx32 " instructions=%" PRIu64
           " cycles=%" PRIu64 " entry=0x%08" PRIx32 " cfsr=0x%08" PRIx32 " bfar=0x%08" PRIx32 "\n",
           result.stop, result.pc, result.opcode, result.instructions, result.cycles, entry_address,
           fault_status, cortex_m4_get_fault_address(kinetis_cpu(device)));
    for (uint8_t register_index = 0; register_index < 16; register_index++) {
        printf("r%u=0x%08" PRIx32 "%c", register_index,
               cortex_m4_get_register(kinetis_cpu(device), register_index),
               register_index == 15 ? '\n' : ' ');
    }
    if (coverage != NULL) {
        const CortexM4CoverageResult coverage_result = cortex_m4_coverage_result(coverage);
        printf("coverage instructions=%zu/%zu %.2f%% branches=%zu/%zu %.2f%%\n",
               coverage_result.covered_instructions, coverage_result.total_instructions,
               coverage_result.instruction_coverage_percent, coverage_result.covered_branch_sites,
               coverage_result.total_branch_sites, coverage_result.branch_coverage_percent);
        cortex_m4_set_coverage(kinetis_cpu(device), NULL);
        cortex_m4_coverage_destroy(coverage);
    }
    kinetis_destroy(device);
    const bool failed =
        fault_status != 0 || result.stop == CORTEX_M4_STOP_CLOCK ||
        result.stop == CORTEX_M4_STOP_UNSUPPORTED || result.stop == CORTEX_M4_STOP_BUS_FAULT ||
        result.stop == CORTEX_M4_STOP_USAGE_FAULT || result.stop == CORTEX_M4_STOP_LOCKUP;
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
