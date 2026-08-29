#include "cortex_m4.h"
#include "kinetis.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cortex_m4_firmware_image.h"

typedef struct {
    uint32_t address;
    uint32_t value;
} InitialWrite;

typedef struct {
    CortexM4* cpu;
    uint32_t addresses[16];
    uint32_t opcodes[16];
    size_t count;
    bool frozen;
} ExecutionTrace;

static void record_trace(void* context, uint32_t address, uint32_t opcode, bool executed) {
    ExecutionTrace* trace = context;
    if (trace->frozen || !executed)
        return;
    if ((cortex_m4_get_xpsr(trace->cpu) & 0x1ffu) != 0u) {
        trace->frozen = true;
        return;
    }
    const size_t index = trace->count % (sizeof(trace->addresses) / sizeof(trace->addresses[0]));
    trace->addresses[index] = address;
    trace->opcodes[index] = opcode;
    trace->count++;
}

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

static bool parse_initial_write(const char* text, InitialWrite* write) {
    char* separator = NULL;
    errno = 0;
    const unsigned long address = strtoul(text, &separator, 0);
    if (errno != 0 || separator == text || *separator != ':')
        return false;
    char* end = NULL;
    errno = 0;
    const unsigned long value = strtoul(separator + 1, &end, 0);
    if (errno != 0 || end == separator + 1 || *end != '\0' || address > UINT32_MAX ||
        value > UINT32_MAX)
        return false;
    write->address = (uint32_t)address;
    write->value = (uint32_t)value;
    return true;
}

static void print_usage(const char* program) {
    fprintf(stderr,
            "usage: %s IMAGE --profile DEVICE --reset-address ADDRESS "
            "[--package CODE] [--binary-address ADDRESS] "
            "[--max-instructions COUNT] "
            "[--max-cycles COUNT] [--stop-address ADDRESS] "
            "[--write32 ADDRESS:VALUE] [--coverage] "
            "[--minimum-instructions-covered COUNT] [--minimum-branches-covered COUNT]\n",
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
    uint64_t minimum_instructions_covered = 0u;
    uint64_t minimum_branches_covered = 0u;
    InitialWrite initial_writes[32];
    size_t initial_write_count = 0u;
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

        if (strcmp(argv[argument_index], "--write32") == 0) {
            if (initial_write_count >= sizeof(initial_writes) / sizeof(initial_writes[0]) ||
                !parse_initial_write(argv[argument_index + 1],
                                     &initial_writes[initial_write_count])) {
                fprintf(stderr, "invalid initial write: %s\n", argv[argument_index + 1]);
                return EXIT_FAILURE;
            }
            initial_write_count++;
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
        } else if (strcmp(argv[argument_index], "--minimum-instructions-covered") == 0) {
            minimum_instructions_covered = parsed_value;
            coverage_requested = true;
        } else if (strcmp(argv[argument_index], "--minimum-branches-covered") == 0) {
            minimum_branches_covered = parsed_value;
            coverage_requested = true;
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
    for (size_t write_index = 0u; write_index < initial_write_count; write_index++) {
        const InitialWrite* initial_write = &initial_writes[write_index];
        if (!kinetis_write(device, initial_write->address, &initial_write->value,
                           sizeof(initial_write->value))) {
            fprintf(stderr, "failed initial write at 0x%08" PRIx32 "\n",
                    initial_write->address);
            kinetis_destroy(device);
            return EXIT_FAILURE;
        }
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
    ExecutionTrace trace = {.cpu = kinetis_cpu(device)};
    cortex_m4_set_trace(kinetis_cpu(device), record_trace, &trace);
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
    const uint16_t active_exception = (uint16_t)(cortex_m4_get_xpsr(kinetis_cpu(device)) & 0x1ffu);
    if (active_exception == 3u) {
        const uint32_t stack_pointer = cortex_m4_get_register(kinetis_cpu(device), 13u);
        uint32_t stacked_registers[8] = {0u};
        uint32_t stacked_lr = 0u;
        uint32_t stacked_pc = 0u;
        uint32_t stacked_xpsr = 0u;
        bool frame_read = true;
        for (uint8_t frame_index = 0u; frame_index < 8u; frame_index++)
            frame_read = frame_read &&
                         cortex_m4_read_memory(kinetis_cpu(device),
                                               stack_pointer + (uint32_t)frame_index * 4u, 4u,
                                               &stacked_registers[frame_index]);
        if (frame_read) {
            stacked_lr = stacked_registers[5];
            stacked_pc = stacked_registers[6];
            stacked_xpsr = stacked_registers[7];
            printf("exception=%u stacked_lr=0x%08" PRIx32 " stacked_pc=0x%08" PRIx32
                   " stacked_xpsr=0x%08" PRIx32 "\n",
                   active_exception, stacked_lr, stacked_pc, stacked_xpsr);
            printf("stacked_r0=0x%08" PRIx32 " stacked_r1=0x%08" PRIx32
                   " stacked_r2=0x%08" PRIx32 " stacked_r3=0x%08" PRIx32
                   " stacked_r12=0x%08" PRIx32 "\n",
                   stacked_registers[0], stacked_registers[1], stacked_registers[2],
                   stacked_registers[3], stacked_registers[4]);
        }
        const size_t trace_length =
            trace.count < sizeof(trace.addresses) / sizeof(trace.addresses[0])
                ? trace.count
                : sizeof(trace.addresses) / sizeof(trace.addresses[0]);
        const size_t trace_start = trace.count - trace_length;
        for (size_t trace_offset = 0u; trace_offset < trace_length; trace_offset++) {
            const size_t trace_index =
                (trace_start + trace_offset) % (sizeof(trace.addresses) / sizeof(trace.addresses[0]));
            printf("trace pc=0x%08" PRIx32 " opcode=0x%08" PRIx32 "\n",
                   trace.addresses[trace_index], trace.opcodes[trace_index]);
        }
    }
    CortexM4CoverageResult coverage_result = {0};
    if (coverage != NULL) {
        coverage_result = cortex_m4_coverage_result(coverage);
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
    const bool coverage_failed =
        coverage_result.covered_instructions < minimum_instructions_covered ||
        coverage_result.covered_branch_sites < minimum_branches_covered;
    if (coverage_failed)
        fprintf(stderr, "coverage minimum not reached\n");
    return failed || coverage_failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
