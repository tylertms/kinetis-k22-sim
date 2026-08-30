#include "kinetis.h"

#include <stdint.h>

#include "test.h"

typedef struct {
    uint32_t addresses[8];
    uint32_t opcodes[8];
    bool executed[8];
    uint8_t count;
} TraceState;

static void capture_trace(void* context, uint32_t address, uint32_t opcode, bool executed) {
    TraceState* trace = context;
    if (trace->count < 8) {
        trace->addresses[trace->count] = address;
        trace->opcodes[trace->count] = opcode;
        trace->executed[trace->count] = executed;
        trace->count++;
    }
}

int main(void) {
    TestState state = {0};
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    configuration.flash_size = 4096;
    configuration.sram_size = 4096;
    Kinetis* device = kinetis_create(configuration);
    expect(&state, device != NULL, "device != NULL");

    const uint32_t vectors[2] = {0x20000100u, 0x00000101u};
    const uint16_t program[] = {0x2000u, 0x2801u, 0xbf08u, 0x3001u, 0xf240u, 0x0102u, 0xbe00u};
    expect(&state, kinetis_load(device, 0, vectors, sizeof(vectors)),
           "kinetis_load(device, 0, vectors, sizeof(vectors))");
    expect(&state, kinetis_load(device, 0x100, program, sizeof(program)),
           "kinetis_load(device, 0x100, program, sizeof(program))");
    expect(&state, kinetis_reset(device), "kinetis_reset(device)");
    test_connect_debugger(&state, kinetis_cpu(device));

    TraceState trace = {0};
    cortex_m4_set_trace(kinetis_cpu(device), capture_trace, &trace);
    const CortexM4Result result = cortex_m4_run(kinetis_cpu(device), (CortexM4RunLimits){16, 32});
    expect(&state, result.stop == CORTEX_M4_STOP_BREAKPOINT,
           "result.stop == CORTEX_M4_STOP_BREAKPOINT");
    expect(&state, trace.count == 6, "trace.count == 6");
    expect(&state, trace.addresses[0] == 0x100u, "trace.addresses[0] == 0x100u");
    expect(&state, trace.addresses[3] == 0x106u, "trace.addresses[3] == 0x106u");
    expect(&state, trace.addresses[4] == 0x108u, "trace.addresses[4] == 0x108u");
    expect(&state, trace.opcodes[4] == 0xf2400102u, "trace.opcodes[4] == 0xf2400102u");
    expect(&state, !trace.executed[3], "!trace.executed[3]");
    expect(&state, trace.executed[4], "trace.executed[4]");
    expect(&state, cortex_m4_get_register(kinetis_cpu(device), 0) == 0,
           "cortex_m4_get_register(kinetis_cpu(device), 0) == 0");
    expect(&state, cortex_m4_get_register(kinetis_cpu(device), 1) == 2,
           "cortex_m4_get_register(kinetis_cpu(device), 1) == 2");

    cortex_m4_set_trace(kinetis_cpu(device), NULL, NULL);
    expect(&state, kinetis_reset(device), "kinetis_reset(device)");
    expect(&state, trace.count == 6, "trace.count == 6");

    CortexM4Coverage* coverage = cortex_m4_coverage_create(0x100u, 16u);
    expect(&state, coverage != NULL, "coverage != NULL");
    for (uint32_t address = 0x100u; address <= 0x106u; address += 2u) {
        expect(&state, cortex_m4_coverage_define_instruction(coverage, address, false),
               "coverage defines a narrow instruction");
    }
    expect(&state, cortex_m4_coverage_define_instruction(coverage, 0x108u, false),
           "coverage defines a wide instruction");
    expect(&state, cortex_m4_coverage_define_instruction(coverage, 0x10cu, false),
           "coverage defines the breakpoint instruction");
    cortex_m4_set_coverage(kinetis_cpu(device), coverage);
    expect(&state, kinetis_reset(device), "coverage reset succeeds");
    test_connect_debugger(&state, kinetis_cpu(device));
    const CortexM4Result coverage_run =
        cortex_m4_run(kinetis_cpu(device), (CortexM4RunLimits){16, 32});
    expect(&state, coverage_run.stop == CORTEX_M4_STOP_BREAKPOINT,
           "coverage run reaches the breakpoint");
    CortexM4CoverageResult coverage_result = cortex_m4_coverage_result(coverage);
    expect(&state, coverage_result.instructions == 5u, "coverage records executed instructions");
    expect(&state, coverage_result.skipped == 1u, "coverage records skipped instructions");
    expect(&state, coverage_result.outside_range == 0u, "coverage stays inside its range");
    expect(&state, coverage_result.unique_instructions == 5u,
           "coverage counts unique instructions");
    expect(&state, coverage_result.unique_skipped == 1u, "coverage counts unique skips");
    expect(&state, coverage_result.conditional_branches == 0u, "coverage does not invent branches");
    expect(&state, coverage_result.covered_instructions == 5u,
           "coverage counts defined instructions reached");
    expect(&state, coverage_result.total_instructions == 6u,
           "coverage counts defined instructions");
    expect(&state,
           coverage_result.instruction_coverage_percent > 83.3 &&
               coverage_result.instruction_coverage_percent < 83.4,
           "coverage calculates the instruction percentage");
    expect(&state, cortex_m4_coverage_flags(coverage, 0x100u) == CORTEX_M4_COVERAGE_EXECUTED,
           "coverage exposes executed instructions");
    expect(&state, cortex_m4_coverage_flags(coverage, 0x106u) == CORTEX_M4_COVERAGE_SKIPPED,
           "coverage exposes skipped instructions");
    expect(&state, cortex_m4_coverage_flags(coverage, 0x10cu) == CORTEX_M4_COVERAGE_EXECUTED,
           "coverage exposes the final instruction");
    cortex_m4_coverage_clear(coverage);
    coverage_result = cortex_m4_coverage_result(coverage);
    expect(&state, coverage_result.covered_instructions == 0u,
           "coverage clear resets covered instructions");
    expect(&state, coverage_result.total_instructions == 6u,
           "coverage clear preserves defined instructions");
    cortex_m4_set_coverage(kinetis_cpu(device), NULL);
    cortex_m4_coverage_destroy(coverage);
    coverage = cortex_m4_coverage_create(0x100u, 16u);
    expect(&state, coverage != NULL, "branch coverage != NULL");
    expect(&state, cortex_m4_coverage_define_instruction(coverage, 0x100u, true),
           "coverage defines a narrow branch");
    expect(&state, cortex_m4_coverage_define_instruction(coverage, 0x104u, true),
           "coverage defines a wide branch");
    expect(&state, cortex_m4_coverage_define_instruction(coverage, 0x10au, true),
           "coverage defines a compare-and-branch instruction");
    cortex_m4_set_coverage(kinetis_cpu(device), coverage);
    const uint16_t branch_program[] = {0xd100u, 0xbf00u, 0xf040u, 0x8001u,
                                       0xbf00u, 0xb100u, 0xbf00u, 0xbe00u};
    expect(&state, kinetis_load(device, 0x100u, branch_program, sizeof(branch_program)),
           "branch coverage program is loaded");
    expect(&state, kinetis_reset(device), "taken branch reset succeeds");
    test_connect_debugger(&state, kinetis_cpu(device));
    expect(&state,
           cortex_m4_run(kinetis_cpu(device), (CortexM4RunLimits){16, 32}).stop ==
               CORTEX_M4_STOP_BREAKPOINT,
           "taken branch reaches the breakpoint");
    expect(&state, kinetis_reset(device), "not-taken branch reset succeeds");
    cortex_m4_set_register(kinetis_cpu(device), 0u, 1u);
    cortex_m4_set_xpsr(kinetis_cpu(device), (1u << 30u) | (1u << 24u));
    test_connect_debugger(&state, kinetis_cpu(device));
    expect(&state,
           cortex_m4_run(kinetis_cpu(device), (CortexM4RunLimits){16, 32}).stop ==
               CORTEX_M4_STOP_BREAKPOINT,
           "not-taken branch reaches the breakpoint");
    coverage_result = cortex_m4_coverage_result(coverage);
    expect(&state, coverage_result.instructions == 11u, "coverage clear resets counters");
    expect(&state, coverage_result.conditional_branches == 6u,
           "coverage records conditional branches");
    expect(&state, coverage_result.branches_taken == 3u, "coverage records taken branches");
    expect(&state, coverage_result.branches_not_taken == 3u, "coverage records not-taken branches");
    expect(&state, coverage_result.observed_branch_sites == 3u,
           "coverage counts observed branch sites");
    expect(&state, coverage_result.observed_branch_outcomes == 6u,
           "coverage counts observed branch outcomes");
    expect(&state, coverage_result.branch_sites_with_both_outcomes == 3u,
           "coverage counts branches with both outcomes");
    expect(&state, coverage_result.covered_branch_sites == 3u,
           "coverage counts defined branch sites reached");
    expect(&state, coverage_result.total_branch_sites == 3u,
           "coverage counts defined branch sites");
    expect(&state, coverage_result.branch_coverage_percent == 100.0,
           "coverage calculates the branch percentage");
    expect(&state, coverage_result.covered_branch_outcomes == 6u,
           "coverage counts covered branch outcomes");
    expect(&state, coverage_result.total_branch_outcomes == 6u,
           "coverage counts total branch outcomes");
    expect(&state, coverage_result.branch_outcome_coverage_percent == 100.0,
           "coverage calculates the branch outcome percentage");
    expect(&state, cortex_m4_coverage_select_range(coverage, 0x100u, 8u),
           "coverage selects an address range");
    CortexM4CoverageResult selected = cortex_m4_coverage_selected_result(coverage);
    expect(&state, selected.covered_instructions == 2u,
           "selected coverage counts covered instructions");
    expect(&state, selected.total_instructions == 2u, "selected coverage counts instructions");
    expect(&state, selected.covered_branch_outcomes == 4u,
           "selected coverage counts branch outcomes");
    expect(&state, selected.total_branch_outcomes == 4u,
           "selected coverage counts possible branch outcomes");
    expect(&state, selected.branch_outcome_coverage_percent == 100.0,
           "selected coverage calculates branch outcome coverage");
    expect(&state, !cortex_m4_coverage_select_range(coverage, 0x101u, 2u),
           "coverage rejects an odd selected address");
    expect(&state, !cortex_m4_coverage_select_range(coverage, 0x100u, 3u),
           "coverage rejects an odd selected size");
    expect(&state,
           cortex_m4_coverage_flags(coverage, 0x100u) ==
               (CORTEX_M4_COVERAGE_EXECUTED | CORTEX_M4_COVERAGE_BRANCH_TAKEN |
                CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN),
           "coverage exposes branch outcomes");
    expect(&state,
           cortex_m4_coverage_flags(coverage, 0x104u) ==
               (CORTEX_M4_COVERAGE_EXECUTED | CORTEX_M4_COVERAGE_BRANCH_TAKEN |
                CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN),
           "coverage exposes wide branch outcomes");
    expect(&state,
           cortex_m4_coverage_flags(coverage, 0x10au) ==
               (CORTEX_M4_COVERAGE_EXECUTED | CORTEX_M4_COVERAGE_BRANCH_TAKEN |
                CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN),
           "coverage exposes compare-and-branch outcomes");
    cortex_m4_set_coverage(kinetis_cpu(device), NULL);
    cortex_m4_coverage_destroy(coverage);
    expect(&state, cortex_m4_coverage_create(1u, 2u) == NULL, "coverage rejects odd addresses");
    expect(&state, cortex_m4_coverage_create(0u, 1u) == NULL, "coverage rejects odd sizes");
    expect(&state, cortex_m4_coverage_create(UINT32_MAX - 1u, 4u) == NULL,
           "coverage rejects overflowing ranges");
    expect(&state, !cortex_m4_coverage_define_instruction(NULL, 0u, false),
           "coverage definition requires coverage");
    cortex_m4_coverage_clear(NULL);
    cortex_m4_coverage_destroy(NULL);

    kinetis_destroy(device);
    return test_finish(&state);
}
