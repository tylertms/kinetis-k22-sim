#include "architecture/cortex_m4/internal.h"

#include <string.h>

#include "test.h"

typedef struct {
    uint8_t memory[1024];
    uint32_t rejected_read;
    uint32_t reset_count;
    bool clock_running;
} ApiBus;

static bool bus_read(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                     uint32_t* output_value) {
    ApiBus* bus = context;
    (void)access;
    if (address == bus->rejected_read || (uint64_t)address + size > sizeof(bus->memory)) {
        return false;
    }
    *output_value = 0u;
    memcpy(output_value, bus->memory + address, size);
    return true;
}

static bool bus_write(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                      uint32_t write_value) {
    ApiBus* bus = context;
    (void)access;
    if ((uint64_t)address + size > sizeof(bus->memory)) {
        return false;
    }
    memcpy(bus->memory + address, &write_value, size);
    return true;
}

static void bus_reset(void* context) {
    ApiBus* bus = context;
    bus->reset_count++;
}

static bool bus_clock_running(void* context) {
    ApiBus* bus = context;
    return bus->clock_running;
}

static CortexM4* create_cpu(TestState* state, ApiBus* bus) {
    bus->rejected_read = UINT32_MAX;
    bus->clock_running = true;
    const uint32_t vectors[2] = {0x300u, 0x101u};
    memcpy(bus->memory, vectors, sizeof(vectors));
    CortexM4* cpu = cortex_m4_create((CortexM4Bus){bus, bus_read, bus_write, NULL, bus_reset});
    expect(state, cpu != NULL, "cpu != NULL");
    cortex_m4_set_clock(cpu, bus_clock_running, bus);
    expect(state, cortex_m4_reset(cpu, 0u), "cortex_m4_reset(cpu, 0u)");
    return cpu;
}

static void load_instruction(ApiBus* bus, uint16_t first_halfword, uint16_t second_halfword) {
    memcpy(bus->memory + 0x100u, &first_halfword, sizeof(first_halfword));
    memcpy(bus->memory + 0x102u, &second_halfword, sizeof(second_halfword));
}

static void test_creation_and_configuration(TestState* state) {
    expect(state, cortex_m4_create((CortexM4Bus){NULL, NULL, bus_write, NULL, NULL}) == NULL,
           "core creation requires a readable bus");
    expect(state, cortex_m4_create((CortexM4Bus){NULL, bus_read, NULL, NULL, NULL}) == NULL,
           "core creation requires a writable bus");
    ApiBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->mpu_region_number = 7u;
    expect(state, cortex_m4_configure_implementation(cpu, 32u, 4u, 4u),
           "cortex_m4_configure_implementation(cpu, 32u, 4u, 4u)");
    expect(state, cpu->mpu_region_number == 0u, "cpu->mpu_region_number == 0u");
    cortex_m4_destroy(cpu);
}

static void test_reset_failures(TestState* state) {
    ApiBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    bus.rejected_read = 0u;
    expect(state, !cortex_m4_reset(cpu, 0u), "!cortex_m4_reset(cpu, 0u)");
    expect(state, cortex_m4_get_stop(cpu) == CORTEX_M4_STOP_BUS_FAULT,
           "cortex_m4_get_stop(cpu) == CORTEX_M4_STOP_BUS_FAULT");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    const uint32_t invalid_vector = 0x100u;
    memcpy(bus.memory + 4u, &invalid_vector, sizeof(invalid_vector));
    expect(state, !cortex_m4_reset(cpu, 0u), "!cortex_m4_reset(cpu, 0u)");
    expect(state, cortex_m4_get_stop(cpu) == CORTEX_M4_STOP_USAGE_FAULT,
           "cortex_m4_get_stop(cpu) == CORTEX_M4_STOP_USAGE_FAULT");
    cortex_m4_destroy(cpu);
}

static void test_step_guards(TestState* state) {
    expect(state, cortex_m4_step(NULL).stop == CORTEX_M4_STOP_LOCKUP,
           "cortex_m4_step(NULL).stop == CORTEX_M4_STOP_LOCKUP");
    expect(state, cortex_m4_run(NULL, (CortexM4RunLimits){0u, 0u}).stop == CORTEX_M4_STOP_LOCKUP,
           "cortex_m4_run(NULL, (CortexM4RunLimits){0u, 0u}).stop == CORTEX_M4_STOP_LOCKUP");
    ApiBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->stop = CORTEX_M4_STOP_BREAKPOINT;
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_BREAKPOINT,
           "cortex_m4_step(cpu).stop == CORTEX_M4_STOP_BREAKPOINT");
    cpu->stop = CORTEX_M4_STOP_RUNNING;
    cortex_m4_request_stop(cpu);
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_LIMIT,
           "cortex_m4_step(cpu).stop == CORTEX_M4_STOP_LIMIT");
    cortex_m4_request_stop(NULL);
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    bus.clock_running = false;
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_CLOCK,
           "a stopped clock prevents a core step");
    expect(state, cortex_m4_run(cpu, (CortexM4RunLimits){1u, 1u}).stop == CORTEX_M4_STOP_CLOCK,
           "a stopped clock ends a core run");
    expect(state, cortex_m4_get_register(cpu, 15u) == 0x100u,
           "a stopped clock preserves the program counter");
    expect(state,
           cortex_m4_get_instruction_count(cpu) == 0u && cortex_m4_get_cycle_count(cpu) == 0u,
           "a stopped clock retires no work");
    bus.clock_running = true;
    expect(state, cortex_m4_run(cpu, (CortexM4RunLimits){1u, 0u}).stop == CORTEX_M4_STOP_LIMIT,
           "a core run resumes when its clock returns");
    expect(state, cortex_m4_get_instruction_count(cpu) == 1u,
           "the resumed core retires its next instruction");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    expect(state, cortex_m4_set_breakpoint(cpu, 0u, 0x100u, true),
           "cortex_m4_set_breakpoint(cpu, 0u, 0x100u, true)");
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_BREAKPOINT,
           "cortex_m4_step(cpu).stop == CORTEX_M4_STOP_BREAKPOINT");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    expect(state, cortex_m4_write_memory(cpu, 0xe000edf0u, 4u, 0xa05f0003u),
           "cortex_m4_write_memory(cpu, 0xe000edf0u, 4u, 0xa05f0003u)");
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_BREAKPOINT,
           "cortex_m4_step(cpu).stop == CORTEX_M4_STOP_BREAKPOINT");
    cortex_m4_destroy(cpu);
}

static void test_fetch_faults(TestState* state) {
    ApiBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    bus.rejected_read = 0x100u;
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_RUNNING,
           "cortex_m4_step(cpu).stop == CORTEX_M4_STOP_RUNNING");
    expect(state, (cortex_m4_get_fault_status(cpu) & (1u << 8)) != 0u,
           "(cortex_m4_get_fault_status(cpu) & (1u << 8)) != 0u");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    load_instruction(&bus, 0xf000u, 0u);
    bus.rejected_read = 0x102u;
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_RUNNING,
           "cortex_m4_step(cpu).stop == CORTEX_M4_STOP_RUNNING");
    expect(state, (cortex_m4_get_fault_status(cpu) & (1u << 8)) != 0u,
           "(cortex_m4_get_fault_status(cpu) & (1u << 8)) != 0u");
    cortex_m4_destroy(cpu);
}

static void test_reset_request(TestState* state) {
    ApiBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    load_instruction(&bus, 0x6001u, 0xbe00u);
    cortex_m4_set_register(cpu, 0u, 0xe000ed0cu);
    cortex_m4_set_register(cpu, 1u, 0x05fa0004u);
    expect(state, cortex_m4_step(cpu).stop == CORTEX_M4_STOP_RUNNING,
           "cortex_m4_step(cpu).stop == CORTEX_M4_STOP_RUNNING");
    expect(state, bus.reset_count == 1u, "bus.reset_count == 1u");
    cortex_m4_destroy(cpu);
}

static void test_api_boundaries(TestState* state) {
    ApiBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->control = CORTEX_M4_CONTROL_SPSEL;
    cpu->psp = 0x333u;
    expect(state, cortex_m4_read_register_internal(cpu, 13u) == 0x333u,
           "cortex_m4_read_register_internal(cpu, 13u) == 0x333u");
    cortex_m4_write_register_internal(cpu, 13u, 0x445u);
    expect(state, cpu->psp == 0x444u, "cpu->psp == 0x444u");
    expect(state, cortex_m4_get_fault_address(cpu) == 0u, "cortex_m4_get_fault_address(cpu) == 0u");
    expect(state, cortex_m4_get_fault_address(NULL) == 0u,
           "cortex_m4_get_fault_address(NULL) == 0u");
    expect(state, cortex_m4_get_instruction_count(NULL) == 0u,
           "cortex_m4_get_instruction_count(NULL) == 0u");
    expect(state, cortex_m4_get_cycle_count(NULL) == 0u, "cortex_m4_get_cycle_count(NULL) == 0u");
    expect(state,
           cortex_m4_get_register(NULL, 0u) == 0u &&
               cortex_m4_get_register(cpu, CORTEX_M4_REGISTER_COUNT) == 0u,
           "invalid core register reads return zero");
    cortex_m4_set_register(NULL, 0u, 1u);
    cortex_m4_set_register(cpu, CORTEX_M4_REGISTER_COUNT, 1u);
    expect(state,
           cortex_m4_get_xpsr(NULL) == 0u && cortex_m4_get_control(NULL) == 0u &&
               cortex_m4_get_fault_status(NULL) == 0u &&
               cortex_m4_get_stop(NULL) == CORTEX_M4_STOP_LOCKUP,
           "null core state getters return safe defaults");
    cortex_m4_set_xpsr(NULL, 0u);
    cortex_m4_set_control(NULL, 0u);
    cpu->xpsr = CORTEX_M4_XPSR_T | 3u;
    cpu->control = 0u;
    cortex_m4_set_control(cpu, 3u);
    expect(state, cpu->control == 0u, "handler mode rejects control changes");
    cpu->xpsr = CORTEX_M4_XPSR_T;
    expect(state,
           cortex_m4_get_fp_register(NULL, 0u) == 0u &&
               cortex_m4_get_fp_register(cpu, CORTEX_M4_FP_REGISTER_COUNT) == 0u,
           "invalid floating-point register reads return zero");
    cortex_m4_set_fp_register(NULL, 0u, 1u);
    cortex_m4_set_fp_register(cpu, CORTEX_M4_FP_REGISTER_COUNT, 1u);
    expect(state, cortex_m4_get_fpscr(NULL) == 0u, "null FPSCR reads return zero");
    cortex_m4_set_fpscr(NULL, 1u);
    cortex_m4_set_nzcv(cpu, 0x80000000u, true, true);
    expect(state, (cpu->xpsr & CORTEX_M4_XPSR_V) != 0u, "(cpu->xpsr & CORTEX_M4_XPSR_V) != 0u");
    expect(state, cortex_m4_condition_passed(cpu, 14u), "cortex_m4_condition_passed(cpu, 14u)");
    expect(state, !cortex_m4_condition_passed(cpu, 15u), "!cortex_m4_condition_passed(cpu, 15u)");
    bool carry_out = false;
    expect(state, cortex_m4_shift(0x80000000u, 2u, 32u, false, &carry_out) == 0xffffffffu,
           "cortex_m4_shift(0x80000000u, 2u, 32u, false, &carry_out) == 0xffffffffu");
    expect(state, carry_out, "carry_out");
    expect(state, cortex_m4_shift(1u, 3u, 0u, true, &carry_out) == 0x80000000u,
           "cortex_m4_shift(1u, 3u, 0u, true, &carry_out) == 0x80000000u");
    expect(state, carry_out, "carry_out");
    expect(state, cortex_m4_bus_write(cpu, 0xe000ed94u, 4u, CORTEX_M4_ACCESS_DEBUG, 1u),
           "cortex_m4_bus_write(cpu, 0xe000ed94u, 4u, CORTEX_M4_ACCESS_DEBUG, 1u)");
    uint32_t output_value = 0u;
    expect(state,
           !cortex_m4_read_memory(NULL, 0u, 1u, &output_value) &&
               !cortex_m4_read_memory(cpu, 0u, 1u, NULL) &&
               !cortex_m4_write_memory(NULL, 0u, 1u, 0u),
           "null debugger memory accesses are rejected");
    expect(state,
           !cortex_m4_bus_read(cpu, 0u, 3u, CORTEX_M4_ACCESS_DATA, &output_value) &&
               !cortex_m4_bus_read(cpu, 0u, 1u, CORTEX_M4_ACCESS_DATA, NULL) &&
               !cortex_m4_bus_write(cpu, 0u, 3u, CORTEX_M4_ACCESS_DATA, 0u),
           "core bus rejects invalid access widths and outputs");
    cpu->ccr |= 1u << 3u;
    expect(state, !cortex_m4_data_write(cpu, 1u, 4u, CORTEX_M4_ACCESS_DATA, 0u),
           "!cortex_m4_data_write(cpu, 1u, 4u, CORTEX_M4_ACCESS_DATA, 0u)");
    expect(state, (cpu->cfsr & (1u << 24)) != 0u, "(cpu->cfsr & (1u << 24)) != 0u");
    cortex_m4_destroy(cpu);
}

int main(void) {
    TestState state = {0};
    test_creation_and_configuration(&state);
    test_reset_failures(&state);
    test_step_guards(&state);
    test_fetch_faults(&state);
    test_reset_request(&state);
    test_api_boundaries(&state);
    return test_finish(&state);
}
