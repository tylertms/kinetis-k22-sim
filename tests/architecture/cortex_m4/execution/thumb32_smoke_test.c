#include "kinetis.h"

#include <stdint.h>

#include "test.h"

static Kinetis* create_device(TestState* state) {
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    configuration.flash_size = 4096;
    configuration.sram_size = 65536;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "device != NULL");
    const uint32_t vectors[2] = {0x20001000u, 0x00000101u};
    expect(state, kinetis_load(device, 0, vectors, sizeof(vectors)),
           "kinetis_load(device, 0, vectors, sizeof(vectors))");
    return device;
}

static void load_instruction(TestState* state, Kinetis* device, uint16_t first, uint16_t second) {
    const uint8_t program[] = {
        (uint8_t)first, (uint8_t)(first >> 8), (uint8_t)second, (uint8_t)(second >> 8), 0x00, 0xbe};
    expect(state, kinetis_load(device, 0x100, program, sizeof(program)),
           "kinetis_load(device, 0x100, program, sizeof(program))");
    expect(state, kinetis_reset(device), "kinetis_reset(device)");
    test_connect_debugger(state, kinetis_cpu(device));
}

static void execute(TestState* state, Kinetis* device) {
    const CortexM4Result result = cortex_m4_run(kinetis_cpu(device), (CortexM4RunLimits){2, 10});
    expect(state, result.stop == CORTEX_M4_STOP_BREAKPOINT,
           "result.stop == CORTEX_M4_STOP_BREAKPOINT");
}

int main(void) {
    TestState state = {0};
    Kinetis* device = create_device(&state);
    CortexM4* cpu = kinetis_cpu(device);

    load_instruction(&state, device, 0xfa01u, 0xf303u);
    cortex_m4_set_register(cpu, 1, 1);
    cortex_m4_set_register(cpu, 3, 4);
    execute(&state, device);
    expect(&state, cortex_m4_get_register(cpu, 3) == 16, "cortex_m4_get_register(cpu, 3) == 16");

    const uint16_t register_shifts[] = {0xfa01u, 0xfa21u, 0xfa41u, 0xfa61u};
    for (size_t index = 0; index < sizeof(register_shifts) / sizeof(register_shifts[0]); index++) {
        load_instruction(&state, device, register_shifts[index], 0xf003u);
        cortex_m4_set_register(cpu, 1, 0x81234567u);
        cortex_m4_set_register(cpu, 3, 0);
        cortex_m4_set_xpsr(cpu, cortex_m4_get_xpsr(cpu) | (1u << 29));
        execute(&state, device);
        expect(&state, cortex_m4_get_register(cpu, 0) == 0x81234567u,
               "cortex_m4_get_register(cpu, 0) == 0x81234567u");
        expect(&state, (cortex_m4_get_xpsr(cpu) & (1u << 29)) != 0,
               "(cortex_m4_get_xpsr(cpu) & (1u << 29)) != 0");
    }

    load_instruction(&state, device, 0xfbb2u, 0xf3f3u);
    cortex_m4_set_register(cpu, 2, 100);
    cortex_m4_set_register(cpu, 3, 7);
    execute(&state, device);
    expect(&state, cortex_m4_get_register(cpu, 3) == 14, "cortex_m4_get_register(cpu, 3) == 14");

    load_instruction(&state, device, 0xfb01u, 0xf303u);
    cortex_m4_set_register(cpu, 1, 0x10000u);
    cortex_m4_set_register(cpu, 3, 0x10000u);
    execute(&state, device);
    expect(&state, cortex_m4_get_register(cpu, 3) == 0, "cortex_m4_get_register(cpu, 3) == 0");

    load_instruction(&state, device, 0xfba3u, 0x1302u);
    cortex_m4_set_register(cpu, 3, 0xffffffffu);
    cortex_m4_set_register(cpu, 2, 2);
    execute(&state, device);
    expect(&state, cortex_m4_get_register(cpu, 1) == 0xfffffffeu,
           "cortex_m4_get_register(cpu, 1) == 0xfffffffeu");
    expect(&state, cortex_m4_get_register(cpu, 3) == 1, "cortex_m4_get_register(cpu, 3) == 1");

    load_instruction(&state, device, 0xf3c2u, 0x020eu);
    cortex_m4_set_register(cpu, 2, 0xffffffffu);
    execute(&state, device);
    expect(&state, cortex_m4_get_register(cpu, 2) == 0x7fffu,
           "cortex_m4_get_register(cpu, 2) == 0x7fffu");

    load_instruction(&state, device, 0xf442u, 0x0270u);
    cortex_m4_set_register(cpu, 2, 1);
    execute(&state, device);
    expect(&state, cortex_m4_get_register(cpu, 2) == 0x00f00001u,
           "cortex_m4_get_register(cpu, 2) == 0x00f00001u");

    kinetis_destroy(device);
    return test_finish(&state);
}
