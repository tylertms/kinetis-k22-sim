#include "kinetis.h"

#include <stdint.h>

#include "test.h"

int main(void) {
    TestState state = {0};
    KinetisConfiguration configuration = kinetis_default_configuration();
    configuration.flash_size = 4096;
    configuration.sram_size = 65536;
    Kinetis* device = kinetis_create(configuration);
    expect(&state, device != NULL, "device != NULL");
    const uint32_t vectors[2] = {0x20001000u, 0x00000101u};
    const uint16_t program[] = {0x2001u, 0x3002u, 0x0040u, 0x2806u, 0xd100u, 0xbe00u, 0xbe01u};
    expect(&state, kinetis_load(device, 0, vectors, sizeof(vectors)),
           "kinetis_load(device, 0, vectors, sizeof(vectors))");
    expect(&state, kinetis_load(device, 0x100, program, sizeof(program)),
           "kinetis_load(device, 0x100, program, sizeof(program))");
    expect(&state, kinetis_reset(device), "kinetis_reset(device)");
    test_connect_debugger(&state, kinetis_cpu(device));
    CortexM4Result result = cortex_m4_run(kinetis_cpu(device), (CortexM4RunLimits){20, 100});
    expect(&state, result.stop == CORTEX_M4_STOP_BREAKPOINT,
           "result.stop == CORTEX_M4_STOP_BREAKPOINT");
    expect(&state, cortex_m4_get_register(kinetis_cpu(device), 0) == 6,
           "cortex_m4_get_register(kinetis_cpu(device), 0) == 6");
    expect(&state, result.instructions == 6, "result.instructions == 6");
    kinetis_destroy(device);
    return test_finish(&state);
}
