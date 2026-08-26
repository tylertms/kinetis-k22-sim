#include "kinetis.h"

#include <stdint.h>

#include "kinetis_test.h"
#include "test.h"

enum {
    PIT_MCR = 0x40037000u,
    PIT0_LDVAL = 0x40037100u,
    PIT0_CVAL = 0x40037104u,
    PIT0_TCTRL = 0x40037108u,
    PIT0_TFLG = 0x4003710cu,
    PIT1_LDVAL = 0x40037110u,
    PIT1_CVAL = 0x40037114u,
    PIT1_TCTRL = 0x40037118u,
    PIT1_TFLG = 0x4003711cu,
    PIT0_IRQ = 48,
    SIM_CLKDIV1 = 0x40048044u,
    SIM_SCGC6 = 0x4004803cu,
};

static uint32_t read_u32(TestState* state, Kinetis* device, uint32_t address) {
    uint32_t output_value = 0;
    expect(state, kinetis_read(device, address, &output_value, sizeof(output_value)),
           "kinetis_read(device, address, &output_value, sizeof(output_value))");
    return output_value;
}

static void write_u32(TestState* state, Kinetis* device, uint32_t address, uint32_t value) {
    expect(state, kinetis_write(device, address, &value, sizeof(value)),
           "kinetis_write(device, address, &value, sizeof(value))");
}

int main(void) {
    TestState state = {0};
    Kinetis* device = kinetis_create(kinetis_configuration(KINETIS_PROFILE_MK22FN51212));
    expect(&state, device != NULL, "device != NULL");
    write_u32(&state, device, SIM_SCGC6, read_u32(&state, device, SIM_SCGC6) | (1u << 23));
    write_u32(&state, device, PIT_MCR, 0);
    write_u32(&state, device, PIT0_LDVAL, 2);
    write_u32(&state, device, PIT0_TCTRL, 3);
    kinetis_advance(device, 2);
    expect(&state, read_u32(&state, device, PIT0_CVAL) == 0,
           "read_u32(&state, device, PIT0_CVAL) == 0");
    expect(&state, read_u32(&state, device, PIT0_TFLG) == 0,
           "read_u32(&state, device, PIT0_TFLG) == 0");
    kinetis_advance(device, 1);
    expect(&state, read_u32(&state, device, PIT0_CVAL) == 2,
           "read_u32(&state, device, PIT0_CVAL) == 2");
    expect(&state, read_u32(&state, device, PIT0_TFLG) == 1,
           "read_u32(&state, device, PIT0_TFLG) == 1");
    expect(&state, cortex_m4_get_irq_pending(kinetis_cpu(device), PIT0_IRQ),
           "cortex_m4_get_irq_pending(kinetis_cpu(device), PIT0_IRQ)");
    write_u32(&state, device, PIT0_TFLG, 1);
    expect(&state, read_u32(&state, device, PIT0_TFLG) == 0,
           "read_u32(&state, device, PIT0_TFLG) == 0");
    cortex_m4_set_irq(kinetis_cpu(device), PIT0_IRQ, false);
    expect(&state, !cortex_m4_get_irq_pending(kinetis_cpu(device), PIT0_IRQ),
           "!cortex_m4_get_irq_pending(kinetis_cpu(device), PIT0_IRQ)");
    write_u32(&state, device, PIT0_TCTRL, 0);
    kinetis_advance(device, 100);
    expect(&state, read_u32(&state, device, PIT0_CVAL) == 2,
           "read_u32(&state, device, PIT0_CVAL) == 2");

    expect(&state, kinetis_test_disable_watchdog(device), "watchdog disabled for maximum advance");
    write_u32(&state, device, SIM_CLKDIV1, 0xf0000000u);
    expect(&state, kinetis_bus_clock_hz(device) > kinetis_core_clock_hz(device),
           "bus clock exceeds the divided core clock");
    write_u32(&state, device, PIT0_LDVAL, 0u);
    write_u32(&state, device, PIT0_TCTRL, 1u);
    write_u32(&state, device, PIT1_LDVAL, UINT32_MAX);
    write_u32(&state, device, PIT1_TCTRL, 5u);
    kinetis_advance(device, UINT32_MAX);
    expect(&state, read_u32(&state, device, PIT1_CVAL) == 0u,
           "saturated expiration count advances a chained maximum-period timer once");
    expect(&state, read_u32(&state, device, PIT1_TFLG) == 0u,
           "saturated expiration count does not overflow the chained timer");
    kinetis_destroy(device);
    return test_finish(&state);
}
