#include "kinetis.h"

#include <stdint.h>

#include "test.h"

enum {
    ADC0_SC1A = 0x4003b000u,
    ADC0_CFG1 = 0x4003b008u,
    ADC0_RA = 0x4003b010u,
    ADC0_SC3 = 0x4003b024u,
    ADC0_PG = 0x4003b02cu,
    ADC0_MG = 0x4003b030u,
    ADC0_CLPD = 0x4003b034u,
    ADC0_CLPS = 0x4003b038u,
    ADC0_IRQ = 39,
    SIM_SCGC6 = 0x4004803cu,
};

static uint32_t read32(TestState* state, Kinetis* device, uint32_t address) {
    uint32_t value = 0;
    expect(state, kinetis_read(device, address, &value, sizeof(value)),
           "kinetis_read(device, address, &value, sizeof(value))");
    return value;
}

static void write32(TestState* state, Kinetis* device, uint32_t address, uint32_t value) {
    expect(state, kinetis_write(device, address, &value, sizeof(value)),
           "kinetis_write(device, address, &value, sizeof(value))");
}

int main(void) {
    TestState state = {0};
    Kinetis* device = kinetis_create(kinetis_default_configuration());
    expect(&state, device != NULL, "device != NULL");
    write32(&state, device, SIM_SCGC6, read32(&state, device, SIM_SCGC6) | (1u << 27));
    write32(&state, device, ADC0_CFG1, 0x0cu);
    kinetis_set_adc0_channel(device, 7, 0x345u);
    write32(&state, device, ADC0_SC1A, 7u | 0x40u);
    kinetis_advance(device, 18u);
    expect(&state, (read32(&state, device, ADC0_SC1A) & 0x80u) != 0,
           "(read32(&state, device, ADC0_SC1A) & 0x80u) != 0");
    expect(&state, cortex_m4_get_irq_pending(kinetis_cpu(device), ADC0_IRQ),
           "cortex_m4_get_irq_pending(kinetis_cpu(device), ADC0_IRQ)");
    expect(&state, read32(&state, device, ADC0_RA) == 0x345u,
           "read32(&state, device, ADC0_RA) == 0x345u");
    expect(&state, (read32(&state, device, ADC0_SC1A) & 0x80u) == 0,
           "(read32(&state, device, ADC0_SC1A) & 0x80u) == 0");
    cortex_m4_set_irq(kinetis_cpu(device), ADC0_IRQ, false);
    write32(&state, device, ADC0_SC1A, 31u);
    expect(&state, (read32(&state, device, ADC0_SC1A) & 0x80u) == 0,
           "(read32(&state, device, ADC0_SC1A) & 0x80u) == 0");
    write32(&state, device, ADC0_SC3, 0x80u);
    expect(&state, (read32(&state, device, ADC0_SC3) & 0x80u) == 0,
           "(read32(&state, device, ADC0_SC3) & 0x80u) == 0");
    expect(&state, read32(&state, device, ADC0_PG) == 0x8200u,
           "read32(&state, device, ADC0_PG) == 0x8200u");
    expect(&state, read32(&state, device, ADC0_MG) == 0x8200u,
           "read32(&state, device, ADC0_MG) == 0x8200u");
    expect(&state, read32(&state, device, ADC0_CLPD) == 0x0au,
           "read32(&state, device, ADC0_CLPD) == 0x0au");
    expect(&state, read32(&state, device, ADC0_CLPS) == 0x20u,
           "read32(&state, device, ADC0_CLPS) == 0x20u");
    kinetis_destroy(device);
    return test_finish(&state);
}
