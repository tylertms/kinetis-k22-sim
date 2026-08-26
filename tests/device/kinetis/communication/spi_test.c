#include "kinetis.h"

#include <stdint.h>

#include "test.h"

enum {
    SIM_SCGC6 = 0x4004803cu,
    SPI0_SR = 0x4002c02cu,
    SPI0_RSER = 0x4002c030u,
    SPI0_PUSHR = 0x4002c034u,
    SPI0_POPR = 0x4002c038u,
    SPI0_IRQ = 26,
};

static uint32_t read_register(TestState* state, Kinetis* device, uint32_t address) {
    uint32_t register_value = 0u;
    expect(state, kinetis_read(device, address, &register_value, sizeof(register_value)),
           "kinetis_read(device, address, &register_value, sizeof(register_value))");
    return register_value;
}

static void write_register(TestState* state, Kinetis* device, uint32_t address,
                           uint32_t register_value) {
    expect(state, kinetis_write(device, address, &register_value, sizeof(register_value)),
           "kinetis_write(device, address, &register_value, sizeof(register_value))");
}

int main(void) {
    TestState state = {0};
    Kinetis* device = kinetis_create(kinetis_default_configuration());

    expect(&state, device != NULL, "device != NULL");
    write_register(&state, device, SIM_SCGC6,
                   read_register(&state, device, SIM_SCGC6) | (1u << 12));
    write_register(&state, device, SPI0_RSER, 1u << 17);
    expect(&state, kinetis_spi0_receive(device, 0x1234u), "kinetis_spi0_receive(device, 0x1234u)");
    expect(&state, (read_register(&state, device, SPI0_SR) & (1u << 17)) != 0,
           "(read_register(&state, device, SPI0_SR) & (1u << 17)) != 0");
    expect(&state, cortex_m4_get_irq_pending(kinetis_cpu(device), SPI0_IRQ),
           "cortex_m4_get_irq_pending(kinetis_cpu(device), SPI0_IRQ)");
    expect(&state, read_register(&state, device, SPI0_POPR) == 0x1234u,
           "read_register(&state, device, SPI0_POPR) == 0x1234u");
    expect(&state, (read_register(&state, device, SPI0_SR) & (1u << 17)) == 0,
           "(read_register(&state, device, SPI0_SR) & (1u << 17)) == 0");
    write_register(&state, device, SPI0_PUSHR, 0xabcdu);
    uint16_t transmitted_value = 0u;
    expect(&state, kinetis_spi0_transmit(device, &transmitted_value),
           "kinetis_spi0_transmit(device, &transmitted_value)");
    expect(&state, transmitted_value == 0xabcdu, "transmitted_value == 0xabcdu");
    expect(&state, !kinetis_spi0_transmit(device, &transmitted_value),
           "!kinetis_spi0_transmit(device, &transmitted_value)");
    kinetis_destroy(device);
    return test_finish(&state);
}
