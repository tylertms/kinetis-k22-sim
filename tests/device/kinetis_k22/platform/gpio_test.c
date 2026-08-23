#include "kinetis_k22.h"

#include <stdint.h>

#include "test.h"

enum {
    GPIOA_PDOR = 0x400ff000u,
    GPIOA_PSOR = 0x400ff004u,
    GPIOA_PCOR = 0x400ff008u,
    GPIOA_PTOR = 0x400ff00cu,
    GPIOA_PDIR = 0x400ff010u,
    GPIOA_PDDR = 0x400ff014u,
    PORTA_PCR3 = 0x4004900cu,
    PORTA_IRQ = 59,
};

static uint32_t read32(TestState* state, KinetisK22* device, uint32_t register_address) {
    uint32_t read_value = 0u;
    expect(state, kinetis_k22_read(device, register_address, &read_value, sizeof(read_value)),
           "kinetis_k22_read(device, register_address, &read_value, sizeof(read_value))");
    return read_value;
}

static void write32(TestState* state, KinetisK22* device, uint32_t register_address,
                    uint32_t write_value) {
    expect(state, kinetis_k22_write(device, register_address, &write_value, sizeof(write_value)),
           "kinetis_k22_write(device, register_address, &write_value, sizeof(write_value))");
}

int main(void) {
    TestState state = {0};
    KinetisK22* device = kinetis_k22_create(kinetis_k22_default_configuration());
    expect(&state, device != NULL, "device != NULL");
    for (uint8_t pin_index = 0u; pin_index < 32u; pin_index++) {
        if (pin_index != 3u) {
            kinetis_k22_gpio_drive(device, 0u, pin_index, false);
        }
    }

    write32(&state, device, PORTA_PCR3, 1u << 8);
    write32(&state, device, GPIOA_PDDR, 1u << 3);
    write32(&state, device, GPIOA_PSOR, 1u << 3);
    expect(&state, read32(&state, device, GPIOA_PDOR) == (1u << 3),
           "read32(&state, device, GPIOA_PDOR) == (1u << 3)");
    expect(&state, read32(&state, device, GPIOA_PDIR) == (1u << 3),
           "read32(&state, device, GPIOA_PDIR) == (1u << 3)");
    write32(&state, device, GPIOA_PTOR, 1u << 3);
    expect(&state, read32(&state, device, GPIOA_PDOR) == 0,
           "read32(&state, device, GPIOA_PDOR) == 0");
    write32(&state, device, GPIOA_PDOR, 1u << 3);
    write32(&state, device, GPIOA_PCOR, 1u << 3);
    expect(&state, read32(&state, device, GPIOA_PDOR) == 0,
           "read32(&state, device, GPIOA_PDOR) == 0");

    write32(&state, device, GPIOA_PDDR, 0);
    write32(&state, device, PORTA_PCR3, 3u);
    expect(&state, read32(&state, device, GPIOA_PDIR) == (1u << 3),
           "read32(&state, device, GPIOA_PDIR) == (1u << 3)");
    write32(&state, device, PORTA_PCR3, 9u << 16);
    kinetis_k22_gpio_drive(device, 0, 3, false);
    kinetis_k22_gpio_drive(device, 0, 3, true);
    expect(&state, (read32(&state, device, PORTA_PCR3) & (1u << 24)) != 0,
           "(read32(&state, device, PORTA_PCR3) & (1u << 24)) != 0");
    expect(&state, cortex_m4_get_irq_pending(kinetis_k22_cpu(device), PORTA_IRQ),
           "cortex_m4_get_irq_pending(kinetis_k22_cpu(device), PORTA_IRQ)");
    cortex_m4_set_irq(kinetis_k22_cpu(device), PORTA_IRQ, false);
    expect(&state, (read32(&state, device, PORTA_PCR3) & (1u << 24)) != 0,
           "(read32(&state, device, PORTA_PCR3) & (1u << 24)) != 0");
    expect(&state, cortex_m4_get_irq_pending(kinetis_k22_cpu(device), PORTA_IRQ),
           "cortex_m4_get_irq_pending(kinetis_k22_cpu(device), PORTA_IRQ)");
    write32(&state, device, PORTA_PCR3, 1u << 24);
    expect(&state, (read32(&state, device, PORTA_PCR3) & (1u << 24)) == 0,
           "(read32(&state, device, PORTA_PCR3) & (1u << 24)) == 0");
    cortex_m4_set_irq(kinetis_k22_cpu(device), PORTA_IRQ, false);
    expect(&state, (read32(&state, device, PORTA_PCR3) & (1u << 24)) == 0,
           "(read32(&state, device, PORTA_PCR3) & (1u << 24)) == 0");
    expect(&state, !cortex_m4_get_irq_pending(kinetis_k22_cpu(device), PORTA_IRQ),
           "!cortex_m4_get_irq_pending(kinetis_k22_cpu(device), PORTA_IRQ)");

    KinetisK22* copied_device = kinetis_k22_create(kinetis_k22_default_configuration());
    expect(&state, copied_device != NULL, "copied_device != NULL");
    expect(&state, kinetis_k22_copy(copied_device, device),
           "kinetis_k22_copy(copied_device, device)");
    expect(&state, read32(&state, copied_device, GPIOA_PDIR) == (1u << 3),
           "read32(&state, copied_device, GPIOA_PDIR) == (1u << 3)");
    kinetis_k22_gpio_release(copied_device, 0, 3);
    expect(&state, read32(&state, copied_device, GPIOA_PDIR) == 0,
           "read32(&state, copied_device, GPIOA_PDIR) == 0");
    expect(&state, read32(&state, device, GPIOA_PDIR) == (1u << 3),
           "read32(&state, device, GPIOA_PDIR) == (1u << 3)");
    kinetis_k22_destroy(copied_device);
    kinetis_k22_destroy(device);
    return test_finish(&state);
}
