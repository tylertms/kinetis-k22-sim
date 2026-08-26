#include "kinetis.h"

#include <stdint.h>

#include "test.h"

enum {
    UART1_C2 = 0x4006b003u,
    UART1_S1 = 0x4006b004u,
    UART1_C3 = 0x4006b006u,
    UART1_D = 0x4006b007u,
    UART1_IRQ = 33,
    UART1_ERROR_IRQ = 34,
};

static uint8_t read_byte(TestState* state, Kinetis* device, uint32_t address) {
    uint8_t register_value = 0u;
    expect(state, kinetis_read(device, address, &register_value, sizeof(register_value)),
           "kinetis_read(device, address, &register_value, sizeof(register_value))");
    return register_value;
}

static void write_byte(TestState* state, Kinetis* device, uint32_t address, uint8_t write_value) {
    expect(state, kinetis_write(device, address, &write_value, sizeof(write_value)),
           "kinetis_write(device, address, &write_value, sizeof(write_value))");
}

int main(void) {
    TestState state = {0};
    Kinetis* device = kinetis_create(kinetis_default_configuration());

    expect(&state, device != NULL, "device != NULL");
    write_byte(&state, device, UART1_C2, 0x20u);
    write_byte(&state, device, UART1_C3, 0x08u);
    expect(&state, kinetis_uart1_receive(device, 0x5au, 0x08u),
           "kinetis_uart1_receive(device, 0x5au, 0x08u)");
    expect(&state, (read_byte(&state, device, UART1_S1) & 0x20u) != 0,
           "(read_byte(&state, device, UART1_S1) & 0x20u) != 0");
    expect(&state, cortex_m4_get_irq_pending(kinetis_cpu(device), UART1_IRQ),
           "cortex_m4_get_irq_pending(kinetis_cpu(device), UART1_IRQ)");
    expect(&state, cortex_m4_get_irq_pending(kinetis_cpu(device), UART1_ERROR_IRQ),
           "cortex_m4_get_irq_pending(kinetis_cpu(device), UART1_ERROR_IRQ)");
    expect(&state, read_byte(&state, device, UART1_D) == 0x5au,
           "read_byte(&state, device, UART1_D) == 0x5au");
    expect(&state, (read_byte(&state, device, UART1_S1) & 0x20u) == 0,
           "(read_byte(&state, device, UART1_S1) & 0x20u) == 0");
    expect(&state, kinetis_uart1_error(device, 0xffu), "kinetis_uart1_error(device, 0xffu)");
    expect(&state, (read_byte(&state, device, UART1_S1) & 0x0fu) == 0x0fu,
           "(read_byte(&state, device, UART1_S1) & 0x0fu) == 0x0fu");
    expect(&state, read_byte(&state, device, UART1_D) == 0u,
           "read_byte(&state, device, UART1_D) == 0u");
    expect(&state, (read_byte(&state, device, UART1_S1) & 0x0fu) == 0u,
           "(read_byte(&state, device, UART1_S1) & 0x0fu) == 0u");
    write_byte(&state, device, UART1_D, 0xa5u);
    uint8_t transmitted_value = 0u;
    expect(&state, kinetis_uart1_transmit(device, &transmitted_value),
           "kinetis_uart1_transmit(device, &transmitted_value)");
    expect(&state, transmitted_value == 0xa5u, "transmitted_value == 0xa5u");
    expect(&state, !kinetis_uart1_transmit(device, &transmitted_value),
           "!kinetis_uart1_transmit(device, &transmitted_value)");
    kinetis_destroy(device);
    return test_finish(&state);
}
