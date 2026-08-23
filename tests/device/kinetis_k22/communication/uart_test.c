#include "kinetis_k22.h"

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

static uint8_t read_byte(TestState* state, KinetisK22* device, uint32_t address) {
    uint8_t register_value = 0u;
    expect(state, kinetis_k22_read(device, address, &register_value, sizeof(register_value)),
           "kinetis_k22_read(device, address, &register_value, sizeof(register_value))");
    return register_value;
}

static void write_byte(TestState* state, KinetisK22* device, uint32_t address,
                       uint8_t write_value) {
    expect(state, kinetis_k22_write(device, address, &write_value, sizeof(write_value)),
           "kinetis_k22_write(device, address, &write_value, sizeof(write_value))");
}

int main(void) {
    TestState state = {0};
    KinetisK22* device = kinetis_k22_create(kinetis_k22_default_configuration());

    expect(&state, device != NULL, "device != NULL");
    write_byte(&state, device, UART1_C2, 0x20u);
    write_byte(&state, device, UART1_C3, 0x08u);
    expect(&state, kinetis_k22_uart1_receive(device, 0x5au, 0x08u),
           "kinetis_k22_uart1_receive(device, 0x5au, 0x08u)");
    expect(&state, (read_byte(&state, device, UART1_S1) & 0x20u) != 0,
           "(read_byte(&state, device, UART1_S1) & 0x20u) != 0");
    expect(&state, cortex_m4_get_irq_pending(kinetis_k22_cpu(device), UART1_IRQ),
           "cortex_m4_get_irq_pending(kinetis_k22_cpu(device), UART1_IRQ)");
    expect(&state, cortex_m4_get_irq_pending(kinetis_k22_cpu(device), UART1_ERROR_IRQ),
           "cortex_m4_get_irq_pending(kinetis_k22_cpu(device), UART1_ERROR_IRQ)");
    expect(&state, read_byte(&state, device, UART1_D) == 0x5au,
           "read_byte(&state, device, UART1_D) == 0x5au");
    expect(&state, (read_byte(&state, device, UART1_S1) & 0x20u) == 0,
           "(read_byte(&state, device, UART1_S1) & 0x20u) == 0");
    write_byte(&state, device, UART1_D, 0xa5u);
    uint8_t transmitted_value = 0u;
    expect(&state, kinetis_k22_uart1_transmit(device, &transmitted_value),
           "kinetis_k22_uart1_transmit(device, &transmitted_value)");
    expect(&state, transmitted_value == 0xa5u, "transmitted_value == 0xa5u");
    expect(&state, !kinetis_k22_uart1_transmit(device, &transmitted_value),
           "!kinetis_k22_uart1_transmit(device, &transmitted_value)");
    kinetis_k22_destroy(device);
    return test_finish(&state);
}
