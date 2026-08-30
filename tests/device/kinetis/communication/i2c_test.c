#include "kinetis.h"

#include <stdint.h>

#include "test.h"

enum {
    I2C0_C1 = 0x40066002u,
    I2C0_S = 0x40066003u,
    I2C0_D = 0x40066004u,
    I2C0_FLT = 0x40066006u,
    I2C0_C1_RSTA = 0x42cc0048u,
    I2C0_C1_TXAK = 0x42cc004cu,
    I2C0_IRQ = 24,
    SIM_SCGC4 = 0x40048034u,
};

static uint8_t read_register8(TestState* state, Kinetis* device, uint32_t address) {
    uint8_t register_value = 0u;
    expect(state, kinetis_read(device, address, &register_value, sizeof(register_value)),
           "kinetis_read(device, address, &register_value, sizeof(register_value))");
    return register_value;
}

static void write_register8(TestState* state, Kinetis* device, uint32_t address,
                            uint8_t register_value) {
    expect(state, kinetis_write(device, address, &register_value, sizeof(register_value)),
           "kinetis_write(device, address, &register_value, sizeof(register_value))");
}

static void write_register32(TestState* state, Kinetis* device, uint32_t address,
                             uint32_t register_value) {
    expect(state, kinetis_write(device, address, &register_value, sizeof(register_value)),
           "kinetis_write(device, address, &register_value, sizeof(register_value))");
}

static uint32_t read_register32(TestState* state, Kinetis* device, uint32_t address) {
    uint32_t register_value = 0u;
    expect(state, kinetis_read(device, address, &register_value, sizeof(register_value)),
           "kinetis_read(device, address, &register_value, sizeof(register_value))");
    return register_value;
}

static void expect_transfer(TestState* state, Kinetis* device, KinetisI2cTransferType type,
                            uint8_t transfer_value) {
    KinetisI2cTransfer i2c_transfer = {0};
    expect(state, kinetis_i2c0_transfer(device, &i2c_transfer),
           "kinetis_i2c0_transfer(device, &i2c_transfer)");
    expect(state, i2c_transfer.type == type, "i2c_transfer.type == type");
    expect(state, i2c_transfer.value == transfer_value, "i2c_transfer.value == transfer_value");
}

int main(void) {
    TestState state = {0};
    Kinetis* device = kinetis_create(kinetis_configuration(KINETIS_PROFILE_MK22FN51212));

    expect(&state, device != NULL, "device != NULL");
    write_register32(&state, device, SIM_SCGC4,
                     read_register32(&state, device, SIM_SCGC4) | (1u << 6));
    write_register8(&state, device, I2C0_C1, 0xf0u);
    expect_transfer(&state, device, KINETIS_I2C_START, 0);
    write_register8(&state, device, I2C0_D, 0xa4u);
    expect_transfer(&state, device, KINETIS_I2C_WRITE, 0xa4u);
    kinetis_i2c0_acknowledge(device, true);
    expect(&state, (read_register8(&state, device, I2C0_S) & 3u) == 2u,
           "(read_register8(&state, device, I2C0_S) & 3u) == 2u");
    expect(&state, cortex_m4_get_irq_pending(kinetis_cpu(device), I2C0_IRQ),
           "cortex_m4_get_irq_pending(kinetis_cpu(device), I2C0_IRQ)");
    write_register8(&state, device, I2C0_S, 2u);
    expect(&state, (read_register8(&state, device, I2C0_S) & 2u) == 0,
           "(read_register8(&state, device, I2C0_S) & 2u) == 0");
    expect(&state, cortex_m4_get_irq_pending(kinetis_cpu(device), I2C0_IRQ),
           "cortex_m4_get_irq_pending(kinetis_cpu(device), I2C0_IRQ)");
    cortex_m4_set_irq(kinetis_cpu(device), I2C0_IRQ, false);
    expect(&state, !cortex_m4_get_irq_pending(kinetis_cpu(device), I2C0_IRQ),
           "!cortex_m4_get_irq_pending(kinetis_cpu(device), I2C0_IRQ)");

    write_register8(&state, device, I2C0_C1, 0xf0u);
    write_register32(&state, device, I2C0_C1_RSTA, 1u);
    expect_transfer(&state, device, KINETIS_I2C_REPEATED_START, 0);
    write_register8(&state, device, I2C0_C1, 0xe0u);
    write_register32(&state, device, I2C0_C1_TXAK, 1u);
    expect(&state, kinetis_i2c0_receive(device, 0x5au), "kinetis_i2c0_receive(device, 0x5au)");
    expect(&state, read_register8(&state, device, I2C0_D) == 0x5au,
           "read_register8(&state, device, I2C0_D) == 0x5au");
    expect_transfer(&state, device, KINETIS_I2C_READ, 0);
    write_register8(&state, device, I2C0_C1, 0xc0u);
    expect_transfer(&state, device, KINETIS_I2C_STOP, 0);
    KinetisI2cTransfer i2c_transfer = {0};
    expect(&state, !kinetis_i2c0_transfer(device, &i2c_transfer),
           "!kinetis_i2c0_transfer(device, &i2c_transfer)");

    const uint32_t vector_words[] = {0x20001000u, 0x00000101u};
    const uint16_t store_words[] = {0x7008u, 0x8008u};
    expect(&state, kinetis_load(device, 0, vector_words, sizeof(vector_words)),
           "kinetis_load(device, 0, vector_words, sizeof(vector_words))");
    expect(&state, kinetis_load(device, 0x100u, store_words, sizeof(store_words)),
           "kinetis_load(device, 0x100u, store_words, sizeof(store_words))");
    expect(&state, kinetis_reset(device), "kinetis_reset(device)");
    write_register32(&state, device, SIM_SCGC4,
                     read_register32(&state, device, SIM_SCGC4) | (1u << 6));
    write_register8(&state, device, I2C0_C1, 0xf0u);
    expect_transfer(&state, device, KINETIS_I2C_START, 0);
    cortex_m4_set_register(kinetis_cpu(device), 0, 1u);
    cortex_m4_set_register(kinetis_cpu(device), 1, I2C0_C1_RSTA);
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING");
    expect_transfer(&state, device, KINETIS_I2C_REPEATED_START, 0);
    cortex_m4_set_register(kinetis_cpu(device), 1, I2C0_C1_TXAK);
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING");
    expect(&state, (read_register8(&state, device, I2C0_C1) & 8u) != 0,
           "(read_register8(&state, device, I2C0_C1) & 8u) != 0");
    expect(&state, kinetis_i2c0_lose_arbitration(device), "kinetis_i2c0_lose_arbitration(device)");
    expect(&state, (read_register8(&state, device, I2C0_S) & 0x12u) == 0x12u,
           "(read_register8(&state, device, I2C0_S) & 0x12u) == 0x12u");
    expect(&state, (read_register8(&state, device, I2C0_C1) & 0x20u) == 0,
           "(read_register8(&state, device, I2C0_C1) & 0x20u) == 0");
    write_register8(&state, device, I2C0_FLT, 0x2au);
    expect(&state, read_register8(&state, device, I2C0_FLT) == 0x3au,
           "read_register8(&state, device, I2C0_FLT) == 0x3au");
    expect(&state, kinetis_i2c_detect_start(device, KINETIS_SERIAL_I2C0),
           "kinetis_i2c_detect_start(device, KINETIS_SERIAL_I2C0)");
    expect(&state, (read_register8(&state, device, I2C0_S) & 0x20u) != 0u,
           "I2C START asserts bus busy status");
    expect(&state, read_register8(&state, device, I2C0_FLT) == 0x3au,
           "read_register8(&state, device, I2C0_FLT) == 0x3au");
    write_register8(&state, device, I2C0_FLT, 0x3au);
    write_register8(&state, device, I2C0_S, 2u);
    expect(&state, read_register8(&state, device, I2C0_FLT) == 0x2au,
           "read_register8(&state, device, I2C0_FLT) == 0x2au");
    expect(&state, kinetis_i2c_detect_stop(device, KINETIS_SERIAL_I2C0),
           "kinetis_i2c_detect_stop(device, KINETIS_SERIAL_I2C0)");
    expect(&state, (read_register8(&state, device, I2C0_S) & 0x20u) == 0u,
           "I2C STOP clears bus busy status");
    expect(&state, read_register8(&state, device, I2C0_FLT) == 0x2au,
           "unmatched slave stop does not latch STOPF");
    write_register8(&state, device, I2C0_FLT, 0x6au);
    write_register8(&state, device, I2C0_S, 2u);
    expect(&state, read_register8(&state, device, I2C0_FLT) == 0x2au,
           "read_register8(&state, device, I2C0_FLT) == 0x2au");

    write_register8(&state, device, I2C0_C1, 0xf0u);
    expect_transfer(&state, device, KINETIS_I2C_START, 0);
    write_register8(&state, device, I2C0_D, 0xa4u);
    expect_transfer(&state, device, KINETIS_I2C_WRITE, 0xa4u);
    expect(&state, kinetis_i2c_acknowledge(device, KINETIS_SERIAL_I2C0, true),
           "kinetis_i2c_acknowledge(device, KINETIS_SERIAL_I2C0, true)");
    write_register8(&state, device, I2C0_FLT, 0x3au);
    write_register8(&state, device, I2C0_S, 2u);
    write_register8(&state, device, I2C0_C1, 0xe0u);
    (void)read_register8(&state, device, I2C0_D);
    expect_transfer(&state, device, KINETIS_I2C_READ, 0);
    expect(&state, kinetis_i2c_receive(device, KINETIS_SERIAL_I2C0, 0x5au),
           "kinetis_i2c_receive(device, KINETIS_SERIAL_I2C0, 0x5au)");
    expect(&state, (read_register8(&state, device, I2C0_S) & 2u) == 0u,
           "I2C input remains pending until transfer time elapses");
    expect(&state, kinetis_i2c_acknowledge(device, KINETIS_SERIAL_I2C0, true),
           "kinetis_i2c_acknowledge(device, KINETIS_SERIAL_I2C0, true)");
    expect(&state, (read_register8(&state, device, I2C0_S) & 2u) != 0u,
           "I2C input completes after transfer time elapses");
    expect(&state, read_register8(&state, device, I2C0_D) == 0x5au,
           "read_register8(&state, device, I2C0_D) == 0x5au");
    kinetis_destroy(device);
    return test_finish(&state);
}
