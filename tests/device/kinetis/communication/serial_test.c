#include "device/kinetis/communication/serial/api.h"
#include "test.h"

#include <string.h>

enum {
    LPUART0_BASE = 0x4002a000u,
    SPI0_BASE = 0x4002c000u,
    SPI1_BASE = 0x4002d000u,
    I2C0_BASE = 0x40066000u,
    I2C2_BASE = 0x400e6000u,
    UART0_BASE = 0x4006a000u,
    UART_S1 = 4u,
};

static uint32_t read_register(TestState* state, KinetisSerial* serial, uint32_t address,
                              uint8_t size) {
    uint32_t register_value = 0u;
    expect(state, kinetis_serial_read(serial, address, size, &register_value),
           "kinetis_serial_read(serial, address, size, &register_value)");
    return register_value;
}

static void write_register(TestState* state, KinetisSerial* serial, uint32_t address, uint8_t size,
                           uint32_t register_value) {
    expect(state, kinetis_serial_write(serial, address, size, register_value),
           "kinetis_serial_write(serial, address, size, register_value)");
}

static KinetisSerial create_serial(TestState* state, KinetisProfile profile_id) {
    KinetisSerial serial;
    expect(state, kinetis_serial_init(&serial, kinetis_profile_get(profile_id)),
           "kinetis_serial_init(&serial, kinetis_profile_get(profile_id))");
    return serial;
}

static void expect_event(TestState* state, KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                         KinetisSerialEventType event_type, uint16_t event_value) {
    KinetisSerialEvent serial_event;
    expect(state, kinetis_serial_pop_event(serial, &serial_event),
           "kinetis_serial_pop_event(serial, &serial_event)");
    expect(state, serial_event.endpoint == endpoint, "serial_event.endpoint == endpoint");
    expect(state, serial_event.type == event_type, "serial_event.type == event_type");
    expect(state, serial_event.value == event_value, "serial_event.value == event_value");
}

static void test_profiles_and_gates(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);
    uint32_t register_value = 0u;

    expect(state, serial.lpuart0.present, "serial.lpuart0.present");
    expect(state, serial.spi[1].present, "serial.spi[1].present");
    expect(state, !serial.i2c[2].present, "!serial.i2c[2].present");
    expect(state, !kinetis_serial_read(&serial, UART0_BASE + 4, 1, &register_value),
           "!kinetis_serial_read(&serial, UART0_BASE + 4, 1, &register_value)");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true)");
    expect(state, read_register(state, &serial, UART0_BASE + 4, 1) == 0xc0u,
           "read_register(state, &serial, UART0_BASE + 4, 1) == 0xc0u");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, false),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, false)");
    expect(state, !kinetis_serial_write(&serial, UART0_BASE + 3, 1, 0xffu),
           "!kinetis_serial_write(&serial, UART0_BASE + 3, 1, 0xffu)");
    expect(state, !kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C2, true),
           "!kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C2, true)");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART1, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART1, true)");
    expect(state,
           !kinetis_serial_read(&serial, serial.uart[1].base + serial.uart[1].block_size, 1,
                                &register_value),
           "!kinetis_serial_read(&serial, serial.uart[1].base + serial.uart[1].block_size, 1, "
           "&register_value)");

    serial = create_serial(state, KINETIS_PROFILE_MK22FN1M012);
    expect(state, !serial.lpuart0.present, "!serial.lpuart0.present");
    expect(state, serial.spi[1].present, "serial.spi[1].present");
    expect(state, serial.spi[2].present, "serial.spi[2].present");
    expect(state, serial.uart[3].present, "serial.uart[3].present");
    expect(state, serial.uart[4].present, "serial.uart[4].present");
    expect(state, serial.uart[5].present, "serial.uart[5].present");
    expect(state, serial.i2c[2].present, "serial.i2c[2].present");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C2, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C2, true)");
    expect(state, read_register(state, &serial, I2C2_BASE + 3, 1) == 0x80u,
           "read_register(state, &serial, I2C2_BASE + 3, 1) == 0x80u");
}

static void test_uart_transfer_status_interrupt_and_dma(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true)");
    serial.uart[0].registers[UART_S1] |= 0x10u;

    write_register(state, &serial, UART0_BASE + 3, 1, 0x10u);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0)");
    write_register(state, &serial, UART0_BASE, 1, 0);
    write_register(state, &serial, UART0_BASE + 1, 1, 1);
    write_register(state, &serial, UART0_BASE + 3, 1, 0x2cu);
    write_register(state, &serial, UART0_BASE + 6, 1, 0x02u);
    write_register(state, &serial, UART0_BASE + 0x0b, 1, 0x20u);
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART0, 0x15au, 0x02u),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART0, 0x15au, 0x02u)");

    kinetis_serial_advance(&serial, 159);
    expect(state, (read_register(state, &serial, UART0_BASE + 4, 1) & 0x20u) == 0,
           "(read_register(state, &serial, UART0_BASE + 4, 1) & 0x20u) == 0");
    kinetis_serial_advance(&serial, 1);
    uint32_t status_register = read_register(state, &serial, UART0_BASE + 4, 1);
    expect(state, (status_register & 0x22u) == 0x22u, "(status_register & 0x22u) == 0x22u");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0)");
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0_ERROR),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0_ERROR)");
    expect(state, kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_RECEIVE),
           "kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_RECEIVE)");
    expect(state, read_register(state, &serial, UART0_BASE + 7, 1) == 0x5au,
           "read_register(state, &serial, UART0_BASE + 7, 1) == 0x5au");
    expect(state, (read_register(state, &serial, UART0_BASE + 4, 1) & 0x2fu) == 0,
           "(read_register(state, &serial, UART0_BASE + 4, 1) & 0x2fu) == 0");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0)");

    write_register(state, &serial, UART0_BASE + 7, 1, 0xa5u);

    kinetis_serial_advance(&serial, 159);
    uint16_t transmitted_value = 0;
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "!kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value)");
    kinetis_serial_advance(&serial, 1);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value)");
    expect(state, transmitted_value == 0xa5u, "transmitted_value == 0xa5u");
    expect(state, (read_register(state, &serial, UART0_BASE + 4, 1) & 0xc0u) == 0xc0u,
           "(read_register(state, &serial, UART0_BASE + 4, 1) & 0xc0u) == 0xc0u");
}

static void test_uart_fifo_overrun_flush_and_copy(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART1, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART1, true)");

    uint32_t uart_base = serial.uart[1].base;
    write_register(state, &serial, uart_base, 1, 0);
    write_register(state, &serial, uart_base + 1, 1, 1);
    write_register(state, &serial, uart_base + 3, 1, 0x04u);
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART1, 0x11, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART1, 0x11, 0)");
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART1, 0x22, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART1, 0x22, 0)");
    kinetis_serial_advance(&serial, 320);
    expect(state, (read_register(state, &serial, uart_base + 4, 1) & 0x08u) != 0,
           "(read_register(state, &serial, uart_base + 4, 1) & 0x08u) != 0");
    expect(state, (read_register(state, &serial, uart_base + 0x12, 1) & 0x04u) != 0,
           "(read_register(state, &serial, uart_base + 0x12, 1) & 0x04u) != 0");
    write_register(state, &serial, uart_base + 0x12, 1, 0x04u);
    write_register(state, &serial, uart_base + 0x11, 1, 0x40u);
    expect(state, read_register(state, &serial, uart_base + 0x16, 1) == 0,
           "read_register(state, &serial, uart_base + 0x16, 1) == 0");

    write_register(state, &serial, uart_base + 0x10, 1, 0x88u);
    for (uint16_t receive_value = 0u; receive_value < 8u; receive_value++) {
        expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART1, receive_value, 0),
               "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_UART1, receive_value, 0)");
    }
    kinetis_serial_advance(&serial, 1280);
    expect(state, read_register(state, &serial, uart_base + 0x16, 1) == 1,
           "read_register(state, &serial, uart_base + 0x16, 1) == 1");

    KinetisSerial serial_copy;
    expect(state, kinetis_serial_copy(&serial_copy, &serial),
           "kinetis_serial_copy(&serial_copy, &serial)");
    expect(state, memcmp(&serial_copy, &serial, sizeof(serial)) == 0,
           "memcmp(&serial_copy, &serial, sizeof(serial)) == 0");
    kinetis_serial_reset(&serial_copy);
    expect(state, !serial_copy.uart[1].clock_enabled, "!serial_copy.uart[1].clock_enabled");
    expect(state, serial_copy.uart[1].receive.count == 0, "serial_copy.uart[1].receive.count == 0");
    expect(state, serial.uart[1].receive.count != 0, "serial.uart[1].receive.count != 0");
}

static void test_lpuart(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true)");
    expect(state, read_register(state, &serial, LPUART0_BASE, 4) == 0x0f000004u,
           "read_register(state, &serial, LPUART0_BASE, 4) == 0x0f000004u");
    expect(state, read_register(state, &serial, LPUART0_BASE + 4, 4) == 0x00c00000u,
           "read_register(state, &serial, LPUART0_BASE + 4, 4) == 0x00c00000u");
    write_register(state, &serial, LPUART0_BASE, 4, 0x0f200001u);
    write_register(state, &serial, LPUART0_BASE + 8, 4, (1u << 18) | (1u << 19) | (1u << 21));
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_LPUART0, 0x155u, 1),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_LPUART0, 0x155u, 1)");
    kinetis_serial_advance(&serial, 160);
    expect(state, (read_register(state, &serial, LPUART0_BASE + 4, 4) & 0x00210000u) == 0x00210000u,
           "(read_register(state, &serial, LPUART0_BASE + 4, 4) & 0x00210000u) == "
           "0x00210000u");
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_LPUART0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_LPUART0)");
    expect(state, kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_LPUART0_RECEIVE),
           "kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_LPUART0_RECEIVE)");
    expect(state, (read_register(state, &serial, LPUART0_BASE + 0x0c, 4) & 0x3ffu) == 0x155u,
           "(read_register(state, &serial, LPUART0_BASE + 0x0c, 4) & 0x3ffu) == 0x155u");
    write_register(state, &serial, LPUART0_BASE + 4, 4, 0x00010000u);
    expect(state, (read_register(state, &serial, LPUART0_BASE + 4, 4) & 0x00010000u) == 0,
           "(read_register(state, &serial, LPUART0_BASE + 4, 4) & 0x00010000u) == 0");
}

static void test_uart_frame_lengths(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);
    uint16_t transmitted_value = 0;

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true)");
    write_register(state, &serial, UART0_BASE + 1, 1, 1u);
    write_register(state, &serial, UART0_BASE + 2, 1, 1u << 4);
    write_register(state, &serial, UART0_BASE + 3, 1, 1u << 3);
    write_register(state, &serial, UART0_BASE + 7, 1, 0x11u);
    kinetis_serial_advance(&serial, 175u);
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "eleven-bit UART frame remains active before its final clock");
    kinetis_serial_advance(&serial, 1u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "eleven-bit UART frame completes on its final clock");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true)");
    write_register(state, &serial, LPUART0_BASE, 4, 0x0f002001u);
    write_register(state, &serial, LPUART0_BASE + 8, 4, 1u << 19);
    write_register(state, &serial, LPUART0_BASE + 0x0c, 4, 0x22u);
    kinetis_serial_advance(&serial, 175u);
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "two-stop-bit LPUART frame remains active before its final clock");
    kinetis_serial_advance(&serial, 1u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "two-stop-bit LPUART frame completes on its final clock");
}

static void test_spi_transfer_fifo_interrupt_dma_and_errors(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI0, true)");
    expect(state, read_register(state, &serial, SPI0_BASE, 4) == 0x00004001u,
           "read_register(state, &serial, SPI0_BASE, 4) == 0x00004001u");
    write_register(state, &serial, SPI0_BASE + 0x34, 4, 0x1234u);
    expect(state, serial.spi[0].transmit.count == 0, "serial.spi[0].transmit.count == 0");
    write_register(state, &serial, SPI0_BASE, 4, 1u);
    write_register(state, &serial, SPI0_BASE + 0x34, 4, 0x1234u);
    expect(state, serial.spi[0].transmit.count == 1, "HALT accepts data into the transmit FIFO");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x34, 4) == 0x1234u,
           "HALT preserves the accepted push value");
    write_register(state, &serial, SPI0_BASE, 4, 1u << 14);
    write_register(state, &serial, SPI0_BASE + 0x34, 4, 0x5678u);
    expect(state, serial.spi[0].transmit.count == 1, "MDIS ignores push writes");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x34, 4) == 0x1234u,
           "MDIS preserves the last accepted push value");
    write_register(state, &serial, SPI0_BASE, 4, 1u << 11);
    write_register(state, &serial, SPI0_BASE, 4, 0);
    write_register(state, &serial, SPI0_BASE + 0x30, 4, 3u << 16);
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, 0x12abu, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, 0x12abu, 0)");
    write_register(state, &serial, SPI0_BASE + 0x34, 1, 0x5au);
    kinetis_serial_advance(&serial, 64);
    expect(state, read_register(state, &serial, SPI0_BASE + 0x38, 1) == 0xabu,
           "read_register(state, &serial, SPI0_BASE + 0x38, 1) == 0xabu");
    uint16_t transmitted_value;
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_SPI0, &transmitted_value),
           "kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_SPI0, &transmitted_value)");
    expect(state, transmitted_value == 0x5au, "transmitted_value == 0x5au");
    write_register(state, &serial, SPI0_BASE, 4, UINT32_C(0x80000000));
    write_register(state, &serial, SPI0_BASE + 0x36, 2, 0x9803u);
    write_register(state, &serial, SPI0_BASE + 0x34, 1, 0x6au);
    write_register(state, &serial, SPI0_BASE + 0x34, 1, 0x6bu);
    expect(state, read_register(state, &serial, SPI0_BASE + 0x34, 4) == 0x9803006au,
           "read_register(state, &serial, SPI0_BASE + 0x34, 4) == 0x9803006au");
    kinetis_serial_advance(&serial, 1024);
    KinetisSerialSpiTransfer byte_transfer;
    expect(state, kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &byte_transfer),
           "kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &byte_transfer)");
    expect(state, byte_transfer.data == 0x6au, "byte_transfer.data == 0x6au");
    expect(state, byte_transfer.chip_selects == 3u, "byte_transfer.chip_selects == 3u");
    expect(state, byte_transfer.clock_and_transfer_attributes == 1u,
           "byte_transfer.clock_and_transfer_attributes == 1u");
    expect(state, byte_transfer.continuous_chip_select, "byte_transfer.continuous_chip_select");
    expect(state, byte_transfer.end_of_queue, "byte_transfer.end_of_queue");
    expect(state, kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &byte_transfer),
           "kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &byte_transfer)");
    expect(state, byte_transfer.data == 0x6bu, "byte_transfer.data == 0x6bu");
    expect(state, byte_transfer.chip_selects == 3u, "byte_transfer.chip_selects == 3u");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x34, 4) == 0x9803006bu,
           "read_register(state, &serial, SPI0_BASE + 0x34, 4) == 0x9803006bu");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0xffffu,
           "read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0xffffu");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0xffffu,
           "read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0xffffu");
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, 0x1234u, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, 0x1234u, 0)");
    write_register(state, &serial, SPI0_BASE + 0x34, 4, 0xabcdu);
    kinetis_serial_advance(&serial, 63);
    expect(state, (read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 17)) == 0,
           "(read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 17)) == 0");
    kinetis_serial_advance(&serial, 1);
    expect(state, kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_SPI0_RECEIVE),
           "kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_SPI0_RECEIVE)");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_SPI0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_SPI0)");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0x1234u,
           "read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0x1234u");
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_SPI0, &transmitted_value),
           "kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_SPI0, &transmitted_value)");
    expect(state, transmitted_value == 0xabcdu, "transmitted_value == 0xabcdu");

    write_register(state, &serial, SPI0_BASE + 0x30, 4, 1u << 17);
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, 0x5678u, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, 0x5678u, 0)");
    write_register(state, &serial, SPI0_BASE + 0x34, 4, 0x90030011u);
    kinetis_serial_advance(&serial, 64);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_SPI0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_SPI0)");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0x5678u,
           "read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0x5678u");
    KinetisSerialSpiTransfer spi_transfer;
    expect(state, kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &spi_transfer),
           "kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &spi_transfer)");
    expect(state, spi_transfer.data == 0x11u, "spi_transfer.data == 0x11u");
    expect(state, spi_transfer.chip_selects == 3u, "spi_transfer.chip_selects == 3u");
    expect(state, spi_transfer.clock_and_transfer_attributes == 1u,
           "spi_transfer.clock_and_transfer_attributes == 1u");
    expect(state, spi_transfer.continuous_chip_select, "spi_transfer.continuous_chip_select");
    expect(state, !spi_transfer.end_of_queue, "!spi_transfer.end_of_queue");
    expect(state, read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0x5678u,
           "read_register(state, &serial, SPI0_BASE + 0x38, 4) == 0x5678u");
    expect(state, (read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 19)) != 0,
           "(read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 19)) != 0");
    write_register(state, &serial, SPI0_BASE + 0x2c, 4, 1u << 19);
    expect(state, (read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 19)) == 0,
           "(read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 19)) == 0");

    write_register(state, &serial, SPI0_BASE, 4, 1u << 11);
    expect(state, serial.spi[0].transmit.count == 0, "serial.spi[0].transmit.count == 0");
    expect(state, (read_register(state, &serial, SPI0_BASE, 4) & (1u << 11)) == 0,
           "(read_register(state, &serial, SPI0_BASE, 4) & (1u << 11)) == 0");
}

static void test_spi_profile_presence(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN1M012);
    uint32_t register_value;

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI1, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI1, true)");
    expect(state, kinetis_serial_read(&serial, SPI1_BASE, 4, &register_value),
           "kinetis_serial_read(&serial, SPI1_BASE, 4, &register_value)");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI2, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI2, true)");
    expect(state, kinetis_serial_read(&serial, 0x400ac000u, 4, &register_value),
           "kinetis_serial_read(&serial, 0x400ac000u, 4, &register_value)");
}

static void test_i2c_master_events_timing_irq_and_dma(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true)");
    write_register(state, &serial, I2C0_BASE + 2, 1, 0xf1u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_START, 0);
    write_register(state, &serial, I2C0_BASE + 4, 1, 0xa4u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_WRITE, 0xa4u);
    kinetis_serial_advance(&serial, 179);
    expect(state, (read_register(state, &serial, I2C0_BASE + 3, 1) & 2u) == 0,
           "(read_register(state, &serial, I2C0_BASE + 3, 1) & 2u) == 0");
    kinetis_serial_advance(&serial, 1);
    expect(state, (read_register(state, &serial, I2C0_BASE + 3, 1) & 0x83u) == 0x82u,
           "(read_register(state, &serial, I2C0_BASE + 3, 1) & 0x83u) == 0x82u");
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0)");
    expect(state, kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_I2C0),
           "kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_I2C0)");
    write_register(state, &serial, I2C0_BASE + 3, 1, 2u);
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0)");

    expect(state, kinetis_serial_i2c_set_acknowledge(&serial, KINETIS_SERIAL_I2C0, false),
           "kinetis_serial_i2c_set_acknowledge(&serial, KINETIS_SERIAL_I2C0, false)");
    write_register(state, &serial, I2C0_BASE + 4, 1, 0xa5u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_WRITE, 0xa5u);
    kinetis_serial_advance(&serial, 180);
    expect(state, (read_register(state, &serial, I2C0_BASE + 3, 1) & 1u) != 0,
           "(read_register(state, &serial, I2C0_BASE + 3, 1) & 1u) != 0");
    write_register(state, &serial, I2C0_BASE + 2, 1, 0xf5u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_REPEATED_START, 0);
    write_register(state, &serial, I2C0_BASE + 2, 1, 0xe1u);
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C0, 0x5au, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C0, 0x5au, 0)");
    read_register(state, &serial, I2C0_BASE + 4, 1);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_READ, 0);
    kinetis_serial_advance(&serial, 180);
    expect(state, read_register(state, &serial, I2C0_BASE + 4, 1) == 0x5au,
           "read_register(state, &serial, I2C0_BASE + 4, 1) == 0x5au");
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_READ, 0);
    write_register(state, &serial, I2C0_BASE + 2, 1, 0xc1u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_STOP, 0);
    write_register(state, &serial, I2C0_BASE + 2, 1, 0xe1u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_START, 0);
    write_register(state, &serial, I2C0_BASE + 2, 1, 0u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_STOP, 0);
}

static void test_clock_domains(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN1M012);
    kinetis_serial_set_clocks(&serial, 120000000u, 60000000u, 30000000u);
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true),
           "system-clocked UART gate is available");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART2, true),
           "bus-clocked UART gate is available");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI0, true),
           "SPI gate is available");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true),
           "I2C gate is available");

    for (size_t uart_index = 0u; uart_index <= 2u; uart_index += 2u) {
        const uint32_t uart_base = serial.uart[uart_index].base;
        write_register(state, &serial, uart_base, 1u, 0u);
        write_register(state, &serial, uart_base + 1u, 1u, 1u);
        write_register(state, &serial, uart_base + 3u, 1u, 0x08u);
        write_register(state, &serial, uart_base + 7u, 1u, (uint32_t)(0x40u + uart_index));
    }
    write_register(state, &serial, SPI0_BASE, 4u, UINT32_C(0x80000000));
    write_register(state, &serial, SPI0_BASE + 0x34u, 4u, 0x98030055u);
    write_register(state, &serial, I2C0_BASE + 2u, 1u, 0xf0u);
    write_register(state, &serial, I2C0_BASE + 4u, 1u, 0x52u);

    uint16_t transmitted_value = 0u;
    KinetisSerialSpiTransfer spi_transfer;
    kinetis_serial_advance(&serial, 127u);
    expect(state, !kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &spi_transfer),
           "SPI waits for 64 bus clocks");
    kinetis_serial_advance(&serial, 1u);
    expect(state, kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &spi_transfer),
           "SPI completes after 128 core clocks");
    kinetis_serial_advance(&serial, 31u);
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "system-clocked UART waits for its final core clock");
    kinetis_serial_advance(&serial, 1u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "system-clocked UART advances at the core rate");
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART2, &transmitted_value),
           "bus-clocked UART remains in progress");
    kinetis_serial_advance(&serial, 159u);
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART2, &transmitted_value),
           "bus-clocked UART waits for its final bus clock");
    expect(state, (read_register(state, &serial, I2C0_BASE + 3u, 1u) & 2u) == 0u,
           "I2C waits for its final bus clock");
    kinetis_serial_advance(&serial, 1u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART2, &transmitted_value),
           "bus-clocked UART advances at the bus rate");
    kinetis_serial_advance(&serial, 40u);
    expect(state, (read_register(state, &serial, I2C0_BASE + 3u, 1u) & 2u) != 0u,
           "I2C completes after 180 bus clocks");

    write_register(state, &serial, serial.uart[0].base + 7u, 1u, 0x60u);
    write_register(state, &serial, serial.uart[2].base + 7u, 1u, 0x62u);
    kinetis_serial_set_clock_domains(&serial, false, false);
    kinetis_serial_advance(&serial, 320u);
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "system-clocked UART stops with its clock domain");
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART2, &transmitted_value),
           "bus-clocked UART stops with its clock domain");
    kinetis_serial_set_clock_domains(&serial, true, false);
    kinetis_serial_advance(&serial, 160u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, &transmitted_value),
           "system-clocked UART resumes with its clock domain");
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART2, &transmitted_value),
           "bus-clocked UART remains stopped independently");
    kinetis_serial_set_clock_domains(&serial, true, true);
    kinetis_serial_advance(&serial, 320u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART2, &transmitted_value),
           "bus-clocked UART resumes with its clock domain");
    kinetis_serial_set_clock_domains(NULL, false, false);
}

static void test_lpuart_clock_domain(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);
    uint16_t transmitted_value = 0u;

    kinetis_serial_set_clocks(&serial, 120000000u, 60000000u, 0u);
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true),
           "LPUART clock gate is available");
    write_register(state, &serial, LPUART0_BASE, 4u, 0x0f000001u);
    write_register(state, &serial, LPUART0_BASE + 8u, 4u, 1u << 19);
    write_register(state, &serial, LPUART0_BASE + 0x0cu, 4u, 0x5au);
    kinetis_serial_advance(&serial, 120000000u);
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "disabled LPUART source does not advance");

    kinetis_serial_set_clocks(&serial, 120000000u, 60000000u, 30000000u);
    kinetis_serial_advance(&serial, 639u);
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "LPUART waits for its final source clock");
    kinetis_serial_advance(&serial, 1u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "LPUART advances at its selected source rate");
    write_register(state, &serial, LPUART0_BASE + 0x0cu, 4u, 0x6au);
    kinetis_serial_set_clock_domains(&serial, false, false);
    kinetis_serial_advance(&serial, 640u);
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "LPUART uses its independent clock in Stop");
}

static void test_i2c_start_stop_detection(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true)");
    write_register(state, &serial, I2C0_BASE + 2u, 1u, 0xc0u);
    write_register(state, &serial, I2C0_BASE + 6u, 1u, 0x2au);
    expect(state, read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x2au,
           "read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x2au");

    write_register(state, &serial, I2C0_BASE + 2u, 1u, 0xe0u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_START, 0);
    expect(state, read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x3au,
           "read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x3au");
    expect(state, (read_register(state, &serial, I2C0_BASE + 3u, 1u) & 2u) != 0u,
           "start detection latches IICIF");
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0)");
    write_register(state, &serial, I2C0_BASE + 3u, 1u, 2u);
    expect(state, (read_register(state, &serial, I2C0_BASE + 3u, 1u) & 2u) != 0u,
           "IICIF reasserts until STARTF is cleared");
    write_register(state, &serial, I2C0_BASE + 6u, 1u, 0x3au);
    write_register(state, &serial, I2C0_BASE + 3u, 1u, 2u);
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "clearing STARTF before IICIF clears the interrupt");
    write_register(state, &serial, I2C0_BASE + 2u, 1u, 0xc0u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C0, KINETIS_SERIAL_EVENT_I2C_STOP, 0);
    expect(state, read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x6au,
           "master stop latches STOPF");
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "master stop requests an enabled detection interrupt");
    write_register(state, &serial, I2C0_BASE + 6u, 1u, 0x6au);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "clearing master STOPF leaves IICIF pending");
    write_register(state, &serial, I2C0_BASE + 3u, 1u, 2u);
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "clearing master IICIF removes the interrupt");

    expect(state, kinetis_serial_i2c_detect_start(&serial, KINETIS_SERIAL_I2C0),
           "kinetis_serial_i2c_detect_start(&serial, KINETIS_SERIAL_I2C0)");
    expect(state, read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x3au,
           "read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x3au");
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0)");
    write_register(state, &serial, I2C0_BASE + 3u, 1u, 2u);
    expect(state, (read_register(state, &serial, I2C0_BASE + 3u, 1u) & 2u) != 0u,
           "IICIF reasserts while STARTF remains set");
    write_register(state, &serial, I2C0_BASE + 6u, 1u, 0x3au);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "clearing STARTF leaves IICIF pending");
    write_register(state, &serial, I2C0_BASE + 3u, 1u, 2u);
    expect(state, read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x2au,
           "read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x2au");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0)");

    expect(state, kinetis_serial_i2c_detect_stop(&serial, KINETIS_SERIAL_I2C0),
           "kinetis_serial_i2c_detect_stop(&serial, KINETIS_SERIAL_I2C0)");
    expect(state, read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x6au,
           "read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x6au");
    expect(state, (read_register(state, &serial, I2C0_BASE + 3u, 1u) & 2u) != 0u,
           "stop detection latches IICIF");
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0)");
    write_register(state, &serial, I2C0_BASE + 3u, 1u, 2u);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "IICIF reasserts while STOPF remains set");
    write_register(state, &serial, I2C0_BASE + 6u, 1u, 0x6au);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "clearing STOPF leaves IICIF pending");
    write_register(state, &serial, I2C0_BASE + 3u, 1u, 2u);
    expect(state, read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x2au,
           "read_register(state, &serial, I2C0_BASE + 6u, 1u) == 0x2au");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_I2C0)");
}

static void test_i2c_arbitration_disable_slave_and_reset(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN1M012);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C2, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C2, true)");
    write_register(state, &serial, I2C2_BASE, 1, 0x52u);
    write_register(state, &serial, I2C2_BASE + 2, 1, 0xc0u);
    expect(state, kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C2, 0x29u, true),
           "kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C2, 0x29u, true)");
    expect(state, (read_register(state, &serial, I2C2_BASE + 3, 1) & 0x46u) == 0x46u,
           "(read_register(state, &serial, I2C2_BASE + 3, 1) & 0x46u) == 0x46u");
    expect(state, !kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C2, 0x2au, false),
           "!kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C2, 0x2au, false)");
    write_register(state, &serial, I2C2_BASE + 4, 1, 0x9au);
    uint16_t slave_transmitted_value;
    expect(state,
           kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_I2C2, &slave_transmitted_value),
           "kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_I2C2, &slave_transmitted_value)");
    expect(state, slave_transmitted_value == 0x9au, "slave_transmitted_value == 0x9au");

    expect(state, kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C2, 0x29u, false),
           "kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C2, 0x29u, false)");
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C2, 0x6bu, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C2, 0x6bu, 0)");
    expect(state, read_register(state, &serial, I2C2_BASE + 4, 1) == 0x6bu,
           "read_register(state, &serial, I2C2_BASE + 4, 1) == 0x6bu");

    write_register(state, &serial, I2C2_BASE + 2, 1, 0xf0u);
    expect_event(state, &serial, KINETIS_SERIAL_I2C2, KINETIS_SERIAL_EVENT_I2C_START, 0);
    expect(state, kinetis_serial_i2c_lose_arbitration(&serial, KINETIS_SERIAL_I2C2),
           "kinetis_serial_i2c_lose_arbitration(&serial, KINETIS_SERIAL_I2C2)");
    expect(state, (read_register(state, &serial, I2C2_BASE + 3, 1) & 0x12u) == 0x12u,
           "(read_register(state, &serial, I2C2_BASE + 3, 1) & 0x12u) == 0x12u");
    expect(state, (read_register(state, &serial, I2C2_BASE + 2, 1) & 0x20u) == 0,
           "(read_register(state, &serial, I2C2_BASE + 2, 1) & 0x20u) == 0");
    write_register(state, &serial, I2C2_BASE + 3, 1, 0x12u);
    expect(state, (read_register(state, &serial, I2C2_BASE + 3, 1) & 0x12u) == 0,
           "(read_register(state, &serial, I2C2_BASE + 3, 1) & 0x12u) == 0");
    write_register(state, &serial, I2C2_BASE + 2, 1, 0);
    expect(state, read_register(state, &serial, I2C2_BASE + 3, 1) == 0,
           "read_register(state, &serial, I2C2_BASE + 3, 1) == 0");
    kinetis_serial_reset(&serial);
    expect(state, serial.i2c[2].acknowledge, "serial.i2c[2].acknowledge");
    expect(state, !serial.i2c[2].clock_enabled, "!serial.i2c[2].clock_enabled");
}

static void test_register_edge_paths(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);
    kinetis_serial_set_clocks(&serial, 96000000u, 48000000u, 96000000u);
    expect(state, serial.core_clock_hz == 96000000u, "serial.core_clock_hz == 96000000u");
    expect(state, serial.bus_clock_hz == 48000000u, "serial.bus_clock_hz == 48000000u");
    expect(state, serial.lpuart_clock_hz == 96000000u, "serial.lpuart_clock_hz == 96000000u");
    kinetis_serial_set_clocks(NULL, 0, 0, 0);
    kinetis_serial_set_clocks(&serial, 96000000u, 96000000u, 96000000u);
    kinetis_serial_reset(NULL);
    kinetis_serial_advance(NULL, 1);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true)");
    expect(state, read_register(state, &serial, UART0_BASE + 7, 1) == 0,
           "read_register(state, &serial, UART0_BASE + 7, 1) == 0");
    write_register(state, &serial, UART0_BASE + 4, 1, 0x1fu);
    write_register(state, &serial, UART0_BASE + 5, 1, 0xffu);
    write_register(state, &serial, UART0_BASE + 0x10, 1, 0x88u);
    for (uint16_t uart_write_value = 0u; uart_write_value < 9u; uart_write_value++) {
        write_register(state, &serial, UART0_BASE + 7, 1, uart_write_value);
    }
    expect(state, (read_register(state, &serial, UART0_BASE + 0x12, 1) & 0x02u) != 0,
           "(read_register(state, &serial, UART0_BASE + 0x12, 1) & 0x02u) != 0");
    write_register(state, &serial, UART0_BASE + 0x11, 1, 0x80u);
    expect(state, read_register(state, &serial, UART0_BASE + 0x14, 1) == 0,
           "read_register(state, &serial, UART0_BASE + 0x14, 1) == 0");
    uint32_t ignored_value;
    expect(state, !kinetis_serial_read(&serial, UART0_BASE, 4, &ignored_value),
           "!kinetis_serial_read(&serial, UART0_BASE, 4, &ignored_value)");
    expect(state, !kinetis_serial_write(&serial, UART0_BASE, 2, 0),
           "!kinetis_serial_write(&serial, UART0_BASE, 2, 0)");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true)");
    expect(state, read_register(state, &serial, LPUART0_BASE + 0x0c, 4) == 0x1000u,
           "read_register(state, &serial, LPUART0_BASE + 0x0c, 4) == 0x1000u");
    write_register(state, &serial, LPUART0_BASE, 4, 0x0f000001u);
    write_register(state, &serial, LPUART0_BASE + 8, 4, 1u << 19);
    write_register(state, &serial, LPUART0_BASE + 0x0c, 4, 0x11u);
    write_register(state, &serial, LPUART0_BASE + 0x0c, 4, 0x22u);
    expect(state, (read_register(state, &serial, LPUART0_BASE + 4, 4) & (1u << 19)) != 0,
           "(read_register(state, &serial, LPUART0_BASE + 4, 4) & (1u << 19)) != 0");
    kinetis_serial_advance(&serial, 160);
    uint16_t transmitted_value;
    expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_LPUART0, &transmitted_value)");
    expect(state, transmitted_value == 0x11u, "transmitted_value == 0x11u");
    expect(state, !kinetis_serial_read(&serial, LPUART0_BASE + 1, 1, &ignored_value),
           "!kinetis_serial_read(&serial, LPUART0_BASE + 1, 1, &ignored_value)");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI0, true)");
    write_register(state, &serial, SPI0_BASE, 4, 0);
    for (uint32_t spi_write_value = 0u; spi_write_value < 5u; spi_write_value++) {
        write_register(state, &serial, SPI0_BASE + 0x34, 4, spi_write_value);
    }
    expect(state, (read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 27)) != 0,
           "(read_register(state, &serial, SPI0_BASE + 0x2c, 4) & (1u << 27)) != 0");
    write_register(state, &serial, SPI0_BASE, 4, (1u << 10) | (1u << 11));
    expect(state, serial.spi[0].receive.count == 0, "serial.spi[0].receive.count == 0");
    expect(state, serial.spi[0].transmit.count == 0, "serial.spi[0].transmit.count == 0");
    write_register(state, &serial, SPI0_BASE + 0x0c, 4, 0xb8000000u);
    write_register(state, &serial, SPI0_BASE, 4, UINT32_C(0x80000000));
    for (uint16_t receive_value = 0u; receive_value < 5u; receive_value++) {
        expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, receive_value, 0),
               "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_SPI0, receive_value, 0)");
        write_register(state, &serial, SPI0_BASE + 0x34, 4,
                       receive_value == 4u ? (1u << 27) | receive_value : receive_value);
        kinetis_serial_advance(&serial, 16);
    }
    uint32_t spi_status = read_register(state, &serial, SPI0_BASE + 0x2c, 4);
    expect(state, (spi_status & (1u << 19)) != 0, "(spi_status & (1u << 19)) != 0");
    expect(state, (spi_status & (1u << 28)) != 0, "(spi_status & (1u << 28)) != 0");
    for (size_t transmit_index = 0u; transmit_index < 4u; transmit_index++) {
        expect(state, kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_SPI0, &transmitted_value),
               "kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_SPI0, &transmitted_value)");
    }
    KinetisSerialSpiTransfer spi_transfer;
    expect(state, kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &spi_transfer),
           "kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, &spi_transfer)");
    expect(state, spi_transfer.data == 4, "spi_transfer.data == 4");
    expect(state, spi_transfer.chip_selects == 0, "spi_transfer.chip_selects == 0");
    expect(state, spi_transfer.clock_and_transfer_attributes == 0,
           "spi_transfer.clock_and_transfer_attributes == 0");
    expect(state, !spi_transfer.continuous_chip_select, "!spi_transfer.continuous_chip_select");
    expect(state, spi_transfer.end_of_queue, "spi_transfer.end_of_queue");
    expect(state, !kinetis_serial_read(&serial, SPI0_BASE, 1, &ignored_value),
           "!kinetis_serial_read(&serial, SPI0_BASE, 1, &ignored_value)");
    expect(state, !kinetis_serial_write(&serial, SPI0_BASE + 2, 4, 0),
           "!kinetis_serial_write(&serial, SPI0_BASE + 2, 4, 0)");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true)");
    write_register(state, &serial, I2C0_BASE + 6, 1, 0xffu);
    expect(state, read_register(state, &serial, I2C0_BASE + 6, 1) == 0xafu,
           "read_register(state, &serial, I2C0_BASE + 6, 1) == 0xafu");
    expect(state, !kinetis_serial_read(&serial, I2C0_BASE, 2, &ignored_value),
           "!kinetis_serial_read(&serial, I2C0_BASE, 2, &ignored_value)");
    expect(state, !kinetis_serial_write(&serial, I2C0_BASE, 4, 0),
           "!kinetis_serial_write(&serial, I2C0_BASE, 4, 0)");
    expect(state, !kinetis_serial_i2c_set_acknowledge(&serial, KINETIS_SERIAL_UART0, true),
           "!kinetis_serial_i2c_set_acknowledge(&serial, KINETIS_SERIAL_UART0, true)");
    expect(state, !kinetis_serial_i2c_lose_arbitration(&serial, KINETIS_SERIAL_I2C1),
           "!kinetis_serial_i2c_lose_arbitration(&serial, KINETIS_SERIAL_I2C1)");
    expect(state, !kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C1, 0, false),
           "!kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C1, 0, false)");
}

static void test_event_capacity(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true)");
    for (size_t event_index = 0u; event_index < 33u; event_index++) {
        write_register(state, &serial, I2C0_BASE + 2, 1, 0xf0u);
        write_register(state, &serial, I2C0_BASE + 2, 1, 0xd0u);
    }
    expect(state, serial.event_count == KINETIS_SERIAL_EVENT_CAPACITY,
           "serial.event_count == KINETIS_SERIAL_EVENT_CAPACITY");
    KinetisSerialEvent serial_event;
    expect(state, kinetis_serial_pop_event(&serial, &serial_event),
           "kinetis_serial_pop_event(&serial, &serial_event)");
    expect(state, serial_event.type == KINETIS_SERIAL_EVENT_I2C_START,
           "serial_event.type == KINETIS_SERIAL_EVENT_I2C_START");
}

static void test_signal_and_overflow_matrix(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);
    expect(state, !kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_ADC0, true),
           "!kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_ADC0, true)");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0)");
    expect(state, !kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_TRANSMIT),
           "!kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_TRANSMIT)");
    expect(state, !kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_LPUART0_TRANSMIT),
           "!kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_LPUART0_TRANSMIT)");
    expect(state, !kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_SPI0_TRANSMIT),
           "!kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_SPI0_TRANSMIT)");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_UART0, true)");
    write_register(state, &serial, UART0_BASE + 0x0b, 1, 0x80u);
    expect(state, !kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_TRANSMIT),
           "!kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_TRANSMIT)");
    write_register(state, &serial, UART0_BASE + 3, 1, 0x80u);
    expect(state, kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_TRANSMIT),
           "kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_UART0_TRANSMIT)");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_UART0)");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_LPUART0, true)");
    write_register(state, &serial, LPUART0_BASE, 4, 0x0f800001u);
    expect(state, kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_LPUART0_TRANSMIT),
           "kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_LPUART0_TRANSMIT)");
    write_register(state, &serial, LPUART0_BASE + 8, 4, (1u << 18) | (1u << 27));
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_LPUART0, 1, 8),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_LPUART0, 1, 8)");
    expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_LPUART0, 2, 0),
           "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_LPUART0, 2, 0)");
    kinetis_serial_advance(&serial, 320);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_LPUART0),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_LPUART0)");
    expect(state, (read_register(state, &serial, LPUART0_BASE + 4, 4) & (1u << 19)) != 0,
           "(read_register(state, &serial, LPUART0_BASE + 4, 4) & (1u << 19)) != 0");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI0, true)");
    write_register(state, &serial, SPI0_BASE + 0x30, 4, 3u << 24);
    expect(state, kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_SPI0_TRANSMIT),
           "kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_SPI0_TRANSMIT)");
    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI1, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_SPI1, true)");
    write_register(state, &serial, SPI1_BASE + 0x30, 4, 1u << 25);
    expect(state, kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_SPI1),
           "kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_SPI1)");

    expect(state, kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true),
           "kinetis_serial_set_clock_gate(&serial, KINETIS_PERIPHERAL_I2C0, true)");
    write_register(state, &serial, I2C0_BASE, 1, 0x52u);
    write_register(state, &serial, I2C0_BASE + 2, 1, 0x80u);
    expect(state, kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C0, 0x29u, false),
           "kinetis_serial_i2c_slave_address(&serial, KINETIS_SERIAL_I2C0, 0x29u, false)");
    for (uint16_t receive_value = 0u; receive_value < KINETIS_SERIAL_FIFO_CAPACITY;
         receive_value++) {
        expect(state, kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C0, receive_value, 0),
               "kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C0, receive_value, 0)");
    }
    expect(state, !kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C0, 0xffu, 0),
           "!kinetis_serial_push_receive(&serial, KINETIS_SERIAL_I2C0, 0xffu, 0)");

    uint32_t ignored_value;
    expect(state, !kinetis_serial_write(&serial, 0x50000000u, 1, 0),
           "!kinetis_serial_write(&serial, 0x50000000u, 1, 0)");
    expect(state, !kinetis_serial_read(&serial, 0x50000000u, 1, &ignored_value),
           "!kinetis_serial_read(&serial, 0x50000000u, 1, &ignored_value)");
}

static void test_invalid_operations(TestState* state) {
    KinetisSerial serial = create_serial(state, KINETIS_PROFILE_MK22FN51212);
    uint32_t register_value;
    uint16_t transmitted_value;

    expect(state, !kinetis_serial_init(NULL, serial.profile),
           "!kinetis_serial_init(NULL, serial.profile)");
    expect(state, !kinetis_serial_init(&serial, NULL), "!kinetis_serial_init(&serial, NULL)");
    expect(state, !kinetis_serial_copy(NULL, &serial), "!kinetis_serial_copy(NULL, &serial)");
    expect(state, !kinetis_serial_copy(&serial, NULL), "!kinetis_serial_copy(&serial, NULL)");
    expect(state, !kinetis_serial_read(NULL, UART0_BASE, 1, &register_value),
           "!kinetis_serial_read(NULL, UART0_BASE, 1, &register_value)");
    expect(state, !kinetis_serial_read(&serial, UART0_BASE, 1, NULL),
           "!kinetis_serial_read(&serial, UART0_BASE, 1, NULL)");
    expect(state, !kinetis_serial_write(NULL, UART0_BASE, 1, 0),
           "!kinetis_serial_write(NULL, UART0_BASE, 1, 0)");
    expect(state, !kinetis_serial_read(&serial, 0x50000000u, 1, &register_value),
           "!kinetis_serial_read(&serial, 0x50000000u, 1, &register_value)");
    expect(state, !kinetis_serial_push_receive(NULL, KINETIS_SERIAL_UART0, 0u, 0u),
           "!kinetis_serial_push_receive(NULL, KINETIS_SERIAL_UART0, 0, 0)");
    expect(state, !kinetis_serial_push_receive(&serial, KINETIS_SERIAL_ENDPOINT_INVALID, 0u, 0u),
           "!kinetis_serial_push_receive(&serial, KINETIS_SERIAL_ENDPOINT_INVALID, 0, 0)");
    expect(state, !kinetis_serial_pop_transmit(NULL, KINETIS_SERIAL_UART0, &transmitted_value),
           "!kinetis_serial_pop_transmit(NULL, KINETIS_SERIAL_UART0, &transmitted_value)");
    expect(state, !kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, NULL),
           "!kinetis_serial_pop_transmit(&serial, KINETIS_SERIAL_UART0, NULL)");
    KinetisSerialSpiTransfer spi_transfer;
    expect(state, !kinetis_serial_pop_spi_transfer(NULL, KINETIS_SERIAL_SPI0, &spi_transfer),
           "!kinetis_serial_pop_spi_transfer(NULL, KINETIS_SERIAL_SPI0, &spi_transfer)");
    expect(state, !kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, NULL),
           "!kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_SPI0, NULL)");
    expect(state, !kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_UART0, &spi_transfer),
           "!kinetis_serial_pop_spi_transfer(&serial, KINETIS_SERIAL_UART0, &spi_transfer)");
    expect(state, !kinetis_serial_pop_event(&serial, NULL),
           "!kinetis_serial_pop_event(&serial, NULL)");
    KinetisSerialEvent serial_event;
    expect(state, !kinetis_serial_pop_event(&serial, &serial_event),
           "!kinetis_serial_pop_event(&serial, &serial_event)");
    expect(state, !kinetis_serial_pop_event(NULL, &serial_event),
           "!kinetis_serial_pop_event(NULL, &serial_event)");
    expect(state, !kinetis_serial_i2c_set_acknowledge(NULL, KINETIS_SERIAL_I2C0, true),
           "!kinetis_serial_i2c_set_acknowledge(NULL, KINETIS_SERIAL_I2C0, true)");
    expect(state, !kinetis_serial_i2c_detect_start(NULL, KINETIS_SERIAL_I2C0),
           "!kinetis_serial_i2c_detect_start(NULL, KINETIS_SERIAL_I2C0)");
    expect(state, !kinetis_serial_i2c_detect_stop(NULL, KINETIS_SERIAL_I2C0),
           "!kinetis_serial_i2c_detect_stop(NULL, KINETIS_SERIAL_I2C0)");
    expect(state, !kinetis_serial_i2c_lose_arbitration(NULL, KINETIS_SERIAL_I2C0),
           "!kinetis_serial_i2c_lose_arbitration(NULL, KINETIS_SERIAL_I2C0)");
    expect(state, !kinetis_serial_i2c_slave_address(NULL, KINETIS_SERIAL_I2C0, 0, false),
           "!kinetis_serial_i2c_slave_address(NULL, KINETIS_SERIAL_I2C0, 0, false)");
    expect(state, !kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_COUNT),
           "!kinetis_serial_irq(&serial, KINETIS_SERIAL_IRQ_COUNT)");
    expect(state, !kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_COUNT),
           "!kinetis_serial_dma_request(&serial, KINETIS_SERIAL_DMA_COUNT)");
    expect(state, !kinetis_serial_irq(NULL, KINETIS_SERIAL_IRQ_UART0),
           "!kinetis_serial_irq(NULL, KINETIS_SERIAL_IRQ_UART0)");
    expect(state, !kinetis_serial_dma_request(NULL, KINETIS_SERIAL_DMA_UART0_RECEIVE),
           "!kinetis_serial_dma_request(NULL, KINETIS_SERIAL_DMA_UART0_RECEIVE)");

    KinetisSerial unavailable_serial = {0};
    kinetis_serial_advance_endpoint(NULL, KINETIS_SERIAL_UART0);
    kinetis_serial_advance_endpoint(&unavailable_serial, KINETIS_SERIAL_I2C0);
    expect(state,
           !kinetis_serial_push_receive(&unavailable_serial, KINETIS_SERIAL_UART0, 0u, 0u) &&
               !kinetis_serial_push_receive(&unavailable_serial, KINETIS_SERIAL_SPI0, 0u, 0u) &&
               !kinetis_serial_push_receive(&unavailable_serial, KINETIS_SERIAL_I2C0, 0u, 0u),
           "unavailable_serial receivers reject data");
    expect(state,
           !kinetis_serial_pop_transmit(&unavailable_serial, KINETIS_SERIAL_UART0,
                                        &transmitted_value) &&
               !kinetis_serial_pop_transmit(&unavailable_serial, KINETIS_SERIAL_SPI0,
                                            &transmitted_value) &&
               !kinetis_serial_pop_transmit(&unavailable_serial, KINETIS_SERIAL_I2C0,
                                            &transmitted_value),
           "unavailable_serial transmitters have no data");
    expect(
        state,
        !kinetis_serial_pop_spi_transfer(&unavailable_serial, KINETIS_SERIAL_SPI0, &spi_transfer),
        "unavailable_serial SPI has no transfers");
    expect(
        state,
        !kinetis_serial_i2c_set_acknowledge(&unavailable_serial, KINETIS_SERIAL_I2C0, true) &&
            !kinetis_serial_i2c_detect_start(&unavailable_serial, KINETIS_SERIAL_I2C0) &&
            !kinetis_serial_i2c_detect_stop(&unavailable_serial, KINETIS_SERIAL_I2C0) &&
            !kinetis_serial_i2c_lose_arbitration(&unavailable_serial, KINETIS_SERIAL_I2C0) &&
            !kinetis_serial_i2c_slave_address(&unavailable_serial, KINETIS_SERIAL_I2C0, 0u, false),
        "unavailable_serial I2C rejects bus events");
}

int main(void) {
    TestState state = {0};
    test_profiles_and_gates(&state);
    test_uart_transfer_status_interrupt_and_dma(&state);
    test_uart_fifo_overrun_flush_and_copy(&state);
    test_lpuart(&state);
    test_uart_frame_lengths(&state);
    test_spi_transfer_fifo_interrupt_dma_and_errors(&state);
    test_spi_profile_presence(&state);
    test_i2c_master_events_timing_irq_and_dma(&state);
    test_clock_domains(&state);
    test_lpuart_clock_domain(&state);
    test_i2c_start_stop_detection(&state);
    test_i2c_arbitration_disable_slave_and_reset(&state);
    test_register_edge_paths(&state);
    test_event_capacity(&state);
    test_signal_and_overflow_matrix(&state);
    test_invalid_operations(&state);
    return test_finish(&state);
}
