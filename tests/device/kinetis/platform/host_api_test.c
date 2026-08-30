#include "kinetis.h"

#include <stdint.h>

#include "allocation_failure.h"
#include "device/kinetis/internal.h"
#include "test.h"

enum {
    USB0 = 0x40072000u,
    CAN0 = 0x40024000u,
    I2S0 = 0x4002f000u,
    SPI0 = 0x4002c000u,
    I2C0 = 0x40066000u,
    I2C1 = 0x40067000u,
    UART0 = 0x4006a000u,
    PMC_REGSC = 0x4007d002u,
    RCM_SRS0 = 0x4007f000u,
    RCM_SRS1 = 0x4007f001u,
};

static Kinetis* create_device(TestState* test_state) {
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    configuration.profile = KINETIS_PROFILE_MK22FN1M012;
    configuration.package = KINETIS_PACKAGE_LQ_144_LQFP;
    configuration.flash_size = 4096u;
    configuration.sram_size = 65536u;
    Kinetis* device = kinetis_create(configuration);
    expect(test_state, device != NULL, "device != NULL");
    const uint32_t vectors[2] = {0x20001000u, 0x101u};
    expect(test_state, kinetis_load(device, 0u, vectors, sizeof(vectors)),
           "kinetis_load(device, 0u, vectors, sizeof(vectors))");
    expect(test_state, kinetis_reset(device), "kinetis_reset(device)");
    return device;
}

static void io_write(TestState* test_state, Kinetis* device, uint32_t register_address,
                     uint8_t access_size, uint32_t write_value) {
    expect(test_state, kinetis_io_write(&device->io, register_address, access_size, write_value),
           "kinetis_io_write(&device->io, register_address, access_size, write_value)");
}

static void serial_write(TestState* test_state, Kinetis* device, uint32_t register_address,
                         uint8_t access_size, uint32_t write_value) {
    expect(test_state,
           kinetis_serial_write(&device->serial, register_address, access_size, write_value),
           "kinetis_serial_write(&device->serial, register_address, access_size, write_value)");
}

static void test_data_api(TestState* state, Kinetis* device) {
    const uint32_t seed_value = 0x12345678u;
    uint32_t loaded_value = 0u;
    expect(state, kinetis_seed(device, 0x20000000u, &seed_value, sizeof(seed_value)),
           "kinetis_seed(device, 0x20000000u, &seed_value, sizeof(seed_value))");
    expect(state,
           kinetis_read(device, 0x20000000u, &loaded_value, sizeof(loaded_value)) &&
               loaded_value == seed_value,
           "kinetis_read(device, 0x20000000u, &loaded_value, sizeof(loaded_value)) && "
           "loaded_value == seed_value");
    expect(state, !kinetis_seed(NULL, 0u, &seed_value, sizeof(seed_value)),
           "!kinetis_seed(NULL, 0u, &seed_value, sizeof(seed_value))");

    expect(state, kinetis_set_reset_state(device, 0x91u, true),
           "kinetis_set_reset_state(device, 0x91u, true)");
    uint8_t reset_status = 0u;
    expect(state, kinetis_read(device, RCM_SRS0, &reset_status, 1u) && reset_status == 0x81u,
           "kinetis_read(device, RCM_SRS0, &reset_status, 1u) && reset_status == 0x81u");
    expect(state, kinetis_read(device, RCM_SRS1, &reset_status, 1u) && reset_status == 0u,
           "kinetis_read(device, RCM_SRS1, &reset_status, 1u) && reset_status == 0u");
    expect(state, kinetis_read(device, PMC_REGSC, &reset_status, 1u) && (reset_status & 8u) != 0u,
           "kinetis_read(device, PMC_REGSC, &reset_status, 1u) && (reset_status & 8u) != 0u");
    expect(state, !kinetis_set_reset_state(NULL, 0u, false),
           "!kinetis_set_reset_state(NULL, 0u, false)");

    expect(state, kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_A, 30u, 0x1234u),
           "kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_A, 30u, 0x1234u)");
    expect(state, !kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_A, 31u, 0u),
           "!kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_A, 31u, 0u)");
    expect(state, !kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_B, 8u, 0u),
           "!kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_B, 8u, 0u)");
    expect(state, !kinetis_set_adc_input(device, 2u, KINETIS_ADC_MUX_A, 0u, 0u),
           "!kinetis_set_adc_input(device, 2u, KINETIS_ADC_MUX_A, 0u, 0u)");
    expect(state, !kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_COUNT, 0u, 0u),
           "!kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_COUNT, 0u, 0u)");
    expect(state, !kinetis_set_adc_input(NULL, 0u, KINETIS_ADC_MUX_A, 0u, 0u),
           "!kinetis_set_adc_input(NULL, 0u, KINETIS_ADC_MUX_A, 0u, 0u)");
    expect(state, kinetis_set_cmp_input(device, 1u, 7u, 1u),
           "kinetis_set_cmp_input(device, 1u, 7u, 1u)");
    expect(state, !kinetis_set_cmp_input(device, 3u, 0u, 0u),
           "!kinetis_set_cmp_input(device, 3u, 0u, 0u)");
    expect(state, !kinetis_set_cmp_input(NULL, 0u, 0u, 0u),
           "!kinetis_set_cmp_input(NULL, 0u, 0u, 0u)");

    expect(state, kinetis_set_lptmr_input(device, 2u, true),
           "kinetis_set_lptmr_input(device, 2u, true)");
    expect(state, !kinetis_set_lptmr_input(device, 3u, false),
           "!kinetis_set_lptmr_input(device, 3u, false)");
    expect(state, !kinetis_set_lptmr_input(NULL, 0u, false),
           "!kinetis_set_lptmr_input(NULL, 0u, false)");
    expect(state, kinetis_trigger_low_voltage_warning(device),
           "kinetis_trigger_low_voltage_warning(device)");
    expect(state, kinetis_trigger_low_voltage_detect(device),
           "kinetis_trigger_low_voltage_detect(device)");
    expect(state, !kinetis_trigger_low_voltage_warning(NULL),
           "!kinetis_trigger_low_voltage_warning(NULL)");
    expect(state, !kinetis_trigger_low_voltage_detect(NULL),
           "!kinetis_trigger_low_voltage_detect(NULL)");
    expect(state, kinetis_set_ewm_input(device, true), "kinetis_set_ewm_input(device, true)");
    expect(state, kinetis_ewm_output(device), "kinetis_ewm_output(device)");
    expect(state, !kinetis_set_ewm_input(NULL, false), "!kinetis_set_ewm_input(NULL, false)");
    expect(state, !kinetis_ewm_output(NULL), "!kinetis_ewm_output(NULL)");
    expect(state, kinetis_set_llwu_pin(device, 0u, true), "kinetis_set_llwu_pin(device, 0u, true)");
    expect(state, !kinetis_set_llwu_pin(device, 16u, false),
           "!kinetis_set_llwu_pin(device, 16u, false)");
    expect(state, !kinetis_set_llwu_pin(NULL, 0u, false), "!kinetis_set_llwu_pin(NULL, 0u, false)");
    expect(state, kinetis_trigger_llwu_module(device, 0u),
           "kinetis_trigger_llwu_module(device, 0u)");
    expect(state, !kinetis_trigger_llwu_module(device, 8u),
           "!kinetis_trigger_llwu_module(device, 8u)");
    expect(state, !kinetis_trigger_llwu_module(NULL, 0u), "!kinetis_trigger_llwu_module(NULL, 0u)");
    expect(state, kinetis_set_ftm_input(device, 0u, 0u, true),
           "kinetis_set_ftm_input(device, 0u, 0u, true)");
    expect(state, !kinetis_set_ftm_input(device, 4u, 0u, false),
           "!kinetis_set_ftm_input(device, 4u, 0u, false)");
    expect(state, !kinetis_set_ftm_input(NULL, 0u, 0u, false),
           "!kinetis_set_ftm_input(NULL, 0u, 0u, false)");
    expect(state, kinetis_set_ftm_clock_input(device, 0u, true),
           "kinetis_set_ftm_clock_input(device, 0u, true)");
    expect(state, !kinetis_set_ftm_clock_input(device, 2u, false),
           "!kinetis_set_ftm_clock_input(device, 2u, false)");
    expect(state, !kinetis_set_ftm_clock_input(NULL, 0u, false),
           "!kinetis_set_ftm_clock_input(NULL, 0u, false)");
    expect(state, kinetis_set_ftm_fault(device, 0u, 0u, true),
           "kinetis_set_ftm_fault(device, 0u, 0u, true)");
    expect(state, !kinetis_set_ftm_fault(device, 4u, 0u, false),
           "!kinetis_set_ftm_fault(device, 4u, 0u, false)");
    expect(state, !kinetis_set_ftm_fault(device, 0u, 4u, false),
           "!kinetis_set_ftm_fault(device, 0u, 4u, false)");
    expect(state, !kinetis_set_ftm_fault(NULL, 0u, 0u, false),
           "!kinetis_set_ftm_fault(NULL, 0u, 0u, false)");
    expect(state, kinetis_trigger_ftm_hardware(device, 0u, 0u),
           "kinetis_trigger_ftm_hardware(device, 0u, 0u)");
    expect(state, !kinetis_trigger_ftm_hardware(device, 4u, 0u),
           "!kinetis_trigger_ftm_hardware(device, 4u, 0u)");
    expect(state, !kinetis_trigger_ftm_hardware(device, 0u, 3u),
           "!kinetis_trigger_ftm_hardware(device, 0u, 3u)");
    expect(state, !kinetis_trigger_ftm_hardware(NULL, 0u, 0u),
           "!kinetis_trigger_ftm_hardware(NULL, 0u, 0u)");
    bool ftm_output = true;
    expect(state, kinetis_get_ftm_output(device, 0u, 0u, &ftm_output),
           "kinetis_get_ftm_output(device, 0u, 0u, &ftm_output)");
    expect(state, !ftm_output, "!ftm_output");
    expect(state, !kinetis_get_ftm_output(device, 4u, 0u, &ftm_output),
           "!kinetis_get_ftm_output(device, 4u, 0u, &ftm_output)");
    expect(state, !kinetis_get_ftm_output(NULL, 0u, 0u, &ftm_output),
           "!kinetis_get_ftm_output(NULL, 0u, 0u, &ftm_output)");

    uint16_t dac_output = 0u;
    expect(state, kinetis_get_dac_output(device, 0u, &dac_output),
           "kinetis_get_dac_output(device, 0u, &dac_output)");
    expect(state, !kinetis_get_dac_output(device, 2u, &dac_output),
           "!kinetis_get_dac_output(device, 2u, &dac_output)");
    expect(state, !kinetis_get_dac_output(NULL, 0u, &dac_output),
           "!kinetis_get_dac_output(NULL, 0u, &dac_output)");
    kinetis_rng_seed(device, 0x12345678u);
    kinetis_rng_seed(NULL, 0u);
}

static void test_serial_api(TestState* state, Kinetis* device) {
    expect(state, !kinetis_serial_set_clock_gate(NULL, KINETIS_PERIPHERAL_UART0, true),
           "null serial state rejects clock-gate changes");
    expect(state, kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART0, true),
           "kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART0, true)");
    serial_write(state, device, UART0 + 3u, 1u, 0x08u);
    serial_write(state, device, UART0 + 7u, 1u, 0x44u);
    expect(state, kinetis_serial_receive(device, KINETIS_SERIAL_UART1, 0x5au, 0u),
           "kinetis_serial_receive(device, KINETIS_SERIAL_UART1, 0x5au, 0u)");
    expect(state, !kinetis_serial_receive(NULL, KINETIS_SERIAL_UART1, 0u, 0u),
           "!kinetis_serial_receive(NULL, KINETIS_SERIAL_UART1, 0u, 0u)");
    uint16_t transmitted_value = 0u;
    expect(state, !kinetis_serial_transmit(device, KINETIS_SERIAL_UART1, &transmitted_value),
           "!kinetis_serial_transmit(device, KINETIS_SERIAL_UART1, &transmitted_value)");
    expect(state, !kinetis_serial_transmit(device, KINETIS_SERIAL_UART0, &transmitted_value),
           "!kinetis_serial_transmit(device, KINETIS_SERIAL_UART0, &transmitted_value)");
    expect(state, kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART0, true),
           "kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART0, true)");
    kinetis_serial_advance_endpoint(&device->serial, KINETIS_SERIAL_UART0);
    expect(state, kinetis_serial_transmit(device, KINETIS_SERIAL_UART0, &transmitted_value),
           "kinetis_serial_transmit(device, KINETIS_SERIAL_UART0, &transmitted_value)");
    expect(state, transmitted_value == 0x44u, "transmitted_value == 0x44u");
    expect(state,
           !kinetis_serial_transmit(device, KINETIS_SERIAL_ENDPOINT_COUNT, &transmitted_value),
           "!kinetis_serial_transmit(device, KINETIS_SERIAL_ENDPOINT_COUNT, "
           "&transmitted_value)");

    expect(state, kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_SPI0, true),
           "kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_SPI0, true)");
    serial_write(state, device, SPI0, 4u, UINT32_C(0x80000000));
    serial_write(state, device, SPI0 + 0x34u, 4u, 0x98030055u);
    kinetis_serial_advance(&device->serial, 64u);
    KinetisSpiTransfer spi_transfer;
    expect(state, kinetis_spi_transfer(device, KINETIS_SERIAL_SPI0, &spi_transfer),
           "kinetis_spi_transfer(device, KINETIS_SERIAL_SPI0, &spi_transfer)");
    expect(state, spi_transfer.data == 0x55u, "spi_transfer.data == 0x55u");
    expect(state, spi_transfer.chip_selects == 3u, "spi_transfer.chip_selects == 3u");
    expect(state, spi_transfer.clock_and_transfer_attributes == 1u,
           "spi_transfer.clock_and_transfer_attributes == 1u");
    expect(state, spi_transfer.continuous_chip_select, "spi_transfer.continuous_chip_select");
    expect(state, spi_transfer.end_of_queue, "spi_transfer.end_of_queue");
    expect(state, !kinetis_spi_transfer(device, KINETIS_SERIAL_UART0, &spi_transfer),
           "!kinetis_spi_transfer(device, KINETIS_SERIAL_UART0, &spi_transfer)");
    expect(state, !kinetis_spi_transfer(device, KINETIS_SERIAL_SPI0, NULL),
           "!kinetis_spi_transfer(device, KINETIS_SERIAL_SPI0, NULL)");
    expect(state, !kinetis_spi_transfer(device, KINETIS_SERIAL_SPI0, &spi_transfer),
           "empty SPI transfer queue is reported");

    expect(state, kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_I2C0, true),
           "kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_I2C0, true)");
    expect(state, kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_I2C1, true),
           "kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_I2C1, true)");
    serial_write(state, device, I2C0 + 2u, 1u, 0xf0u);
    serial_write(state, device, I2C1 + 2u, 1u, 0xf0u);
    serial_write(state, device, I2C0 + 4u, 1u, 0x52u);
    KinetisI2cTransfer i2c_transfer;
    expect(state, kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C1, &i2c_transfer),
           "kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C1, &i2c_transfer)");
    expect(state, i2c_transfer.type == KINETIS_I2C_START, "i2c_transfer.type == KINETIS_I2C_START");
    expect(state, kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C0, &i2c_transfer),
           "kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C0, &i2c_transfer)");
    expect(state, i2c_transfer.type == KINETIS_I2C_START, "i2c_transfer.type == KINETIS_I2C_START");
    expect(state, kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C0, &i2c_transfer),
           "kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C0, &i2c_transfer)");
    expect(state, i2c_transfer.type == KINETIS_I2C_WRITE, "i2c_transfer.type == KINETIS_I2C_WRITE");
    expect(state, i2c_transfer.value == 0x52u, "i2c_transfer.value == 0x52u");
    expect(state, !kinetis_i2c_transfer(device, KINETIS_SERIAL_UART0, &i2c_transfer),
           "!kinetis_i2c_transfer(device, KINETIS_SERIAL_UART0, &i2c_transfer)");
    expect(state, !kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C0, NULL),
           "!kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C0, NULL)");
    expect(state, kinetis_i2c_acknowledge(device, KINETIS_SERIAL_I2C0, false),
           "kinetis_i2c_acknowledge(device, KINETIS_SERIAL_I2C0, false)");
    expect(state, !kinetis_i2c_acknowledge(device, KINETIS_SERIAL_UART0, true),
           "!kinetis_i2c_acknowledge(device, KINETIS_SERIAL_UART0, true)");
    expect(state, !kinetis_i2c_detect_start(device, KINETIS_SERIAL_UART0),
           "I2C start rejects a non-I2C endpoint");
    expect(state, !kinetis_i2c_detect_stop(NULL, KINETIS_SERIAL_I2C0),
           "I2C stop rejects a null device");
    expect(state, kinetis_i2c_lose_arbitration(device, KINETIS_SERIAL_I2C0),
           "kinetis_i2c_lose_arbitration(device, KINETIS_SERIAL_I2C0)");
    serial_write(state, device, I2C0, 1u, 0x52u);
    serial_write(state, device, I2C0 + 2u, 1u, 0xc0u);
    expect(state, kinetis_i2c_address(device, KINETIS_SERIAL_I2C0, 0x29u, false),
           "kinetis_i2c_address(device, KINETIS_SERIAL_I2C0, 0x29u, false)");
    expect(state, !kinetis_i2c_address(device, KINETIS_SERIAL_I2C0, 0x2au, false),
           "!kinetis_i2c_address(device, KINETIS_SERIAL_I2C0, 0x2au, false)");
    expect(state, !kinetis_i2c_address(device, KINETIS_SERIAL_UART0, 0x29u, false),
           "!kinetis_i2c_address(device, KINETIS_SERIAL_UART0, 0x29u, false)");
    expect(state, !kinetis_i2c_lose_arbitration(device, KINETIS_SERIAL_UART0),
           "!kinetis_i2c_lose_arbitration(device, KINETIS_SERIAL_UART0)");
    expect(state, kinetis_i2c_receive(device, KINETIS_SERIAL_I2C0, 0x6bu),
           "kinetis_i2c_receive(device, KINETIS_SERIAL_I2C0, 0x6bu)");
    expect(state, !kinetis_i2c_receive(device, KINETIS_SERIAL_UART0, 0u),
           "!kinetis_i2c_receive(device, KINETIS_SERIAL_UART0, 0u)");

    uint8_t uart_received_value = 0u;
    uint16_t spi_received_value = 0u;
    expect(state,
           !kinetis_uart1_receive(NULL, 0u, 0u) && !kinetis_uart1_error(NULL, 0u) &&
               !kinetis_uart1_transmit(NULL, &uart_received_value) &&
               !kinetis_uart1_transmit(device, NULL) && !kinetis_spi0_receive(NULL, 0u) &&
               !kinetis_spi0_transmit(NULL, &spi_received_value) &&
               !kinetis_spi0_transmit(device, NULL) && !kinetis_i2c0_receive(NULL, 0u),
           "legacy serial APIs reject null arguments");
    uint16_t uart_received_count = 0u;
    uint16_t spi_received_count = 0u;
    while (uart_received_count <= KINETIS_SERIAL_FIFO_CAPACITY &&
           kinetis_uart1_receive(device, (uint8_t)uart_received_count, 0u)) {
        uart_received_count++;
    }
    while (spi_received_count <= KINETIS_SERIAL_FIFO_CAPACITY &&
           kinetis_spi0_receive(device, spi_received_count)) {
        spi_received_count++;
    }
    expect(state, uart_received_count != 0u && spi_received_count != 0u,
           "legacy serial receive queues accept data before filling");
    expect(state, !kinetis_uart1_receive(device, 0u, 0u) && !kinetis_spi0_receive(device, 0u),
           "legacy serial receive queues reject overflow");
}

static void test_io_api(TestState* state, Kinetis* device) {
    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_USB0, true);
    io_write(state, device, USB0 + 0x84u, 1u, 1u << 3u);
    io_write(state, device, USB0 + 0x94u, 1u, 1u);
    expect(state, kinetis_usb_token(device, 3u, 0x69u, false),
           "kinetis_usb_token(device, 3u, 0x69u, false)");
    expect(state, !kinetis_usb_token(NULL, 0u, 0u, false),
           "!kinetis_usb_token(NULL, 0u, 0u, false)");

    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_CAN0, true);
    io_write(state, device, CAN0, 4u, 0x0fu);
    io_write(state, device, CAN0 + 0x10u, 4u, 0u);
    io_write(state, device, CAN0 + 0x28u, 4u, 1u);
    io_write(state, device, CAN0 + 0x80u, 4u, 4u << 24u);
    KinetisCanFrame can_frame = {0x123u, 2u, {1u, 2u}, false, false};
    expect(state, kinetis_can_receive(device, &can_frame),
           "kinetis_can_receive(device, &can_frame)");
    expect(state, !kinetis_can_receive(device, NULL), "!kinetis_can_receive(device, NULL)");
    expect(state, !kinetis_can_receive(NULL, &can_frame), "!kinetis_can_receive(NULL, &can_frame)");

    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_I2S0, true);
    io_write(state, device, I2S0, 4u, UINT32_C(0x80000000));
    io_write(state, device, I2S0 + 0x80u, 4u, UINT32_C(0x80000000));
    io_write(state, device, I2S0 + 0x20u, 4u, 0x11223344u);
    uint32_t i2s_sample = 0u;
    expect(state, kinetis_i2s_transmit(device, &i2s_sample),
           "kinetis_i2s_transmit(device, &i2s_sample)");
    expect(state, i2s_sample == 0x11223344u, "i2s_sample == 0x11223344u");
    expect(state, kinetis_i2s_receive(device, 0x55667788u),
           "kinetis_i2s_receive(device, 0x55667788u)");
    expect(state, !kinetis_i2s_transmit(NULL, &i2s_sample),
           "!kinetis_i2s_transmit(NULL, &i2s_sample)");
    expect(state, !kinetis_i2s_receive(NULL, 0u), "!kinetis_i2s_receive(NULL, 0u)");

    bool gpio_high = false;
    expect(state, kinetis_gpio_drive(device, 0u, 1u, true),
           "kinetis_gpio_drive(device, 0u, 1u, true)");
    expect(state, kinetis_gpio_pin(device, 0u, 1u, &gpio_high) && gpio_high,
           "kinetis_gpio_pin(device, 0u, 1u, &gpio_high) && gpio_high");
    expect(state, kinetis_gpio_release(device, 0u, 1u), "kinetis_gpio_release(device, 0u, 1u)");
    expect(state, !kinetis_gpio_pin(NULL, 0u, 1u, &gpio_high),
           "!kinetis_gpio_pin(NULL, 0u, 1u, &gpio_high)");
    expect(state, !kinetis_gpio_pin(device, 0u, 1u, NULL),
           "!kinetis_gpio_pin(device, 0u, 1u, NULL)");
    expect(state, !kinetis_gpio_pin(device, UINT8_MAX, 0u, &gpio_high),
           "!kinetis_gpio_pin(device, UINT8_MAX, 0u, &gpio_high)");
    expect(state, !kinetis_gpio_drive(NULL, 0u, 0u, false),
           "!kinetis_gpio_drive(NULL, 0u, 0u, false)");
    expect(state, !kinetis_gpio_release(NULL, 0u, 0u), "!kinetis_gpio_release(NULL, 0u, 0u)");
}

static void test_mkv30_package_and_channels(TestState* state) {
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MKV30F12810);
    configuration.package = KINETIS_PACKAGE_FM_32_QFN;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "MKV30 host API device is created");
    bool output = false;
    expect(state, kinetis_set_ftm_input(device, 0u, 5u, true), "MKV30 FTM0 channel 5 exists");
    expect(state, !kinetis_set_ftm_input(device, 0u, 6u, true),
           "MKV30 FTM0 channel 6 does not exist");
    expect(state, !kinetis_get_ftm_output(device, 0u, 7u, &output),
           "MKV30 FTM0 channel 7 does not exist");
    expect(state, kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_A, 4u, 0x123u),
           "MKV30 FM32 ADC0 channel 4A exists");
    expect(state, kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_B, 4u, 0x456u),
           "MKV30 FM32 ADC0 channel 4B exists");
    expect(state, !kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_B, 5u, 0u),
           "MKV30 FM32 ADC0 channel 5B does not exist");
    expect(state, !kinetis_set_adc_input(device, 0u, KINETIS_ADC_MUX_A, 14u, 0u),
           "MKV30 FM32 ADC0 channel 14 does not exist");
    uint32_t vref_value = 0u;
    expect(state,
           !kinetis_peripheral_read(device, 0x40074000u, 1u, CORTEX_M4_ACCESS_DEBUG, &vref_value),
           "MKV30 FM32 package has no VREF peripheral");
    kinetis_destroy(device);
}

static void test_guards(TestState* state, Kinetis* device) {
    uint32_t guard_value = 0u;
    const uint8_t guard_memory[] = {1u, 2u, 3u, 4u};
    expect(state, !kinetis_reset(NULL), "!kinetis_reset(NULL)");
    expect(state, !kinetis_load(NULL, 0u, guard_memory, sizeof(guard_memory)),
           "!kinetis_load(NULL, 0u, guard_memory, sizeof(guard_memory))");
    expect(state, !kinetis_load(device, 0u, NULL, sizeof(guard_memory)),
           "!kinetis_load(device, 0u, NULL, sizeof(guard_memory))");
    expect(state, !kinetis_load(device, UINT32_MAX, guard_memory, sizeof(guard_memory)),
           "!kinetis_load(device, UINT32_MAX, guard_memory, sizeof(guard_memory))");
    expect(state, !kinetis_read(NULL, 0u, &guard_value, sizeof(guard_value)),
           "!kinetis_read(NULL, 0u, &guard_value, sizeof(guard_value))");
    expect(state, !kinetis_read(device, 0u, NULL, sizeof(guard_value)),
           "!kinetis_read(device, 0u, NULL, sizeof(guard_value))");
    expect(state, !kinetis_write(NULL, 0u, &guard_value, sizeof(guard_value)),
           "!kinetis_write(NULL, 0u, &guard_value, sizeof(guard_value))");
    expect(state, !kinetis_write(device, 0u, NULL, sizeof(guard_value)),
           "!kinetis_write(device, 0u, NULL, sizeof(guard_value))");
    expect(state, !kinetis_copy(NULL, device), "!kinetis_copy(NULL, device)");
    expect(state, !kinetis_copy(device, NULL), "!kinetis_copy(device, NULL)");
    expect(state, !kinetis_memory_read(NULL, 0u, 4u, CORTEX_M4_ACCESS_DEBUG, &guard_value),
           "!kinetis_memory_read(NULL, 0u, 4u, CORTEX_M4_ACCESS_DEBUG, &guard_value)");
    expect(state, !kinetis_memory_read(device, 0u, 4u, CORTEX_M4_ACCESS_DEBUG, NULL),
           "!kinetis_memory_read(device, 0u, 4u, CORTEX_M4_ACCESS_DEBUG, NULL)");
    expect(state, !kinetis_memory_read(device, 0u, 3u, CORTEX_M4_ACCESS_DEBUG, &guard_value),
           "!kinetis_memory_read(device, 0u, 3u, CORTEX_M4_ACCESS_DEBUG, &guard_value)");
    expect(state, !kinetis_memory_write(NULL, 0u, 4u, CORTEX_M4_ACCESS_DEBUG, guard_value),
           "!kinetis_memory_write(NULL, 0u, 4u, CORTEX_M4_ACCESS_DEBUG, guard_value)");
    expect(state, !kinetis_memory_write(device, 0u, 3u, CORTEX_M4_ACCESS_DEBUG, guard_value),
           "!kinetis_memory_write(device, 0u, 3u, CORTEX_M4_ACCESS_DEBUG, guard_value)");
    expect(state, !kinetis_flash_controller_write(NULL, 0u, 4u, guard_value),
           "!kinetis_flash_controller_write(NULL, 0u, 4u, guard_value)");
    expect(state, !kinetis_flash_controller_write(device, UINT32_MAX, 4u, guard_value),
           "!kinetis_flash_controller_write(device, UINT32_MAX, 4u, guard_value)");
    expect(state, !kinetis_flexbus_attach(NULL, 0u, guard_memory, sizeof(guard_memory), false),
           "!kinetis_flexbus_attach(NULL, 0u, guard_memory, sizeof(guard_memory), false)");
    expect(state, !kinetis_flexbus_attach(device, 0u, NULL, sizeof(guard_memory), false),
           "!kinetis_flexbus_attach(device, 0u, NULL, sizeof(guard_memory), false)");
    expect(state, !kinetis_flexbus_attach(device, 0u, guard_memory, 0u, false),
           "!kinetis_flexbus_attach(device, 0u, guard_memory, 0u, false)");
    expect(state,
           !kinetis_flexbus_attach(device, UINT32_MAX, guard_memory, sizeof(guard_memory), false),
           "!kinetis_flexbus_attach(device, UINT32_MAX, guard_memory, sizeof(guard_memory), "
           "false)");
    kinetis_flexbus_detach(NULL);
    expect(state, !kinetis_flexbus_read(NULL, 0u, &guard_value, sizeof(guard_value)),
           "!kinetis_flexbus_read(NULL, 0u, &guard_value, sizeof(guard_value))");
    expect(state, !kinetis_flexbus_read(device, 0u, NULL, sizeof(guard_value)),
           "!kinetis_flexbus_read(device, 0u, NULL, sizeof(guard_value))");
    expect(state, !kinetis_flexbus_read(device, 0u, &guard_value, sizeof(guard_value)),
           "!kinetis_flexbus_read(device, 0u, &guard_value, sizeof(guard_value))");
    expect(state, kinetis_set_usb_charger(device, KINETIS_USB_CHARGER_NONE),
           "kinetis_set_usb_charger(device, KINETIS_USB_CHARGER_NONE)");
    expect(state, kinetis_set_usb_charger(device, KINETIS_USB_CHARGER_ERROR),
           "kinetis_set_usb_charger(device, KINETIS_USB_CHARGER_ERROR)");
    expect(state, !kinetis_set_usb_charger(device, (KinetisUsbCharger)UINT8_MAX),
           "!kinetis_set_usb_charger(device, (KinetisUsbCharger)UINT8_MAX)");
    expect(state, !kinetis_set_usb_charger(NULL, KINETIS_USB_CHARGER_NONE),
           "!kinetis_set_usb_charger(NULL, KINETIS_USB_CHARGER_NONE)");
    expect(state, kinetis_set_usb_pullup(device, true), "kinetis_set_usb_pullup(device, true)");
    expect(state, kinetis_set_usb_pullup(device, false), "kinetis_set_usb_pullup(device, false)");
    expect(state, !kinetis_set_usb_pullup(NULL, true), "!kinetis_set_usb_pullup(NULL, true)");
    KinetisEvent event;
    while (kinetis_next_event(device, &event)) {
    }
    expect(state, !kinetis_next_event(device, &event), "!kinetis_next_event(device, &event)");
    expect(state, !kinetis_next_event(NULL, &event), "!kinetis_next_event(NULL, &event)");
    expect(state, !kinetis_next_event(device, NULL), "!kinetis_next_event(device, NULL)");
    uint8_t card_byte = 0u;
    expect(state, !kinetis_sdhc_read_card(NULL, 0u, &card_byte, sizeof(card_byte)),
           "SDHC card reads require a device");
}

#ifdef KINETIS_TEST_ALLOCATION_FAILURE
static void test_allocation_failures(TestState* state) {
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    configuration.profile = KINETIS_PROFILE_MK22FN1M012;
    configuration.package = KINETIS_PACKAGE_LQ_144_LQFP;
    configuration.flash_size = 4096u;
    configuration.sram_size = 65536u;
    uint32_t failure_count = 0u;
    uint32_t success_count = 0u;
    for (size_t allocation_index = 0u; allocation_index < 24u; allocation_index++) {
        test_fail_allocation_after(allocation_index);
        Kinetis* device = kinetis_create(configuration);
        test_allow_allocations();
        if (device == NULL) {
            failure_count++;
        } else {
            success_count++;
            kinetis_destroy(device);
        }
    }
    expect(state, failure_count >= 6u && success_count != 0u,
           "device construction handles each allocation boundary");

    Kinetis* device = kinetis_create(configuration);
    const uint8_t card_data[] = {1u, 2u, 3u, 4u};
    test_fail_allocation_after(0u);
    bool flexbus_attached = kinetis_flexbus_attach(device, 0u, card_data, sizeof(card_data), false);
    test_allow_allocations();
    expect(state, !flexbus_attached, "FlexBus attachment reports allocation failure");
    kinetis_destroy(device);
}
#endif

int main(void) {
    TestState state = {0};
    Kinetis* device = create_device(&state);
    test_data_api(&state, device);
    test_serial_api(&state, device);
    test_io_api(&state, device);
    test_guards(&state, device);
    kinetis_destroy(device);
    test_mkv30_package_and_channels(&state);
#ifdef KINETIS_TEST_ALLOCATION_FAILURE
    test_allocation_failures(&state);
#endif
    return test_finish(&state);
}
