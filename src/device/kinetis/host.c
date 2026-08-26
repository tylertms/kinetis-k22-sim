#include "device/kinetis/internal.h"

void kinetis_set_adc0_channel(Kinetis* device, uint8_t channel, uint16_t sample_value) {
    (void)kinetis_set_adc_channel(device, 0, channel, sample_value);
}

bool kinetis_set_cmp_input(Kinetis* device, uint8_t instance, uint8_t input_index,
                           uint8_t input_level) {
    if (device == NULL)
        return false;
    return k22_data_set_cmp_input(device->data, instance, input_index, input_level);
}

bool kinetis_set_lptmr_input(Kinetis* device, uint8_t input_index, bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_lptmr_input(&device->timing, input_index, input_high);
}

bool kinetis_trigger_low_voltage_warning(Kinetis* device) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_low_voltage_warning(&device->timing);
}

bool kinetis_trigger_low_voltage_detect(Kinetis* device) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_low_voltage_detect(&device->timing);
}

bool kinetis_set_llwu_pin(Kinetis* device, uint8_t pin, bool pin_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_llwu_pin(&device->timing, pin, pin_high);
}

bool kinetis_trigger_llwu_module(Kinetis* device, uint8_t module) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_llwu_module(&device->timing, module);
}

bool kinetis_set_ewm_input(Kinetis* device, bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_ewm_input(&device->timing, input_high);
}

bool kinetis_ewm_output(const Kinetis* device) {
    if (device == NULL)
        return false;
    return k22_timing_ewm_output(&device->timing);
}

bool kinetis_get_cmt_output(const Kinetis* device, bool* is_driven, bool* is_high) {
    if (device == NULL || is_driven == NULL || is_high == NULL ||
        !k22_profile_has_peripheral(device->profile, K22_PERIPHERAL_CMT))
        return false;
    const uint8_t output_control = (uint8_t)kinetis_internal_raw_load(device, K22_CMT + 4u, 1u);
    *is_driven = (output_control & 0x20u) != 0u;
    bool is_active = (output_control & 0x80u) != 0u;
    if (device->cmt_running) {
        const uint8_t cmt_control = (uint8_t)kinetis_internal_raw_load(device, K22_CMT + 5u, 1u);
        is_active = false;
        if (!device->cmt_extended_space && device->cmt_cycles < device->cmt_mark_ticks &&
            device->cmt_output_delay_ticks == 0u) {
            const uint64_t carrier_ticks = device->cmt_cycles - device->cmt_carrier_offset_ticks;
            is_active =
                (cmt_control & 8u) != 0u ||
                (device->cmt_carrier_period_ticks != 0u &&
                 carrier_ticks % device->cmt_carrier_period_ticks < device->cmt_carrier_high_ticks);
        }
    }
    *is_high = is_active == ((output_control & 0x40u) != 0u);
    return true;
}

bool kinetis_set_ftm_input(Kinetis* device, uint8_t instance, uint8_t channel, bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_ftm_input(&device->timing, instance, channel, input_high);
}

bool kinetis_set_ftm_fault(Kinetis* device, uint8_t instance, uint8_t input_index,
                           bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_ftm_fault(&device->timing, instance, input_index, input_high);
}

bool kinetis_trigger_ftm_hardware(Kinetis* device, uint8_t instance, uint8_t trigger) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_ftm_hardware(&device->timing, instance, trigger);
}

bool kinetis_get_ftm_output(const Kinetis* device, uint8_t instance, uint8_t channel,
                            bool* output_high) {
    if (device == NULL)
        return false;
    return k22_timing_get_ftm_output(&device->timing, instance, channel, output_high);
}

bool kinetis_get_dac_output(const Kinetis* device, uint8_t instance, uint16_t* output_value) {
    if (device == NULL || instance >= 2u ||
        !k22_package_has_peripheral(device->package,
                                    (K22PeripheralId)(K22_PERIPHERAL_DAC0 + instance)))
        return false;
    return k22_data_get_dac_output(device->data, instance, output_value);
}

bool kinetis_set_usb_charger(Kinetis* device, KinetisUsbCharger charger) {
    if (device == NULL || charger > KINETIS_USB_CHARGER_ERROR ||
        !k22_profile_has_peripheral(device->profile, K22_PERIPHERAL_USBDCD)) {
        return false;
    }
    return k22_usbdcd_set_charger(&device->usbdcd, charger);
}

bool kinetis_set_usb_pullup(Kinetis* device, bool enabled) {
    if (device == NULL || !k22_profile_has_peripheral(device->profile, K22_PERIPHERAL_USBDCD))
        return false;
    return k22_usbdcd_set_pullup(&device->usbdcd, enabled);
}

bool kinetis_set_reset_state(Kinetis* device, uint8_t srs0, bool ackiso) {
    if (device == NULL)
        return false;
    return k22_timing_set_reset_state(&device->timing, srs0, ackiso);
}

void kinetis_rng_seed(Kinetis* device, uint32_t seed_value) {
    if (device != NULL) {
        k22_data_rng_seed(device->data, seed_value);
    }
}

bool kinetis_gpio_drive(Kinetis* device, uint8_t port, uint8_t pin, bool pin_high) {
    if (device != NULL && k22_io_drive_pin(&device->io, port, pin, pin_high)) {
        kinetis_refresh_signals(device);
        return true;
    }
    return false;
}

bool kinetis_gpio_release(Kinetis* device, uint8_t port, uint8_t pin) {
    if (device != NULL && k22_io_release_pin(&device->io, port, pin)) {
        kinetis_refresh_signals(device);
        return true;
    }
    return false;
}

bool kinetis_gpio_pin(const Kinetis* device, uint8_t port, uint8_t pin, bool* pin_high) {
    if (device == NULL || pin_high == NULL || !k22_package_pin_exists(device->package, port, pin)) {
        return false;
    }
    *pin_high = (k22_io_pin_input(&device->io, port) & (1u << pin)) != 0u;
    return true;
}

bool kinetis_serial_receive(Kinetis* device, KinetisSerialEndpoint endpoint,
                            uint16_t received_value, uint8_t status) {
    if (!kinetis_internal_serial_endpoint_available(device, endpoint)) {
        return false;
    }
    const K22PeripheralId id = kinetis_internal_serial_endpoint_peripheral(endpoint);
    (void)k22_serial_set_clock_gate(&device->serial, id, true);
    const bool receive_accepted = k22_serial_push_receive(
        &device->serial, (K22SerialEndpoint)endpoint, received_value, status);
    k22_serial_advance_endpoint(&device->serial, (K22SerialEndpoint)endpoint);
    kinetis_internal_refresh_serial_signals(device);
    kinetis_sync_clock_gates(device);
    return receive_accepted;
}

bool kinetis_serial_transmit(Kinetis* device, KinetisSerialEndpoint endpoint,
                             uint16_t* output_value) {
    if (!kinetis_internal_serial_endpoint_available(device, endpoint))
        return false;
    return k22_serial_pop_transmit(&device->serial, (K22SerialEndpoint)endpoint, output_value);
}

bool kinetis_spi_transfer(Kinetis* device, KinetisSerialEndpoint endpoint,
                          KinetisSpiTransfer* output_transfer) {
    if (!kinetis_internal_serial_endpoint_available(device, endpoint) || output_transfer == NULL ||
        endpoint < KINETIS_SERIAL_SPI0 || endpoint > KINETIS_SERIAL_SPI2) {
        return false;
    }

    K22SerialSpiTransfer internal_transfer;
    if (!k22_serial_pop_spi_transfer(&device->serial, (K22SerialEndpoint)endpoint,
                                     &internal_transfer)) {
        return false;
    }

    output_transfer->data = internal_transfer.data;
    output_transfer->chip_selects = internal_transfer.chip_selects;
    output_transfer->clock_and_transfer_attributes =
        internal_transfer.clock_and_transfer_attributes;
    output_transfer->continuous_chip_select = internal_transfer.continuous_chip_select;
    output_transfer->end_of_queue = internal_transfer.end_of_queue;
    return true;
}

bool kinetis_i2c_transfer(Kinetis* device, KinetisSerialEndpoint endpoint,
                          KinetisI2cTransfer* output_transfer) {
    if (!kinetis_internal_serial_endpoint_available(device, endpoint) || output_transfer == NULL ||
        endpoint < KINETIS_SERIAL_I2C0 || endpoint > KINETIS_SERIAL_I2C2) {
        return false;
    }

    K22SerialEvent serial_event;
    if (!kinetis_internal_pop_serial_event(&device->serial, (K22SerialEndpoint)endpoint,
                                           &serial_event)) {
        return false;
    }

    static const KinetisI2cTransferType transfer_types[] = {
        KINETIS_I2C_START, KINETIS_I2C_REPEATED_START, KINETIS_I2C_STOP,
        KINETIS_I2C_WRITE, KINETIS_I2C_READ,
    };
    if ((unsigned)serial_event.type >= sizeof(transfer_types) / sizeof(transfer_types[0])) {
        return false;
    }

    output_transfer->type = transfer_types[serial_event.type];
    output_transfer->value = (uint8_t)serial_event.value;
    return true;
}

bool kinetis_i2c_acknowledge(Kinetis* device, KinetisSerialEndpoint endpoint, bool acknowledge) {
    if (!kinetis_internal_serial_endpoint_available(device, endpoint) ||
        endpoint < KINETIS_SERIAL_I2C0 || endpoint > KINETIS_SERIAL_I2C2) {
        return false;
    }
    const bool acknowledge_accepted =
        k22_serial_i2c_set_acknowledge(&device->serial, (K22SerialEndpoint)endpoint, acknowledge);
    k22_serial_advance_endpoint(&device->serial, (K22SerialEndpoint)endpoint);
    kinetis_internal_refresh_serial_signals(device);
    return acknowledge_accepted;
}

static bool i2c_detect_bus_event(Kinetis* device, KinetisSerialEndpoint endpoint, bool start) {
    if (!kinetis_internal_serial_endpoint_available(device, endpoint) ||
        endpoint < KINETIS_SERIAL_I2C0 || endpoint > KINETIS_SERIAL_I2C2) {
        return false;
    }
    const bool detected =
        start ? k22_serial_i2c_detect_start(&device->serial, (K22SerialEndpoint)endpoint)
              : k22_serial_i2c_detect_stop(&device->serial, (K22SerialEndpoint)endpoint);
    kinetis_internal_refresh_serial_signals(device);
    return detected;
}

bool kinetis_i2c_detect_start(Kinetis* device, KinetisSerialEndpoint endpoint) {
    return i2c_detect_bus_event(device, endpoint, true);
}

bool kinetis_i2c_detect_stop(Kinetis* device, KinetisSerialEndpoint endpoint) {
    return i2c_detect_bus_event(device, endpoint, false);
}

bool kinetis_i2c_lose_arbitration(Kinetis* device, KinetisSerialEndpoint endpoint) {
    if (!kinetis_internal_serial_endpoint_available(device, endpoint) ||
        endpoint < KINETIS_SERIAL_I2C0 || endpoint > KINETIS_SERIAL_I2C2) {
        return false;
    }
    const bool arbitration_lost =
        k22_serial_i2c_lose_arbitration(&device->serial, (K22SerialEndpoint)endpoint);
    kinetis_internal_refresh_serial_signals(device);
    return arbitration_lost;
}

bool kinetis_i2c_receive(Kinetis* device, KinetisSerialEndpoint endpoint, uint8_t received_value) {
    if (endpoint < KINETIS_SERIAL_I2C0 || endpoint > KINETIS_SERIAL_I2C2)
        return false;
    return kinetis_serial_receive(device, endpoint, received_value, 0);
}

bool kinetis_usb_token(Kinetis* device, uint8_t endpoint, uint8_t token, bool transmit) {
    if (device == NULL)
        return false;
    const bool token_accepted = k22_io_usb_token(&device->io, endpoint, token, transmit);
    kinetis_refresh_signals(device);
    return token_accepted;
}

bool kinetis_can_receive(Kinetis* device, const KinetisCanFrame* input_frame) {
    if (device == NULL || input_frame == NULL) {
        return false;
    }

    K22CanFrame can_frame;
    can_frame.identifier = input_frame->identifier;
    can_frame.length = input_frame->length;
    memcpy(can_frame.data, input_frame->data, sizeof(can_frame.data));
    can_frame.extended = input_frame->extended;
    can_frame.remote = input_frame->remote;
    const bool frame_accepted = k22_io_can_receive(&device->io, &can_frame);
    kinetis_refresh_signals(device);
    return frame_accepted;
}

bool kinetis_i2s_receive(Kinetis* device, uint32_t sample_value) {
    if (device == NULL)
        return false;
    const bool sample_accepted = k22_io_i2s_receive(&device->io, sample_value);
    kinetis_refresh_signals(device);
    return sample_accepted;
}

bool kinetis_i2s_transmit(Kinetis* device, uint32_t* output_sample) {
    if (device == NULL)
        return false;
    const bool sample_transmitted = k22_io_i2s_transmit(&device->io, output_sample);
    kinetis_refresh_signals(device);
    return sample_transmitted;
}

bool kinetis_sdhc_insert(Kinetis* device, const void* card_data, size_t card_size,
                         bool write_protected) {
    if (device == NULL || !k22_package_has_peripheral(device->package, K22_PERIPHERAL_SDHC))
        return false;
    const bool card_inserted =
        k22_sdhc_insert(&device->sdhc, card_data, card_size, write_protected);
    kinetis_refresh_signals(device);
    return card_inserted;
}

void kinetis_sdhc_eject(Kinetis* device) {
    if (device != NULL) {
        k22_sdhc_eject(&device->sdhc);
        kinetis_refresh_signals(device);
    }
}

bool kinetis_sdhc_read_card(const Kinetis* device, size_t card_offset, void* card_data,
                            size_t byte_count) {
    if (device == NULL)
        return false;
    return k22_sdhc_read_card(&device->sdhc, card_offset, card_data, byte_count);
}

bool kinetis_uart1_receive(Kinetis* device, uint8_t received_value, uint8_t receive_status) {
    if (device == NULL) {
        return false;
    }
    K22SerialUart* uart = &device->serial.uart[1];
    if (!uart->present || uart->receive.count == K22_SERIAL_FIFO_CAPACITY) {
        return false;
    }
    uart->receive.values[uart->receive.write_index] = received_value;
    uart->receive.metadata[uart->receive.write_index] = receive_status & 0x0fu;
    uart->receive.write_index =
        (uint16_t)((uart->receive.write_index + 1u) % K22_SERIAL_FIFO_CAPACITY);
    uart->receive.count++;
    (void)k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART1, true);
    (void)k22_serial_read(&device->serial, uart->base + 4u, 1, &(uint32_t){0});
    kinetis_internal_refresh_serial_signals(device);
    kinetis_sync_clock_gates(device);
    return true;
}

bool kinetis_uart1_error(Kinetis* device, uint8_t receive_status) {
    enum { UART_STATUS_1_OFFSET = 4u };
    if (device == NULL) {
        return false;
    }
    K22SerialUart* uart = &device->serial.uart[1];
    if (!uart->present) {
        return false;
    }
    uart->registers[UART_STATUS_1_OFFSET] |= receive_status & 0x0fu;
    (void)k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART1, true);
    kinetis_internal_refresh_serial_signals(device);
    kinetis_sync_clock_gates(device);
    return true;
}

bool kinetis_uart1_transmit(Kinetis* device, uint8_t* output_value) {
    uint16_t wide_value = 0;
    if (device == NULL || output_value == NULL) {
        return false;
    }
    K22SerialFifo* fifo = &device->serial.uart[1].transmit;
    if (fifo->count == 0) {
        if (!kinetis_serial_transmit(device, KINETIS_SERIAL_UART1, &wide_value)) {
            return false;
        }
    } else {
        wide_value = fifo->values[fifo->read_index];
        fifo->read_index = (uint16_t)((fifo->read_index + 1u) % K22_SERIAL_FIFO_CAPACITY);
        fifo->count--;
    }
    *output_value = (uint8_t)wide_value;
    return true;
}

bool kinetis_spi0_receive(Kinetis* device, uint16_t received_value) {
    if (device == NULL) {
        return false;
    }
    K22SerialSpi* spi = &device->serial.spi[0];
    if (!spi->present || spi->receive.count == K22_SERIAL_FIFO_CAPACITY) {
        return false;
    }
    spi->receive.values[spi->receive.write_index] = received_value;
    spi->receive.metadata[spi->receive.write_index] = 0;
    spi->receive.write_index =
        (uint16_t)((spi->receive.write_index + 1u) % K22_SERIAL_FIFO_CAPACITY);
    spi->receive.count++;
    (void)k22_serial_read(&device->serial, spi->base, 4, &(uint32_t){0});
    kinetis_internal_refresh_serial_signals(device);
    return true;
}

bool kinetis_spi0_transmit(Kinetis* device, uint16_t* output_value) {
    if (device == NULL || output_value == NULL) {
        return false;
    }
    K22SerialFifo* fifo = &device->serial.spi[0].transmit;
    if (fifo->count == 0) {
        return kinetis_serial_transmit(device, KINETIS_SERIAL_SPI0, output_value);
    }
    *output_value = fifo->values[fifo->read_index];
    fifo->read_index = (uint16_t)((fifo->read_index + 1u) % K22_SERIAL_FIFO_CAPACITY);
    fifo->count--;
    return true;
}

bool kinetis_i2c0_transfer(Kinetis* device, KinetisI2cTransfer* transfer) {
    return kinetis_i2c_transfer(device, KINETIS_SERIAL_I2C0, transfer);
}

void kinetis_i2c0_acknowledge(Kinetis* device, bool acknowledge) {
    (void)kinetis_i2c_acknowledge(device, KINETIS_SERIAL_I2C0, acknowledge);
}

bool kinetis_i2c0_lose_arbitration(Kinetis* device) {
    return kinetis_i2c_lose_arbitration(device, KINETIS_SERIAL_I2C0);
}

bool kinetis_i2c0_receive(Kinetis* device, uint8_t received_value) {
    if (device == NULL || !device->serial.i2c[0].present) {
        return false;
    }
    device->serial.i2c[0].registers[4] = received_value;
    device->serial.i2c[0].registers[3] |= 0x82u;
    (void)k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_I2C0, true);
    kinetis_internal_refresh_serial_signals(device);
    kinetis_sync_clock_gates(device);
    return true;
}

K22DataBus kinetis_data_bus(Kinetis* device) {
    const K22DataBus bus = {device,
                            kinetis_internal_data_bus_read,
                            kinetis_internal_data_bus_write,
                            kinetis_internal_flash_bus_write,
                            kinetis_internal_data_interrupt,
                            kinetis_internal_data_dma_complete};
    return bus;
}

K22SdhcBus kinetis_sdhc_bus(Kinetis* device) {
    const K22SdhcBus bus = {device, kinetis_internal_data_bus_read,
                            kinetis_internal_data_bus_write};
    return bus;
}

K22TimingSignals kinetis_timing_signals(Kinetis* device) {
    const K22TimingSignals signals = {
        .context = device,
        .irq = kinetis_internal_timing_irq,
        .dma = kinetis_internal_timing_dma,
        .reset = kinetis_internal_timing_reset,
        .trigger = kinetis_internal_timing_trigger,
        .dma_trigger = kinetis_internal_timing_dma_trigger,
    };
    return signals;
}

K22IoConfiguration kinetis_io_configuration(Kinetis* device) {
    K22IoConfiguration configuration = k22_io_default_configuration(device->profile);
    for (uint8_t port = 0; port < K22_PACKAGE_PORT_COUNT; port++) {
        configuration.package_pin_mask[port] = k22_package_port_pin_mask(device->package, port);
    }
    configuration.event_handler = kinetis_internal_io_event;
    configuration.event_context = device;
    return configuration;
}
