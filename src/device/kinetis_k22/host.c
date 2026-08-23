#include "device/kinetis_k22/internal.h"

void kinetis_k22_set_adc0_channel(KinetisK22* device, uint8_t channel, uint16_t sample_value) {
    (void)kinetis_k22_set_adc_channel(device, 0, channel, sample_value);
}

bool kinetis_k22_set_cmp_input(KinetisK22* device, uint8_t instance, uint8_t input_index,
                               uint8_t input_level) {
    if (device == NULL)
        return false;
    return k22_data_set_cmp_input(device->data, instance, input_index, input_level);
}

bool kinetis_k22_set_lptmr_input(KinetisK22* device, uint8_t input_index, bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_lptmr_input(&device->timing, input_index, input_high);
}

bool kinetis_k22_trigger_low_voltage_warning(KinetisK22* device) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_low_voltage_warning(&device->timing);
}

bool kinetis_k22_trigger_low_voltage_detect(KinetisK22* device) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_low_voltage_detect(&device->timing);
}

bool kinetis_k22_set_llwu_pin(KinetisK22* device, uint8_t pin, bool pin_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_llwu_pin(&device->timing, pin, pin_high);
}

bool kinetis_k22_trigger_llwu_module(KinetisK22* device, uint8_t module) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_llwu_module(&device->timing, module);
}

bool kinetis_k22_set_ewm_input(KinetisK22* device, bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_ewm_input(&device->timing, input_high);
}

bool kinetis_k22_ewm_output(const KinetisK22* device) {
    if (device == NULL)
        return false;
    return k22_timing_ewm_output(&device->timing);
}

bool kinetis_k22_get_cmt_output(const KinetisK22* device, bool* is_driven, bool* is_high) {
    if (device == NULL || is_driven == NULL || is_high == NULL ||
        !k22_profile_has_peripheral(device->profile, K22_PERIPHERAL_CMT))
        return false;
    const uint8_t output_control = (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 4u, 1u);
    *is_driven = (output_control & 0x20u) != 0u;
    bool is_active = (output_control & 0x80u) != 0u;
    if (device->cmt_running) {
        const uint8_t cmt_control =
            (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u);
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

bool kinetis_k22_set_ftm_input(KinetisK22* device, uint8_t instance, uint8_t channel,
                               bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_ftm_input(&device->timing, instance, channel, input_high);
}

bool kinetis_k22_set_ftm_fault(KinetisK22* device, uint8_t instance, uint8_t input_index,
                               bool input_high) {
    if (device == NULL)
        return false;
    return k22_timing_set_ftm_fault(&device->timing, instance, input_index, input_high);
}

bool kinetis_k22_trigger_ftm_hardware(KinetisK22* device, uint8_t instance, uint8_t trigger) {
    if (device == NULL)
        return false;
    return k22_timing_trigger_ftm_hardware(&device->timing, instance, trigger);
}

bool kinetis_k22_get_ftm_output(const KinetisK22* device, uint8_t instance, uint8_t channel,
                                bool* output_high) {
    if (device == NULL)
        return false;
    return k22_timing_get_ftm_output(&device->timing, instance, channel, output_high);
}

bool kinetis_k22_get_dac_output(const KinetisK22* device, uint8_t instance,
                                uint16_t* output_value) {
    if (device == NULL || instance >= 2u ||
        !k22_package_has_peripheral(device->package,
                                    (K22PeripheralId)(K22_PERIPHERAL_DAC0 + instance)))
        return false;
    return k22_data_get_dac_output(device->data, instance, output_value);
}

bool kinetis_k22_set_usb_charger(KinetisK22* device, KinetisK22UsbCharger charger) {
    if (device == NULL || charger > KINETIS_K22_USB_CHARGER_ERROR ||
        !k22_profile_has_peripheral(device->profile, K22_PERIPHERAL_USBDCD)) {
        return false;
    }
    return k22_usbdcd_set_charger(&device->usbdcd, charger);
}

bool kinetis_k22_set_usb_pullup(KinetisK22* device, bool enabled) {
    if (device == NULL || !k22_profile_has_peripheral(device->profile, K22_PERIPHERAL_USBDCD))
        return false;
    return k22_usbdcd_set_pullup(&device->usbdcd, enabled);
}

void kinetis_k22_rng_seed(KinetisK22* device, uint32_t seed_value) {
    if (device != NULL) {
        k22_data_rng_seed(device->data, seed_value);
    }
}

bool kinetis_k22_gpio_drive(KinetisK22* device, uint8_t port, uint8_t pin, bool pin_high) {
    if (device != NULL && k22_io_drive_pin(&device->io, port, pin, pin_high)) {
        kinetis_k22_refresh_signals(device);
        return true;
    }
    return false;
}

bool kinetis_k22_gpio_release(KinetisK22* device, uint8_t port, uint8_t pin) {
    if (device != NULL && k22_io_release_pin(&device->io, port, pin)) {
        kinetis_k22_refresh_signals(device);
        return true;
    }
    return false;
}

bool kinetis_k22_gpio_pin(const KinetisK22* device, uint8_t port, uint8_t pin, bool* pin_high) {
    if (device == NULL || pin_high == NULL || !k22_package_pin_exists(device->package, port, pin)) {
        return false;
    }
    *pin_high = (k22_io_pin_input(&device->io, port) & (1u << pin)) != 0u;
    return true;
}

bool kinetis_k22_serial_receive(KinetisK22* device, KinetisK22SerialEndpoint endpoint,
                                uint16_t received_value, uint8_t status) {
    if (!kinetis_k22_internal_serial_endpoint_available(device, endpoint)) {
        return false;
    }
    const K22PeripheralId id = kinetis_k22_internal_serial_endpoint_peripheral(endpoint);
    (void)k22_serial_set_clock_gate(&device->serial, id, true);
    const bool receive_accepted = k22_serial_push_receive(
        &device->serial, (K22SerialEndpoint)endpoint, received_value, status);
    k22_serial_advance_endpoint(&device->serial, (K22SerialEndpoint)endpoint);
    kinetis_k22_internal_refresh_serial_signals(device);
    kinetis_k22_sync_clock_gates(device);
    return receive_accepted;
}

bool kinetis_k22_serial_transmit(KinetisK22* device, KinetisK22SerialEndpoint endpoint,
                                 uint16_t* output_value) {
    if (!kinetis_k22_internal_serial_endpoint_available(device, endpoint))
        return false;
    return k22_serial_pop_transmit(&device->serial, (K22SerialEndpoint)endpoint, output_value);
}

bool kinetis_k22_spi_transfer(KinetisK22* device, KinetisK22SerialEndpoint endpoint,
                              KinetisK22SpiTransfer* output_transfer) {
    if (!kinetis_k22_internal_serial_endpoint_available(device, endpoint) ||
        output_transfer == NULL || endpoint < KINETIS_K22_SERIAL_SPI0 ||
        endpoint > KINETIS_K22_SERIAL_SPI2) {
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

bool kinetis_k22_i2c_transfer(KinetisK22* device, KinetisK22SerialEndpoint endpoint,
                              KinetisK22I2cTransfer* output_transfer) {
    if (!kinetis_k22_internal_serial_endpoint_available(device, endpoint) ||
        output_transfer == NULL || endpoint < KINETIS_K22_SERIAL_I2C0 ||
        endpoint > KINETIS_K22_SERIAL_I2C2) {
        return false;
    }

    K22SerialEvent serial_event;
    if (!kinetis_k22_internal_pop_serial_event(&device->serial, (K22SerialEndpoint)endpoint,
                                               &serial_event)) {
        return false;
    }

    static const KinetisK22I2cTransferType transfer_types[] = {
        KINETIS_K22_I2C_START, KINETIS_K22_I2C_REPEATED_START, KINETIS_K22_I2C_STOP,
        KINETIS_K22_I2C_WRITE, KINETIS_K22_I2C_READ,
    };
    if ((unsigned)serial_event.type >= sizeof(transfer_types) / sizeof(transfer_types[0])) {
        return false;
    }

    output_transfer->type = transfer_types[serial_event.type];
    output_transfer->value = (uint8_t)serial_event.value;
    return true;
}

bool kinetis_k22_i2c_acknowledge(KinetisK22* device, KinetisK22SerialEndpoint endpoint,
                                 bool acknowledge) {
    if (!kinetis_k22_internal_serial_endpoint_available(device, endpoint) ||
        endpoint < KINETIS_K22_SERIAL_I2C0 || endpoint > KINETIS_K22_SERIAL_I2C2) {
        return false;
    }
    const bool acknowledge_accepted =
        k22_serial_i2c_set_acknowledge(&device->serial, (K22SerialEndpoint)endpoint, acknowledge);
    k22_serial_advance_endpoint(&device->serial, (K22SerialEndpoint)endpoint);
    kinetis_k22_internal_refresh_serial_signals(device);
    return acknowledge_accepted;
}

bool kinetis_k22_i2c_lose_arbitration(KinetisK22* device, KinetisK22SerialEndpoint endpoint) {
    if (!kinetis_k22_internal_serial_endpoint_available(device, endpoint) ||
        endpoint < KINETIS_K22_SERIAL_I2C0 || endpoint > KINETIS_K22_SERIAL_I2C2) {
        return false;
    }
    const bool arbitration_lost =
        k22_serial_i2c_lose_arbitration(&device->serial, (K22SerialEndpoint)endpoint);
    kinetis_k22_internal_refresh_serial_signals(device);
    return arbitration_lost;
}

bool kinetis_k22_i2c_receive(KinetisK22* device, KinetisK22SerialEndpoint endpoint,
                             uint8_t received_value) {
    if (endpoint < KINETIS_K22_SERIAL_I2C0 || endpoint > KINETIS_K22_SERIAL_I2C2)
        return false;
    return kinetis_k22_serial_receive(device, endpoint, received_value, 0);
}

bool kinetis_k22_usb_token(KinetisK22* device, uint8_t endpoint, uint8_t token, bool transmit) {
    if (device == NULL)
        return false;
    const bool token_accepted = k22_io_usb_token(&device->io, endpoint, token, transmit);
    kinetis_k22_refresh_signals(device);
    return token_accepted;
}

bool kinetis_k22_can_receive(KinetisK22* device, const KinetisK22CanFrame* input_frame) {
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
    kinetis_k22_refresh_signals(device);
    return frame_accepted;
}

bool kinetis_k22_i2s_receive(KinetisK22* device, uint32_t sample_value) {
    if (device == NULL)
        return false;
    const bool sample_accepted = k22_io_i2s_receive(&device->io, sample_value);
    kinetis_k22_refresh_signals(device);
    return sample_accepted;
}

bool kinetis_k22_i2s_transmit(KinetisK22* device, uint32_t* output_sample) {
    if (device == NULL)
        return false;
    const bool sample_transmitted = k22_io_i2s_transmit(&device->io, output_sample);
    kinetis_k22_refresh_signals(device);
    return sample_transmitted;
}

bool kinetis_k22_sdhc_insert(KinetisK22* device, const void* card_data, size_t card_size,
                             bool write_protected) {
    if (device == NULL || !k22_package_has_peripheral(device->package, K22_PERIPHERAL_SDHC))
        return false;
    const bool card_inserted =
        k22_sdhc_insert(&device->sdhc, card_data, card_size, write_protected);
    kinetis_k22_refresh_signals(device);
    return card_inserted;
}

void kinetis_k22_sdhc_eject(KinetisK22* device) {
    if (device != NULL) {
        k22_sdhc_eject(&device->sdhc);
        kinetis_k22_refresh_signals(device);
    }
}

bool kinetis_k22_sdhc_read_card(const KinetisK22* device, size_t card_offset, void* card_data,
                                size_t byte_count) {
    if (device == NULL)
        return false;
    return k22_sdhc_read_card(&device->sdhc, card_offset, card_data, byte_count);
}

bool kinetis_k22_uart1_receive(KinetisK22* device, uint8_t received_value, uint8_t receive_status) {
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
    kinetis_k22_internal_refresh_serial_signals(device);
    kinetis_k22_sync_clock_gates(device);
    return true;
}

bool kinetis_k22_uart1_transmit(KinetisK22* device, uint8_t* output_value) {
    uint16_t wide_value = 0;
    if (device == NULL || output_value == NULL) {
        return false;
    }
    K22SerialFifo* fifo = &device->serial.uart[1].transmit;
    if (fifo->count == 0) {
        if (!kinetis_k22_serial_transmit(device, KINETIS_K22_SERIAL_UART1, &wide_value)) {
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

bool kinetis_k22_spi0_receive(KinetisK22* device, uint16_t received_value) {
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
    kinetis_k22_internal_refresh_serial_signals(device);
    return true;
}

bool kinetis_k22_spi0_transmit(KinetisK22* device, uint16_t* output_value) {
    if (device == NULL || output_value == NULL) {
        return false;
    }
    K22SerialFifo* fifo = &device->serial.spi[0].transmit;
    if (fifo->count == 0) {
        return kinetis_k22_serial_transmit(device, KINETIS_K22_SERIAL_SPI0, output_value);
    }
    *output_value = fifo->values[fifo->read_index];
    fifo->read_index = (uint16_t)((fifo->read_index + 1u) % K22_SERIAL_FIFO_CAPACITY);
    fifo->count--;
    return true;
}

bool kinetis_k22_i2c0_transfer(KinetisK22* device, KinetisK22I2cTransfer* transfer) {
    return kinetis_k22_i2c_transfer(device, KINETIS_K22_SERIAL_I2C0, transfer);
}

void kinetis_k22_i2c0_acknowledge(KinetisK22* device, bool acknowledge) {
    (void)kinetis_k22_i2c_acknowledge(device, KINETIS_K22_SERIAL_I2C0, acknowledge);
}

bool kinetis_k22_i2c0_lose_arbitration(KinetisK22* device) {
    return kinetis_k22_i2c_lose_arbitration(device, KINETIS_K22_SERIAL_I2C0);
}

bool kinetis_k22_i2c0_receive(KinetisK22* device, uint8_t received_value) {
    if (device == NULL || !device->serial.i2c[0].present) {
        return false;
    }
    device->serial.i2c[0].registers[4] = received_value;
    device->serial.i2c[0].registers[3] |= 0x82u;
    (void)k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_I2C0, true);
    kinetis_k22_internal_refresh_serial_signals(device);
    kinetis_k22_sync_clock_gates(device);
    return true;
}

K22DataBus kinetis_k22_data_bus(KinetisK22* device) {
    const K22DataBus bus = {device,
                            kinetis_k22_internal_data_bus_read,
                            kinetis_k22_internal_data_bus_write,
                            kinetis_k22_internal_flash_bus_write,
                            kinetis_k22_internal_data_interrupt,
                            kinetis_k22_internal_data_dma_complete};
    return bus;
}

K22SdhcBus kinetis_k22_sdhc_bus(KinetisK22* device) {
    const K22SdhcBus bus = {device, kinetis_k22_internal_data_bus_read,
                            kinetis_k22_internal_data_bus_write};
    return bus;
}

K22TimingSignals kinetis_k22_timing_signals(KinetisK22* device) {
    const K22TimingSignals signals = {
        .context = device,
        .irq = kinetis_k22_internal_timing_irq,
        .dma = kinetis_k22_internal_timing_dma,
        .reset = kinetis_k22_internal_timing_reset,
        .trigger = kinetis_k22_internal_timing_trigger,
        .dma_trigger = kinetis_k22_internal_timing_dma_trigger,
    };
    return signals;
}

K22IoConfiguration kinetis_k22_io_configuration(KinetisK22* device) {
    K22IoConfiguration configuration = k22_io_default_configuration(device->profile);
    for (uint8_t port = 0; port < K22_PACKAGE_PORT_COUNT; port++) {
        configuration.package_pin_mask[port] = k22_package_port_pin_mask(device->package, port);
    }
    configuration.event_handler = kinetis_k22_internal_io_event;
    configuration.event_context = device;
    return configuration;
}
