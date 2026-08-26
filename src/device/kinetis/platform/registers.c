#include "device/kinetis/internal.h"

static bool manifest_read(Kinetis* device, uint32_t address, uint8_t access_size,
                          uint32_t* output_value) {
    const KinetisRegisterDescriptor* descriptor =
        kinetis_internal_manifest_descriptor_for_access(device, address, access_size);
    if (descriptor == NULL || (descriptor->access & KINETIS_REGISTER_ACCESS_READ) == 0 ||
        address < KINETIS_PERIPHERAL_BASE) {
        return false;
    }
    *output_value =
        kinetis_internal_raw_load(device, address, access_size) &
        kinetis_internal_manifest_access_mask(descriptor, address, descriptor->read_mask) &
        kinetis_internal_manifest_access_mask(descriptor, address, descriptor->implemented_mask) &
        kinetis_internal_width_mask(access_size);
    return true;
}

static bool manifest_write(Kinetis* device, uint32_t address, uint8_t access_size,
                           uint32_t write_value) {
    const KinetisRegisterDescriptor* descriptor =
        kinetis_internal_manifest_descriptor_for_access(device, address, access_size);
    if (descriptor == NULL || address < KINETIS_PERIPHERAL_BASE) {
        return false;
    }
    if ((descriptor->access & KINETIS_REGISTER_ACCESS_WRITE) == 0) {
        return true;
    }
    const uint32_t implemented_access_mask =
        kinetis_internal_manifest_access_mask(descriptor, address, descriptor->implemented_mask) &
        kinetis_internal_width_mask(access_size);
    const uint32_t write_mask =
        kinetis_internal_manifest_access_mask(descriptor, address, descriptor->write_mask);
    const uint32_t w1c_mask =
        kinetis_internal_manifest_access_mask(descriptor, address, descriptor->w1c_mask);
    const uint32_t writable_bits = write_mask & ~w1c_mask & implemented_access_mask;
    const uint32_t w1c_clear_bits = w1c_mask & write_value & implemented_access_mask;
    uint32_t register_value = kinetis_internal_raw_load(device, address, access_size);
    register_value = (register_value & ~writable_bits) | (write_value & writable_bits);
    register_value &= ~w1c_clear_bits;
    kinetis_internal_raw_store(device, address, access_size,
                               register_value & implemented_access_mask);
    return true;
}

static void apply_fmc_control(Kinetis* device, uint32_t address, uint8_t access_size,
                              uint32_t write_value) {
    if (address != KINETIS_FMC + 4u || access_size != 4u) {
        return;
    }
    const uint8_t selected_ways = (uint8_t)((write_value >> 20u) & 0x0fu);
    for (uint8_t cache_way = 0u; cache_way < KINETIS_FMC_WAY_COUNT; cache_way++) {
        if ((selected_ways & (1u << cache_way)) == 0u) {
            continue;
        }

        for (uint8_t cache_set = 0u; cache_set < device->profile->fmc_set_count; cache_set++) {
            const uint32_t tag_address =
                KINETIS_FMC + 0x100u +
                ((uint32_t)cache_way * device->profile->fmc_set_count + cache_set) * 4u;
            if (kinetis_register_manifest_lookup(device->profile->id, tag_address, 32u) != NULL) {
                kinetis_internal_raw_store(device, tag_address, 4u, 0u);
                device->fmc_bank[cache_way][cache_set] = 0u;
                device->fmc_age[cache_way][cache_set] = 0u;

                const uint8_t word_count = device->profile->fmc_line_size / 4u;
                for (uint8_t word_index = 0u; word_index < word_count; word_index++) {
                    const uint32_t data_address =
                        KINETIS_FMC + 0x200u +
                        ((uint32_t)cache_way * device->profile->fmc_set_count + cache_set) *
                            device->profile->fmc_line_size +
                        word_index * 4u;
                    kinetis_internal_raw_store(device, data_address, 4u, 0u);
                }
            }
        }
    }

    const uint32_t control_value = kinetis_internal_raw_load(device, address, 4u) & ~0x00f80000u;
    kinetis_internal_raw_store(device, address, 4u, control_value);
}

bool kinetis_peripheral_read(Kinetis* device, uint32_t address, uint8_t access_size,
                             CortexM4Access access, uint32_t* output_value) {
    KinetisPeripheralLocation location;
    const KinetisRegisterDescriptor* descriptor;

    if (device == NULL || output_value == NULL ||
        !kinetis_profile_resolve_peripheral(device->profile, address, access_size, &location) ||
        !kinetis_package_has_peripheral(device->package, location.id)) {
        return false;
    }
    descriptor = kinetis_internal_manifest_descriptor_for_access(device, address, access_size);
    if (location.id != KINETIS_PERIPHERAL_MCM &&
        !kinetis_internal_manifest_extension(location.id) &&
        (descriptor == NULL || (descriptor->access & KINETIS_REGISTER_ACCESS_READ) == 0u)) {
        return false;
    }
    if (!kinetis_internal_aips_access_allowed(device, address, access, false)) {
        return false;
    }
    if (access != CORTEX_M4_ACCESS_DEBUG &&
        !kinetis_internal_peripheral_clock_enabled(device, location.id)) {
        return false;
    }
    const bool debug_clock = access == CORTEX_M4_ACCESS_DEBUG &&
                             kinetis_internal_enable_debug_clock(device, location.id);
    bool handled =
        kinetis_internal_semantic_read(device, location.id, address, access_size, output_value);
    if (!handled) {
        handled = manifest_read(device, address, access_size, output_value);
    }
    if (debug_clock) {
        kinetis_sync_clock_gates(device);
    }
    kinetis_refresh_signals(device);
    return handled;
}

bool kinetis_peripheral_write(Kinetis* device, uint32_t address, uint8_t access_size,
                              CortexM4Access access, uint32_t write_value) {
    KinetisPeripheralLocation location;
    const KinetisRegisterDescriptor* descriptor;

    if (device == NULL ||
        !kinetis_profile_resolve_peripheral(device->profile, address, access_size, &location) ||
        !kinetis_package_has_peripheral(device->package, location.id)) {
        return false;
    }
    descriptor = kinetis_internal_manifest_descriptor_for_access(device, address, access_size);
    if (location.id != KINETIS_PERIPHERAL_MCM &&
        !kinetis_internal_manifest_extension(location.id) && descriptor == NULL) {
        return false;
    }
    if (descriptor != NULL && (descriptor->access & KINETIS_REGISTER_ACCESS_WRITE) == 0u) {
        return true;
    }
    if (descriptor != NULL) {
        write_value &= kinetis_internal_manifest_access_mask(
            descriptor, address, descriptor->write_mask & descriptor->implemented_mask);
    }
    if (!kinetis_internal_aips_access_allowed(device, address, access, true) ||
        (location.id == KINETIS_PERIPHERAL_AXBS &&
         !kinetis_internal_axbs_write_allowed(device, address)) ||
        (location.id == KINETIS_PERIPHERAL_FMC && access == CORTEX_M4_ACCESS_UNPRIVILEGED_DATA)) {
        return false;
    }
    if (access != CORTEX_M4_ACCESS_DEBUG &&
        !kinetis_internal_peripheral_clock_enabled(device, location.id)) {
        return false;
    }
    const bool debug_clock = access == CORTEX_M4_ACCESS_DEBUG &&
                             kinetis_internal_enable_debug_clock(device, location.id);
    bool handled =
        kinetis_internal_semantic_write(device, location.id, address, access_size, write_value);
    if (!handled) {
        handled = manifest_write(device, address, access_size, write_value);
    }
    if (handled && location.id == KINETIS_PERIPHERAL_FMC) {
        apply_fmc_control(device, address, access_size, write_value);
    }
    if (handled && (address == KINETIS_SIM_SCGC1 || address == KINETIS_SIM_SCGC2)) {
        kinetis_sync_clock_gates(device);
    }
    if (debug_clock) {
        kinetis_sync_clock_gates(device);
    }
    kinetis_refresh_signals(device);
    return handled;
}

static void reset_manifest(Kinetis* device) {
    memset(device->peripheral, 0, KINETIS_PERIPHERAL_SIZE);

    for (size_t register_index = 0; register_index < device->manifest->register_count;
         register_index++) {
        const KinetisRegisterDescriptor* descriptor = &device->manifest->registers[register_index];
        const uint8_t register_size = (uint8_t)(descriptor->width / 8u);
        if (descriptor->address >= KINETIS_PERIPHERAL_BASE &&
            descriptor->address - KINETIS_PERIPHERAL_BASE <=
                (uint32_t)KINETIS_PERIPHERAL_SIZE - register_size) {
            kinetis_internal_raw_store(device, descriptor->address, register_size,
                                       descriptor->reset_value & descriptor->implemented_mask);
        }
    }
}

void kinetis_peripheral_reset(Kinetis* device) {
    uint32_t external[5];
    uint32_t driven[5];
    memcpy(external, device->io.gpio_external, sizeof(external));
    memcpy(driven, device->io.gpio_external_drive, sizeof(driven));
    reset_manifest(device);
    device->cmt_cycles = 0u;
    device->cmt_bus_remainder = 0u;
    device->cmt_mark_ticks = 0u;
    device->cmt_period_ticks = 0u;
    device->cmt_carrier_high_ticks = 0u;
    device->cmt_carrier_period_ticks = 0u;
    device->cmt_carrier_offset_ticks = 0u;
    device->cmt_output_delay_ticks = 0u;
    kinetis_usbdcd_reset(&device->usbdcd);
    memset(device->fmc_bank, 0, sizeof(device->fmc_bank));
    memset(device->fmc_age, 0, sizeof(device->fmc_age));
    device->fmc_access_count = 0u;
    device->cmt_eoc_read = false;
    device->cmt_running = false;
    device->cmt_stop_pending = false;
    device->cmt_fsk_secondary = false;
    device->cmt_extended_space = false;
    device->cmt_dma_pending = false;
    memset(device->comparator_output, 0, sizeof(device->comparator_output));
    kinetis_data_reset(device->data);
    kinetis_serial_reset(&device->serial);
    kinetis_sdhc_reset(&device->sdhc);
    kinetis_io_reset(&device->io);
    memcpy(device->io.gpio_external, external, sizeof(external));
    memcpy(device->io.gpio_external_drive, driven, sizeof(driven));
    for (uint8_t port = 0; port < 5; port++) {
        device->io.gpio_filtered[port] = kinetis_io_pin_input(&device->io, port);
        device->io.gpio_pending[port] = device->io.gpio_filtered[port];
    }
    device->event_read_index = 0;
    device->event_write_index = 0;
    device->event_count = 0;
}

void kinetis_peripheral_advance(Kinetis* device, uint32_t cycle_count) {
    kinetis_timing_set_cpu_sleeping(&device->timing, device->cpu != NULL && device->cpu->sleeping,
                                    device->cpu != NULL && (device->cpu->scr & 4u) != 0u);
    kinetis_timing_set_debug_halted(&device->timing,
                                    device->cpu != NULL && device->cpu->debug.halted);
    kinetis_data_set_debug_halted(device->data, device->cpu != NULL && device->cpu->debug.halted);
    kinetis_timing_advance(&device->timing, cycle_count);
    kinetis_data_advance(device->data, cycle_count);
    kinetis_serial_advance(&device->serial, cycle_count);
    kinetis_io_advance(&device->io, cycle_count);
    if (kinetis_profile_has_peripheral(device->profile, KINETIS_PERIPHERAL_CMT) &&
        kinetis_internal_peripheral_clock_enabled(device, KINETIS_PERIPHERAL_CMT)) {
        kinetis_internal_cmt_advance(device, cycle_count);
    }
    if (kinetis_profile_has_peripheral(device->profile, KINETIS_PERIPHERAL_USBDCD) &&
        kinetis_internal_peripheral_clock_enabled(device, KINETIS_PERIPHERAL_USBDCD)) {
        kinetis_usbdcd_advance(&device->usbdcd, cycle_count);
        if (device->cpu != NULL)
            cortex_m4_set_irq_level(device->cpu, 54u, kinetis_usbdcd_irq(&device->usbdcd));
    }
    kinetis_refresh_signals(device);
}

bool kinetis_next_event(Kinetis* device, KinetisEvent* event) {
    if (device == NULL || event == NULL || device->event_count == 0) {
        return false;
    }
    *event = device->events[device->event_read_index];
    device->event_read_index = (uint8_t)((device->event_read_index + 1u) % KINETIS_EVENT_CAPACITY);
    device->event_count--;
    return true;
}

bool kinetis_set_adc_input(Kinetis* device, uint8_t instance, KinetisAdcMux mux, uint8_t channel,
                           uint16_t sample_value) {
    if (device == NULL ||
        !kinetis_package_adc_input_exists(device->package, instance, mux, channel))
        return false;
    return kinetis_data_set_adc_input(device->data, instance, mux, channel, sample_value);
}
