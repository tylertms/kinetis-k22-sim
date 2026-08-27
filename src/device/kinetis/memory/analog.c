#include "internal.h"

static uint32_t adc_conversion_cycles(const KinetisAdc* adc) {
    const uint8_t configuration1 = adc->registers[8];
    const uint8_t conversion_mode = (configuration1 >> 2) & 3u;
    uint32_t cycle_count = conversion_mode == 0u   ? 10u
                           : conversion_mode == 1u ? 14u
                           : conversion_mode == 2u ? 12u
                                                   : 18u;
    if ((configuration1 & 0x10u) != 0u)
        cycle_count += (adc->registers[9] & 3u) == 0u ? 12u : 20u;
    if ((adc->registers[0x24] & 0x04u) != 0)
        cycle_count *= 1u << ((adc->registers[0x24] & 3u) + 2u);
    return cycle_count;
}

static void adc_schedule(KinetisAdc* adc, uint8_t conversion_slot, bool clear_complete_flag) {
    if ((adc->registers[conversion_slot * 4u] & 0x1fu) == 31u)
        return;
    if (clear_complete_flag)
        adc->registers[conversion_slot * 4u] &= 0x7fu;
    adc->active_slot = conversion_slot;
    adc->remaining_cycles = adc_conversion_cycles(adc);
    adc->converting = true;
}

uint32_t kinetis_data_internal_adc_elapsed_cycles(KinetisData* data, KinetisAdc* adc,
                                                  uint32_t core_cycles) {
    if ((adc->registers[8] & 3u) != 0u)
        return core_cycles;
    if (!data->bus_clock_running || data->core_clock_hz == 0u || data->bus_clock_hz == 0u)
        return 0u;
    const uint32_t divider = 1u << ((adc->registers[8] >> 5u) & 3u);
    const uint64_t denominator = (uint64_t)data->core_clock_hz * divider;
    const uint64_t accumulated = adc->clock_remainder + (uint64_t)core_cycles * data->bus_clock_hz;
    adc->clock_remainder = accumulated % denominator;
    const uint64_t elapsed_cycles = accumulated / denominator;
    return elapsed_cycles > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_cycles;
}

void kinetis_data_internal_adc_start(KinetisAdc* adc, uint8_t slot) {
    adc_schedule(adc, slot, true);
}

static bool adc_compare(const KinetisAdc* adc, uint16_t conversion_result) {
    const uint8_t sc2 = adc->registers[0x20];
    if ((sc2 & 0x20u) == 0)
        return true;
    const uint16_t compare_value_low =
        (uint16_t)kinetis_data_internal_load_bytes(adc->registers, 0x18u, 2u);
    const uint16_t compare_value_high =
        (uint16_t)kinetis_data_internal_load_bytes(adc->registers, 0x1cu, 2u);
    if ((sc2 & 0x08u) != 0) {
        const bool within_range =
            conversion_result >= compare_value_low && conversion_result <= compare_value_high;
        return (sc2 & 0x10u) != 0u ? !within_range : within_range;
    }
    return (sc2 & 0x10u) != 0u ? conversion_result > compare_value_low
                               : conversion_result < compare_value_low;
}

static KinetisAdcMux adc_mux(const KinetisAdc* adc, uint8_t channel) {
    if (channel >= 4u && channel <= 7u && (adc->registers[0x0cu] & 0x10u) != 0u)
        return KINETIS_ADC_MUX_B;
    return KINETIS_ADC_MUX_A;
}

static uint16_t adc_result(const KinetisAdc* adc, uint8_t channel) {
    uint16_t sample_value = adc->inputs[adc_mux(adc, channel)][channel];
    const uint8_t conversion_mode = (adc->registers[8] >> 2) & 3u;
    const uint8_t resolution_bits = conversion_mode == 0u   ? 8u
                                    : conversion_mode == 1u ? 12u
                                    : conversion_mode == 2u ? 10u
                                                            : 16u;
    if (resolution_bits < 16u)
        sample_value &= (uint16_t)((1u << resolution_bits) - 1u);
    return sample_value;
}

void kinetis_data_internal_adc_complete(KinetisData* data, uint8_t instance) {
    KinetisAdc* adc = &data->adc[instance];
    const uint8_t conversion_slot = adc->active_slot;
    const uint8_t channel = adc->registers[conversion_slot * 4u] & 0x1fu;
    const uint16_t conversion_result = adc_result(adc, channel);
    adc->converting = false;
    if (adc_compare(adc, conversion_result)) {
        kinetis_data_internal_store_bytes(adc->registers, 0x10u + conversion_slot * 4u, 2u,
                                          conversion_result);
        adc->registers[conversion_slot * 4u] |= 0x80u;
        if ((adc->registers[conversion_slot * 4u] & 0x40u) != 0u)
            kinetis_data_internal_interrupt(
                data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_ADC0 + instance), true);
        if ((adc->registers[0x20] & 0x04u) != 0)
            kinetis_data_dma_request(data, (uint8_t)(40u + instance));
    }
    if ((adc->registers[0x24] & 0x08u) != 0)
        adc_schedule(adc, conversion_slot, false);
}

bool kinetis_data_internal_adc_read(KinetisData* data, uint8_t instance, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value) {
    KinetisAdc* adc = &data->adc[instance];
    const uint32_t register_offset = address - data->adc_base[instance];
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, ADC_REGISTER_SIZE))
        return false;
    *output_value = kinetis_data_internal_load_bytes(adc->registers, register_offset, byte_count);
    if ((register_offset == 0x10u || register_offset == 0x14u) && byte_count <= 4u) {
        const uint8_t conversion_slot = (uint8_t)((register_offset - 0x10u) / 4u);
        adc->registers[conversion_slot * 4u] &= 0x7fu;
        kinetis_data_internal_interrupt(
            data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_ADC0 + instance), false);
    }
    return true;
}

bool kinetis_data_internal_adc_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value) {
    KinetisAdc* adc = &data->adc[instance];
    const uint32_t register_offset = address - data->adc_base[instance];
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, ADC_REGISTER_SIZE))
        return false;
    if (register_offset == 0x10u || register_offset == 0x14u || register_offset == 0x28u)
        return false;
    if (register_offset == 0u || register_offset == 4u) {
        const uint8_t conversion_slot = (uint8_t)(register_offset / 4u);
        kinetis_data_internal_store_bytes(adc->registers, register_offset, byte_count,
                                          write_value & 0x7fu);
        kinetis_data_internal_interrupt(
            data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_ADC0 + instance), false);
        if ((adc->registers[0x20] & 0x40u) == 0)
            kinetis_data_internal_adc_start(adc, conversion_slot);
        return true;
    }
    if (register_offset == 0x24u) {
        const uint8_t command_code = (uint8_t)write_value;
        adc->registers[0x24] = command_code & 0xcfu;
        if ((command_code & 0x80u) != 0u) {
            adc->registers[0x24] &= 0x3fu;
            adc->registers[0] |= 0x80u;
        }
        return true;
    }
    kinetis_data_internal_store_bytes(adc->registers, register_offset, byte_count, write_value);
    if (register_offset <= 8u && register_offset + byte_count > 8u)
        adc->clock_remainder = 0u;
    return true;
}

void kinetis_data_internal_dac_update_output(KinetisData* data, uint8_t instance) {
    KinetisDac* dac = &data->dac[instance];
    if ((dac->registers[0x21] & 0x80u) == 0) {
        dac->output = 0;
        return;
    }
    const uint8_t table_index = (dac->registers[0x23] >> 4) & 15u;
    dac->output =
        (uint16_t)kinetis_data_internal_load_bytes(dac->registers, (uint32_t)table_index * 2u, 2u) &
        0x0fffu;
}

void kinetis_data_internal_dac_flags(KinetisData* data, uint8_t instance, uint8_t event_flags) {
    KinetisDac* dac = &data->dac[instance];
    dac->registers[0x20] |= event_flags;
    const uint8_t interrupt_enable_mask = dac->registers[0x21] & 7u;
    kinetis_data_internal_interrupt(data,
                                    (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_DAC0 + instance),
                                    (interrupt_enable_mask & event_flags) != 0u);
    if ((dac->registers[0x22] & 0x80u) != 0u && event_flags != 0u)
        kinetis_data_dma_request(data, (uint8_t)(45u + instance));
}

bool kinetis_data_internal_dac_read(KinetisData* data, uint8_t instance, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value) {
    const uint32_t register_offset = address - data->dac_base[instance];
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, DAC_REGISTER_SIZE))
        return false;
    *output_value = kinetis_data_internal_load_bytes(data->dac[instance].registers, register_offset,
                                                     byte_count);
    return true;
}

bool kinetis_data_internal_dac_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value) {
    KinetisDac* dac = &data->dac[instance];
    const uint32_t register_offset = address - data->dac_base[instance];
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, DAC_REGISTER_SIZE))
        return false;
    if (register_offset == 0x20u) {
        dac->registers[0x20] &= (uint8_t)~write_value;
        kinetis_data_internal_interrupt(
            data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_DAC0 + instance), false);
        return true;
    }
    kinetis_data_internal_store_bytes(dac->registers, register_offset, byte_count, write_value);
    kinetis_data_internal_dac_update_output(data, instance);
    if (register_offset <= 0x1fu && (dac->registers[0x22] & 0x80u) == 0u)
        kinetis_data_internal_dac_update_output(data, instance);
    return true;
}

static uint8_t cmp_level(const KinetisCmp* cmp, uint8_t input_selection) {
    if (input_selection == 7u && (cmp->registers[4] & 0x80u) != 0u)
        return cmp->registers[4] & 0x3fu;
    return cmp->inputs[input_selection & 7u];
}

void kinetis_data_internal_cmp_evaluate(KinetisData* data, uint8_t instance) {
    KinetisCmp* cmp = &data->cmp[instance];
    const bool comparator_enabled = (cmp->registers[1] & 1u) != 0u;
    const uint8_t positive_input = (cmp->registers[5] >> 3) & 7u;
    const uint8_t negative_input = cmp->registers[5] & 7u;
    bool comparator_high =
        comparator_enabled && cmp_level(cmp, positive_input) > cmp_level(cmp, negative_input);
    if ((cmp->registers[1] & 0x08u) != 0u)
        comparator_high = !comparator_high;
    const bool previously_high = (cmp->registers[3] & 1u) != 0u;
    if (comparator_high)
        cmp->registers[3] |= 1u;
    else
        cmp->registers[3] &= 0xfeu;
    if (!previously_high && comparator_high)
        cmp->registers[3] |= 4u;
    if (previously_high && !comparator_high)
        cmp->registers[3] |= 2u;
    const bool interrupt_pending =
        ((cmp->registers[3] & 0x08u) != 0u && (cmp->registers[3] & 4u) != 0u) ||
        ((cmp->registers[3] & 0x10u) != 0u && (cmp->registers[3] & 2u) != 0u);
    kinetis_data_internal_interrupt(
        data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_CMP0 + instance), interrupt_pending);
    if (interrupt_pending && (cmp->registers[3] & 0x40u) != 0u)
        kinetis_data_dma_request(data, (uint8_t)(42u + instance));
}

bool kinetis_data_internal_cmp_read(KinetisData* data, uint8_t instance, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value) {
    const uint32_t register_offset = address - (CMP_BASE + (uint32_t)instance * 8u);
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, CMP_REGISTER_SIZE))
        return false;
    *output_value = kinetis_data_internal_load_bytes(data->cmp[instance].registers, register_offset,
                                                     byte_count);
    return true;
}

bool kinetis_data_internal_cmp_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value) {
    KinetisCmp* cmp = &data->cmp[instance];
    const uint32_t register_offset = address - (CMP_BASE + (uint32_t)instance * 8u);
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, CMP_REGISTER_SIZE))
        return false;
    if (register_offset == 3u) {
        const uint8_t previous_status = cmp->registers[3];
        cmp->registers[3] = (uint8_t)((write_value & 0x78u) | (previous_status & 1u) |
                                      ((previous_status & 6u) & ~(write_value & 6u)));
    } else {
        kinetis_data_internal_store_bytes(cmp->registers, register_offset, byte_count, write_value);
    }
    kinetis_data_internal_cmp_evaluate(data, instance);
    return true;
}
