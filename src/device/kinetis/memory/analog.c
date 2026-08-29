#include "internal.h"

static uint32_t adc_clock_hz(const KinetisData* data, const KinetisAdc* adc);

static uint32_t adc_conversion_cycles(const KinetisData* data, const KinetisAdc* adc,
                                      bool first_conversion) {
    const uint8_t configuration1 = adc->registers[8];
    const uint8_t conversion_mode = (configuration1 >> 2) & 3u;
    const bool differential = (adc->registers[adc->active_slot * 4u] & 0x20u) != 0u;
    static const uint8_t single_ended_cycles[] = {17u, 20u, 20u, 25u};
    static const uint8_t differential_cycles[] = {27u, 30u, 30u, 34u};
    uint32_t conversion_cycles =
        differential ? differential_cycles[conversion_mode] : single_ended_cycles[conversion_mode];
    if ((configuration1 & 0x10u) != 0u) {
        static const uint8_t long_sample_cycles[] = {20u, 12u, 6u, 2u};
        conversion_cycles += long_sample_cycles[adc->registers[9] & 3u];
    }
    if ((adc->registers[9] & 4u) != 0u)
        conversion_cycles += 2u;
    if (!first_conversion)
        return conversion_cycles;
    const uint8_t clock_source = configuration1 & 3u;
    const uint32_t adc_hz = adc_clock_hz(data, adc);
    const uint32_t bus_clock_adder =
        data->bus_clock_hz == 0u
            ? 0u
            : (uint32_t)(((uint64_t)5u * adc_hz + data->bus_clock_hz - 1u) /
                         data->bus_clock_hz);
    conversion_cycles += (configuration1 & 0x10u) != 0u ? 3u : 5u;
    conversion_cycles += bus_clock_adder;
    if (clock_source == 3u && (adc->registers[9] & 8u) == 0u)
        conversion_cycles += (adc_hz + 199999u) / 200000u;
    return conversion_cycles;
}

static void adc_schedule(KinetisData* data, KinetisAdc* adc, uint8_t conversion_slot,
                         bool clear_complete_flag, bool first_conversion) {
    if ((adc->registers[conversion_slot * 4u] & 0x1fu) == 31u)
        return;
    if (clear_complete_flag)
        adc->registers[conversion_slot * 4u] &= 0x7fu;
    adc->active_slot = conversion_slot;
    const uint8_t sc3 = adc->registers[0x24];
    adc->average_count = (uint8_t)((sc3 & 4u) == 0u ? 1u : 1u << ((sc3 & 3u) + 2u));
    adc->average_remaining = adc->average_count;
    adc->average_sum = 0;
    adc->remaining_cycles = adc_conversion_cycles(data, adc, first_conversion);
    adc->converting = true;
    adc->registers[0x20] |= 0x80u;
}

static void adc_abort(KinetisAdc* adc) {
    adc->converting = false;
    adc->calibrating = false;
    adc->remaining_cycles = 0u;
    adc->clock_remainder = 0u;
    adc->registers[0x20] &= 0x7fu;
}

static uint32_t adc_clock_hz(const KinetisData* data, const KinetisAdc* adc) {
    const uint8_t clock_source = adc->registers[8] & 3u;
    const uint32_t divider = 1u << ((adc->registers[8] >> 5u) & 3u);
    const uint8_t instance = (uint8_t)(adc - data->adc);
    static const uint32_t adack_typical_hz[2][2] = {
        {5200000u, 6200000u},
        {2400000u, 4000000u},
    };
    const uint8_t low_power = (adc->registers[8] >> 7u) & 1u;
    const uint8_t high_speed = (adc->registers[9] >> 2u) & 1u;
    const uint32_t source_hz =
        clock_source == 0u   ? data->bus_clock_hz
        : clock_source == 1u ? data->bus_clock_hz / 2u
        : clock_source == 2u ? data->adc_alt_clock_hz[instance]
                             : adack_typical_hz[low_power][high_speed];
    return source_hz / divider;
}

static void adc_start_calibration(KinetisData* data, KinetisAdc* adc) {
    adc_abort(adc);
    if ((adc->registers[0x20] & 0x40u) != 0u) {
        adc->registers[0x24] = (adc->registers[0x24] & 0x3fu) | 0x40u;
        return;
    }
    adc->registers[0x24] = (adc->registers[0x24] & 0x3fu) | 0x80u;
    const uint32_t adc_hz = adc_clock_hz(data, adc);
    const uint32_t bus_adder =
        data->bus_clock_hz == 0u
            ? 0u
            : (uint32_t)(((uint64_t)100u * adc_hz + data->bus_clock_hz - 1u) /
                         data->bus_clock_hz);
    adc->remaining_cycles = 14000u + bus_adder;
    adc->converting = true;
    adc->calibrating = true;
}

uint32_t kinetis_data_internal_adc_elapsed_cycles(KinetisData* data, KinetisAdc* adc,
                                                  uint32_t core_cycles) {
    const uint8_t clock_source = adc->registers[8] & 3u;
    if ((clock_source == 0u || clock_source == 1u) && !data->bus_clock_running)
        return 0u;
    if (data->core_clock_hz == 0u)
        return 0u;
    const uint32_t clock_hz = adc_clock_hz(data, adc);
    if (clock_hz == 0u)
        return 0u;
    const uint64_t denominator = data->core_clock_hz;
    const uint64_t accumulated = adc->clock_remainder + (uint64_t)core_cycles * clock_hz;
    adc->clock_remainder = accumulated % denominator;
    const uint64_t elapsed_cycles = accumulated / denominator;
    return elapsed_cycles > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_cycles;
}

void kinetis_data_internal_adc_start(KinetisData* data, KinetisAdc* adc, uint8_t slot) {
    adc_schedule(data, adc, slot, true, true);
}

static int32_t adc_signed_value(const KinetisAdc* adc, uint16_t value) {
    const uint8_t conversion_mode = (adc->registers[8] >> 2u) & 3u;
    static const uint8_t differential_bits[] = {9u, 13u, 11u, 16u};
    const uint8_t bits = differential_bits[conversion_mode];
    const uint32_t sign = 1u << (bits - 1u);
    const uint32_t mask = bits == 16u ? 0xffffu : (1u << bits) - 1u;
    const uint32_t masked = value & mask;
    return (int32_t)((masked ^ sign) - sign);
}

static bool adc_compare(const KinetisAdc* adc, uint16_t conversion_result) {
    const uint8_t sc2 = adc->registers[0x20];
    if ((sc2 & 0x20u) == 0)
        return true;
    const uint16_t raw_low =
        (uint16_t)kinetis_data_internal_load_bytes(adc->registers, 0x18u, 2u);
    const uint16_t raw_high =
        (uint16_t)kinetis_data_internal_load_bytes(adc->registers, 0x1cu, 2u);
    const bool differential = (adc->registers[adc->active_slot * 4u] & 0x20u) != 0u;
    const int32_t result = differential ? adc_signed_value(adc, conversion_result)
                                        : (int32_t)conversion_result;
    const int32_t low = differential ? adc_signed_value(adc, raw_low) : raw_low;
    const int32_t high = differential ? adc_signed_value(adc, raw_high) : raw_high;
    const bool greater = (sc2 & 0x10u) != 0u;
    if ((sc2 & 0x08u) == 0u)
        return greater ? result >= low : result < low;
    if (low <= high)
        return greater ? result >= low && result <= high : result < low || result > high;
    return greater ? result >= low || result <= high : result < low && result > high;
}

static void adc_store_calibration_results(KinetisAdc* adc) {
    static const uint16_t results[] = {0x0au, 0x20u, 0x200u, 0x100u, 0x80u, 0x40u, 0x20u};
    kinetis_data_internal_store_bytes(adc->registers, 0x28u, 2u, 4u);
    for (size_t index = 0u; index < sizeof(results) / sizeof(results[0]); index++) {
        kinetis_data_internal_store_bytes(adc->registers, 0x34u + (uint32_t)index * 4u, 2u,
                                          results[index]);
        kinetis_data_internal_store_bytes(adc->registers, 0x54u + (uint32_t)index * 4u, 2u,
                                          results[index]);
    }
}

static KinetisAdcMux adc_mux(const KinetisAdc* adc, uint8_t channel) {
    if (channel >= 4u && channel <= 7u && (adc->registers[0x0cu] & 0x10u) != 0u)
        return KINETIS_ADC_MUX_B;
    return KINETIS_ADC_MUX_A;
}

static uint16_t adc_result(const KinetisAdc* adc, uint8_t channel) {
    const uint16_t sample_value = adc->inputs[adc_mux(adc, channel)][channel];
    const uint8_t conversion_mode = (adc->registers[8] >> 2) & 3u;
    const bool differential = (adc->registers[adc->active_slot * 4u] & 0x20u) != 0u;
    static const uint8_t single_ended_bits[] = {8u, 12u, 10u, 16u};
    static const uint8_t differential_bits[] = {9u, 13u, 11u, 16u};
    const uint8_t resolution_bits = differential ? differential_bits[conversion_mode]
                                                 : single_ended_bits[conversion_mode];
    const uint32_t result_mask =
        resolution_bits == 16u ? 0xffffu : (1u << resolution_bits) - 1u;
    const uint32_t sign = 1u << (resolution_bits - 1u);
    const uint32_t masked_sample = sample_value & result_mask;
    int32_t corrected =
        differential ? (int32_t)((masked_sample ^ sign) - sign) : (int32_t)masked_sample;
    const int32_t offset =
        (int16_t)kinetis_data_internal_load_bytes(adc->registers, 0x28u, 2u);
    const uint8_t offset_right_shift =
        differential ? (uint8_t)(16u - resolution_bits)
        : resolution_bits < 16u ? (uint8_t)(15u - resolution_bits) : 0u;
    int32_t applied_offset;
    if (!differential && resolution_bits == 16u) {
        applied_offset = offset * 2;
    } else if (offset >= 0) {
        applied_offset = offset >> offset_right_shift;
    } else {
        const int32_t divisor = 1 << offset_right_shift;
        applied_offset = -((-offset + divisor - 1) / divisor);
    }
    corrected -= applied_offset;
    const bool negative_differential = differential && (masked_sample & sign) != 0u;
    const uint32_t gain = kinetis_data_internal_load_bytes(
                              adc->registers, negative_differential ? 0x30u : 0x2cu, 2u) &
                          0xffffu;
    corrected = (int32_t)(((int64_t)corrected * gain) / 0x8000);
    const int32_t minimum = differential ? -(int32_t)sign : 0;
    const int32_t maximum = differential ? (int32_t)sign - 1 : (int32_t)result_mask;
    if (corrected < minimum)
        corrected = minimum;
    if (corrected > maximum)
        corrected = maximum;
    return differential ? (uint16_t)corrected
                        : (uint16_t)((uint32_t)corrected & result_mask);
}

static void adc_refresh_interrupt(KinetisData* data, uint8_t instance) {
    const KinetisAdc* adc = &data->adc[instance];
    const bool asserted = (adc->registers[0] & 0xc0u) == 0xc0u ||
                          (adc->registers[4] & 0xc0u) == 0xc0u;
    kinetis_data_internal_interrupt(
        data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_ADC0 + instance), asserted);
}

void kinetis_data_internal_adc_complete(KinetisData* data, uint8_t instance) {
    KinetisAdc* adc = &data->adc[instance];
    if (adc->calibrating) {
        adc->converting = false;
        adc->calibrating = false;
        adc_store_calibration_results(adc);
        adc->registers[0x24] &= 0x3fu;
        adc->registers[0] |= 0x80u;
        adc_refresh_interrupt(data, instance);
        return;
    }
    const uint8_t conversion_slot = adc->active_slot;
    const uint8_t channel = adc->registers[conversion_slot * 4u] & 0x1fu;
    const uint16_t sample = adc_result(adc, channel);
    const bool differential = (adc->registers[conversion_slot * 4u] & 0x20u) != 0u;
    adc->average_sum += differential ? adc_signed_value(adc, sample) : sample;
    adc->average_remaining--;
    if (adc->average_remaining != 0u) {
        adc->remaining_cycles = adc_conversion_cycles(data, adc, false);
        return;
    }
    const uint16_t conversion_result = (uint16_t)(adc->average_sum / adc->average_count);
    adc->converting = false;
    adc->registers[0x20] &= 0x7fu;
    if (adc_compare(adc, conversion_result)) {
        kinetis_data_internal_store_bytes(adc->registers, 0x10u + conversion_slot * 4u, 2u,
                                          conversion_result);
        adc->registers[conversion_slot * 4u] |= 0x80u;
        if (data->bus.adc_complete != NULL)
            data->bus.adc_complete(data->bus.context, instance, conversion_slot);
        adc_refresh_interrupt(data, instance);
        if ((adc->registers[0x20] & 0x04u) != 0)
            kinetis_data_dma_request(data, (uint8_t)(40u + instance));
    }
    if ((adc->registers[0x24] & 0x08u) != 0)
        adc_schedule(data, adc, conversion_slot, false, false);
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
        adc_refresh_interrupt(data, instance);
    }
    return true;
}

bool kinetis_data_internal_adc_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value) {
    KinetisAdc* adc = &data->adc[instance];
    const uint32_t register_offset = address - data->adc_base[instance];
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, ADC_REGISTER_SIZE))
        return false;
    if (adc->calibrating) {
        adc_abort(adc);
        adc->registers[0x24] = (adc->registers[0x24] & 0x3fu) | 0x40u;
        return true;
    }
    if (register_offset == 0x10u || register_offset == 0x14u)
        return false;
    if (register_offset == 0u || register_offset == 4u) {
        const uint8_t conversion_slot = (uint8_t)(register_offset / 4u);
        if (conversion_slot == 0u || adc->active_slot == conversion_slot) {
            adc_abort(adc);
        }
        kinetis_data_internal_store_bytes(adc->registers, register_offset, byte_count,
                                          write_value & 0x7fu);
        adc_refresh_interrupt(data, instance);
        if (conversion_slot == 0u && (adc->registers[0x20] & 0x40u) == 0)
            kinetis_data_internal_adc_start(data, adc, 0u);
        return true;
    }
    adc_abort(adc);
    if (register_offset == 0x24u) {
        const uint8_t command_code = (uint8_t)write_value;
        const uint8_t calibration_failure =
            (uint8_t)(adc->registers[0x24] & 0x40u & ~(command_code & 0x40u));
        adc->registers[0x24] = (command_code & 0x0fu) | calibration_failure;
        if ((command_code & 0x80u) != 0u)
            adc_start_calibration(data, adc);
        return true;
    }
    kinetis_data_internal_store_bytes(adc->registers, register_offset, byte_count, write_value);
    if (register_offset <= 0x20u && register_offset + byte_count > 0x20u)
        adc->registers[0x20] &= 0x7fu;
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
    const bool disabled_mkv10_buffer = data->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
                                       (dac->registers[0x22] & 1u) == 0u;
    const uint8_t table_index =
        disabled_mkv10_buffer ? 0u : (dac->registers[0x23] >> 4) & 15u;
    dac->output =
        (uint16_t)kinetis_data_internal_load_bytes(dac->registers, (uint32_t)table_index * 2u, 2u) &
        0x0fffu;
}

void kinetis_data_internal_dac_flags(KinetisData* data, uint8_t instance, uint8_t event_flags) {
    KinetisDac* dac = &data->dac[instance];
    dac->registers[0x20] |= event_flags;
    const uint8_t interrupt_enable_mask = dac->registers[0x21] & 7u;
    const uint8_t pending = interrupt_enable_mask & dac->registers[0x20] & 7u;
    const bool dma_enabled = (dac->registers[0x22] & 0x80u) != 0u;
    kinetis_data_internal_interrupt(
        data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_DAC0 + instance),
        !dma_enabled && pending != 0u);
    if (dma_enabled && (interrupt_enable_mask & event_flags) != 0u)
        kinetis_data_dma_request(data, (uint8_t)(45u + instance));
}

bool kinetis_data_internal_dac_read(KinetisData* data, uint8_t instance, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value) {
    const uint32_t register_offset = address - data->dac_base[instance];
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, DAC_REGISTER_SIZE))
        return false;
    if (data->profile->id == KINETIS_PROFILE_MKV10Z1287 && byte_count != 1u)
        return false;
    *output_value = kinetis_data_internal_load_bytes(data->dac[instance].registers, register_offset,
                                                     byte_count);
    if (data->profile->id == KINETIS_PROFILE_MKV10Z1287 && register_offset == 0x21u)
        *output_value &= ~0x10u;
    return true;
}

bool kinetis_data_internal_dac_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value) {
    KinetisDac* dac = &data->dac[instance];
    const uint32_t register_offset = address - data->dac_base[instance];
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, DAC_REGISTER_SIZE))
        return false;
    const bool mkv10 = data->profile->id == KINETIS_PROFILE_MKV10Z1287;
    if (mkv10 && byte_count != 1u)
        return false;
    if (register_offset == 0x20u) {
        dac->registers[0x20] &= mkv10 ? (uint8_t)write_value : (uint8_t)~write_value;
        kinetis_data_internal_dac_flags(data, instance, 0u);
        return true;
    }
    if (mkv10 && register_offset == 0x21u) {
        const bool software_trigger = (write_value & 0x30u) == 0x30u;
        dac->registers[0x21] = (uint8_t)write_value & 0xefu;
        if (software_trigger && (dac->registers[0x22] & 1u) != 0u)
            kinetis_data_internal_dac_trigger(data, instance, true);
    } else {
        kinetis_data_internal_store_bytes(dac->registers, register_offset, byte_count,
                                          mkv10 && register_offset == 0x22u
                                              ? write_value & 0x85u
                                          : mkv10 && register_offset == 0x23u
                                              ? write_value & 0x11u
                                              : write_value);
    }
    if (mkv10 && register_offset <= 3u && (dac->registers[0x21] & 4u) != 0u)
        kinetis_data_internal_dac_flags(data, instance, 4u);
    if (register_offset <= 0x1fu || register_offset >= 0x21u)
        kinetis_data_internal_dac_update_output(data, instance);
    return true;
}

static uint8_t cmp_level(const KinetisCmp* cmp, uint8_t input_selection) {
    if (input_selection == 7u && (cmp->registers[4] & 0x80u) != 0u)
        return cmp->registers[4] & 0x3fu;
    return cmp->inputs[input_selection & 7u];
}

static bool cmp_raw_output(const KinetisCmp* cmp) {
    const bool comparator_enabled = (cmp->registers[1] & 1u) != 0u;
    const uint8_t positive_input = (cmp->registers[5] >> 3) & 7u;
    const uint8_t negative_input = cmp->registers[5] & 7u;
    bool comparator_high =
        comparator_enabled && cmp_level(cmp, positive_input) > cmp_level(cmp, negative_input);
    if ((cmp->registers[1] & 0x08u) != 0u)
        comparator_high = !comparator_high;
    return comparator_high;
}

static void cmp_commit_output(KinetisData* data, uint8_t instance, bool comparator_high) {
    KinetisCmp* cmp = &data->cmp[instance];
    const bool previously_high = (cmp->registers[3] & 1u) != 0u;
    if (comparator_high)
        cmp->registers[3] |= 1u;
    else
        cmp->registers[3] &= 0xfeu;
    const bool rising = !previously_high && comparator_high;
    const bool falling = previously_high && !comparator_high;
    if (rising)
        cmp->registers[3] |= 4u;
    if (falling)
        cmp->registers[3] |= 2u;
    const bool interrupt_pending =
        ((cmp->registers[3] & 0x10u) != 0u && (cmp->registers[3] & 4u) != 0u) ||
        ((cmp->registers[3] & 0x08u) != 0u && (cmp->registers[3] & 2u) != 0u);
    kinetis_data_internal_interrupt(
        data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_CMP0 + instance), interrupt_pending);
    if ((rising || falling) && (cmp->registers[3] & 0x40u) != 0u)
        kinetis_data_dma_request(data, (uint8_t)(42u + instance));
}

static void cmp_filter_sample(KinetisData* data, uint8_t instance, bool sample) {
    KinetisCmp* cmp = &data->cmp[instance];
    const uint8_t filter_count = (cmp->registers[0] >> 4u) & 7u;
    if (filter_count <= 1u) {
        cmp->filter_samples = 0u;
        cmp_commit_output(data, instance, sample);
        return;
    }
    if (cmp->filter_samples == 0u || cmp->filter_candidate != sample) {
        cmp->filter_candidate = sample;
        cmp->filter_samples = 1u;
        return;
    }
    if (cmp->filter_samples < filter_count)
        cmp->filter_samples++;
    if (cmp->filter_samples == filter_count)
        cmp_commit_output(data, instance, sample);
}

static void cmp_initialize_filter(KinetisData* data, uint8_t instance) {
    KinetisCmp* cmp = &data->cmp[instance];
    cmp->registers[3] &= 0xfeu;
    const bool interrupt_pending =
        ((cmp->registers[3] & 0x10u) != 0u && (cmp->registers[3] & 4u) != 0u) ||
        ((cmp->registers[3] & 0x08u) != 0u && (cmp->registers[3] & 2u) != 0u);
    kinetis_data_internal_interrupt(
        data, (KinetisDataInterrupt)(KINETIS_DATA_INTERRUPT_CMP0 + instance),
        interrupt_pending);
}

void kinetis_data_internal_cmp_evaluate(KinetisData* data, uint8_t instance) {
    KinetisCmp* cmp = &data->cmp[instance];
    if ((cmp->registers[1] & 0x20u) != 0u)
        return;
    if (data->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
        (data->stop_mode == KINETIS_DATA_STOP_VLLS0 ||
         ((data->stop_mode == KINETIS_DATA_STOP_VLLS1 ||
           data->stop_mode == KINETIS_DATA_STOP_VLLS3) &&
          (cmp->registers[1] & 0x10u) != 0u)))
        return;
    const bool sample_mode = (cmp->registers[1] & 0x80u) != 0u;
    const bool window_mode = (cmp->registers[1] & 0x40u) != 0u;
    const uint8_t filter_count = (cmp->registers[0] >> 4u) & 7u;
    if ((cmp->registers[1] & 1u) == 0u ||
        (!sample_mode && !window_mode && (filter_count == 0u || cmp->registers[2] == 0u)))
        cmp_commit_output(data, instance, cmp_raw_output(cmp));
}

void kinetis_data_internal_cmp_advance(KinetisData* data, uint8_t instance,
                                       uint32_t core_cycles) {
    KinetisCmp* cmp = &data->cmp[instance];
    if (cmp->trigger_pending) {
        if (core_cycles < cmp->trigger_cycles) {
            cmp->trigger_cycles -= core_cycles;
            return;
        }
        cmp->trigger_cycles = 0u;
        cmp->trigger_pending = false;
        cmp_commit_output(data, instance, cmp_raw_output(cmp));
        return;
    }
    if ((cmp->registers[1] & 0x20u) != 0u)
        return;
    if (data->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
        data->stop_mode != KINETIS_DATA_STOP_NONE)
        return;
    if ((cmp->registers[1] & 0x81u) != 1u || !data->bus_clock_running ||
        data->core_clock_hz == 0u || data->bus_clock_hz == 0u)
        return;
    const uint64_t accumulated = cmp->filter_remainder +
                                 (uint64_t)core_cycles * data->bus_clock_hz;
    const uint64_t bus_cycles = accumulated / data->core_clock_hz;
    cmp->filter_remainder = accumulated % data->core_clock_hz;
    if (bus_cycles == 0u)
        return;
    const bool window_mode = (cmp->registers[1] & 0x40u) != 0u;
    if (window_mode && cmp->sample_input)
        cmp->window_latch = cmp_raw_output(cmp);
    const uint8_t filter_period = cmp->registers[2];
    const uint8_t filter_count = (cmp->registers[0] >> 4u) & 7u;
    if (!window_mode && (filter_count == 0u || filter_period == 0u))
        return;
    if (window_mode && (filter_count == 0u || filter_period == 0u)) {
        if (cmp->sample_input)
            cmp_commit_output(data, instance, cmp->window_latch);
        return;
    }
    const uint64_t filter_bus_cycles = (uint64_t)cmp->filter_bus_cycles + bus_cycles;
    uint64_t sample_count = filter_bus_cycles / filter_period;
    cmp->filter_bus_cycles = (uint32_t)(filter_bus_cycles % filter_period);
    if (sample_count > 7u)
        sample_count = 7u;
    for (uint64_t sample = 0u; sample < sample_count; sample++)
        cmp_filter_sample(data, instance, window_mode ? cmp->window_latch : cmp_raw_output(cmp));
}

void kinetis_data_cmp_trigger(KinetisData* data, uint32_t delay_cycles) {
    if (data == NULL)
        return;
    for (uint8_t instance = 0u; instance < data->cmp_count; instance++) {
        KinetisCmp* cmp = &data->cmp[instance];
        if ((cmp->registers[1] & 0x21u) != 0x21u)
            continue;
        cmp->trigger_cycles = delay_cycles;
        cmp->trigger_pending = true;
    }
}

void kinetis_data_internal_cmp_set_sample(KinetisData* data, uint8_t instance, bool high) {
    KinetisCmp* cmp = &data->cmp[instance];
    const bool rising = high && !cmp->sample_input;
    cmp->sample_input = high;
    if (!rising || (cmp->registers[1] & 0xc1u) != 0x81u)
        return;
    if ((cmp->registers[0] & 0x70u) == 0u) {
        cmp_commit_output(data, instance, false);
        return;
    }
    cmp_filter_sample(data, instance, cmp_raw_output(cmp));
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
    if (register_offset <= 2u && register_offset + byte_count > 0u) {
        cmp->filter_remainder = 0u;
        cmp->filter_bus_cycles = 0u;
        cmp->filter_samples = 0u;
        cmp->filter_candidate = false;
        cmp->window_latch = false;
        cmp->trigger_cycles = 0u;
        cmp->trigger_pending = false;
        const uint8_t filter_count = (cmp->registers[0] >> 4u) & 7u;
        const bool filtered = (cmp->registers[1] & 1u) != 0u && filter_count != 0u &&
                              ((cmp->registers[1] & 0x80u) != 0u || cmp->registers[2] != 0u);
        if (filtered)
            cmp_initialize_filter(data, instance);
    }
    kinetis_data_internal_cmp_evaluate(data, instance);
    return true;
}
