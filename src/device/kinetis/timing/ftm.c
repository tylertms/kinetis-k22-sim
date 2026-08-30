#include "internal.h"

static uint8_t ftm_irq_for_instance(const KinetisTiming* timing, uint8_t instance) {
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287) {
        static const uint8_t interrupts[KINETIS_FTM_COUNT] = {17u, 18u, 19u, 22u, 24u, 26u};
        return interrupts[instance];
    }
    return instance == 3u ? IRQ_FTM3 : IRQ_FTM0 + instance;
}

static uint32_t ftm_system_clock_hz(const KinetisTiming* timing) {
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287)
        return kinetis_timing_system_clock_running(timing) ? timing->core_clock_hz : 0u;
    return kinetis_timing_bus_clock_running(timing) ? timing->bus_clock_hz : 0u;
}

static uint8_t ftm_dma_source_for_channel(const KinetisTiming* timing, uint8_t module,
                                          uint8_t channel) {
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287) {
        static const uint8_t bases[KINETIS_FTM_COUNT] = {24u, 32u, 34u, 36u, 30u, 56u};
        if (module == 3u && channel >= 4u)
            return (uint8_t)(50u + channel);
        return bases[module] + channel;
    }
    static const uint8_t bases[4] = {20u, 28u, 30u, 32u};
    return bases[module] + channel;
}

void kinetis_timing_internal_ftm_dma_complete(KinetisTiming* timing, uint8_t request_source) {
    for (uint8_t instance = 0u; instance < KINETIS_FTM_COUNT; instance++) {
        KinetisFtmState* ftm = &timing->ftm[instance];
        const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
        for (uint8_t channel = 0u; channel < channels; channel++) {
            if (ftm_dma_source_for_channel(timing, instance, channel) == request_source &&
                (ftm->channel_sc[channel] & 1u) != 0u) {
                ftm->channel_sc[channel] &= ~0x80u;
                ftm->channel_flag_read[channel] = false;
                kinetis_timing_internal_update_ftm_irq(timing, instance);
                return;
            }
        }
    }
}

static uint8_t ftm_trigger_bit_for_channel(uint8_t channel) {
    static const uint8_t bits[6] = {4u, 5u, 0u, 1u, 2u, 3u};
    return channel < 6u ? bits[channel] : UINT8_MAX;
}

KinetisPeripheralId kinetis_timing_internal_ftm_peripheral(uint8_t instance) {
    static const KinetisPeripheralId peripherals[KINETIS_FTM_COUNT] = {
        KINETIS_PERIPHERAL_FTM0, KINETIS_PERIPHERAL_FTM1, KINETIS_PERIPHERAL_FTM2,
        KINETIS_PERIPHERAL_FTM3, KINETIS_PERIPHERAL_FTM4, KINETIS_PERIPHERAL_FTM5,
    };
    return instance < KINETIS_FTM_COUNT ? peripherals[instance] : KINETIS_PERIPHERAL_COUNT;
}

uint8_t kinetis_timing_internal_ftm_channel_count(const KinetisTiming* timing, uint8_t instance) {
    if (timing == NULL || timing->profile == NULL || instance >= KINETIS_FTM_COUNT)
        return 0u;
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287)
        return instance == 0u || instance == 3u ? 6u : 2u;
    if (instance == 0u)
        return timing->profile->id == KINETIS_PROFILE_MKV30F12810 ? 6u : 8u;
    return instance == 3u ? 8u : 2u;
}

static bool is_ftm_quadrature_enabled(const KinetisFtmState* ftm) {
    return ftm->quadrature_capable && (ftm->registers[11] & 1u) != 0u;
}

static bool ftm_has_independent_quadrature(const KinetisTiming* timing, uint8_t instance) {
    return timing->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
           (instance == 1u || instance == 2u || instance == 5u);
}

static uint8_t ftm_debug_mode(const KinetisFtmState* ftm) {
    return (uint8_t)((ftm->registers[12] >> 6u) & 3u);
}

static bool ftm_stopped_by_debug(const KinetisTiming* timing, const KinetisFtmState* ftm) {
    return timing->debug_halted && ftm_debug_mode(ftm) != 3u;
}

static bool ftm_global_time_base_running(const KinetisTiming* timing, uint8_t instance) {
    if (timing->profile->id != KINETIS_PROFILE_MKV10Z1287 ||
        (timing->ftm[instance].registers[12] & (1u << 9u)) == 0u)
        return true;
    const uint8_t source = instance < 3u ? 0u : 3u;
    return (timing->ftm[source].registers[12] & (1u << 10u)) != 0u;
}

bool kinetis_timing_internal_ftm_bypass_buffers(const KinetisTiming* timing,
                                                const KinetisFtmState* ftm) {
    return (ftm->sc & 0x18u) == 0u || ftm_stopped_by_debug(timing, ftm);
}

static bool is_ftm_combine_mode(const KinetisFtmState* ftm, uint8_t channel) {
    const uint8_t pair_shift = (uint8_t)((channel / 2u) * 8u);
    const uint32_t pair = ftm->registers[4] >> pair_shift;
    return (ftm->sc & (1u << 5u)) == 0u && !is_ftm_quadrature_enabled(ftm) && (pair & 1u) != 0u &&
           (pair & 4u) == 0u;
}

static bool is_ftm_complementary_mode(const KinetisFtmState* ftm, uint8_t channel) {
    const uint8_t pair_shift = (uint8_t)((channel / 2u) * 8u);
    const uint32_t pair = ftm->registers[4] >> pair_shift;
    const uint8_t first_channel = channel & 0xfeu;
    const bool output_compare =
        (ftm->sc & (1u << 5u)) == 0u && (ftm->channel_sc[first_channel] & 0x30u) == 0x10u;
    return !is_ftm_quadrature_enabled(ftm) && (pair & 2u) != 0u && (pair & 4u) == 0u &&
           !output_compare;
}

void kinetis_timing_internal_set_ftm_routed_input(KinetisTiming* timing, uint8_t instance,
                                                  uint8_t channel, bool input_high) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (ftm->channel_input[channel] != input_high) {
        ftm->channel_input[channel] = input_high;
        ftm->channel_input_age[channel] = 0u;
    }
}

void kinetis_timing_internal_set_ftm_routed_fault(KinetisTiming* timing, uint8_t instance,
                                                  uint8_t input_index, bool input_high) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (ftm->fault_input[input_index] != input_high) {
        ftm->fault_input[input_index] = input_high;
        ftm->fault_input_age[input_index] = 0u;
    }
}

static uint32_t ftm_mux_register(const KinetisTiming* timing, uint8_t instance) {
    return instance < 3u ? timing->sim_sopt4 : timing->sim_sopt6;
}

void kinetis_timing_internal_refresh_ftm_pin_routes(KinetisTiming* timing) {
    for (uint8_t instance = 0u; instance < KINETIS_FTM_COUNT; instance++) {
        const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
        for (uint8_t channel = 0u; channel < channels; channel++) {
            bool pin_route = true;
            if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287 && channel == 0u &&
                (instance == 1u || instance == 2u || instance == 4u || instance == 5u)) {
                const uint8_t shift = instance == 1u || instance == 4u ? 18u : 20u;
                pin_route = ((ftm_mux_register(timing, instance) >> shift) & 3u) == 0u;
            }
            if (pin_route)
                kinetis_timing_internal_set_ftm_routed_input(
                    timing, instance, channel, timing->ftm_pin_input[instance][channel]);
        }
    }
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
        (timing->sim_sopt4 & (1u << 22u)) != 0u) {
        const bool input = timing->ftm_pin_input[2][1] != timing->ftm_pin_input[2][0] !=
                           timing->ftm_pin_input[1][1];
        kinetis_timing_internal_set_ftm_routed_input(timing, 2u, 1u, input);
    }
    for (uint8_t instance = 0u; instance < KINETIS_FTM_COUNT; instance++) {
        for (uint8_t input = 0u; input < 4u; input++) {
            bool pin_route = true;
            if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287 && input == 0u) {
                const uint8_t bit = instance % 3u == 0u ? 0u : (uint8_t)(instance % 3u + 1u);
                pin_route = (ftm_mux_register(timing, instance) & (1u << bit)) == 0u;
            } else if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287 && input == 1u) {
                pin_route = instance == 0u && (timing->sim_sopt4 & 2u) == 0u;
            }
            if (pin_route)
                kinetis_timing_internal_set_ftm_routed_fault(
                    timing, instance, input, timing->ftm_fault_pin[instance][input]);
        }
    }
}

bool kinetis_timing_set_ftm_input(KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                  bool input_high) {
    if (timing == NULL || timing->profile == NULL || instance >= KINETIS_FTM_COUNT ||
        channel >= kinetis_timing_internal_ftm_channel_count(timing, instance))
        return false;
    const KinetisPeripheralId peripheral = kinetis_timing_internal_ftm_peripheral(instance);
    if (!kinetis_timing_internal_has(timing, peripheral))
        return false;
    timing->ftm_pin_input[instance][channel] = input_high;
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287)
        kinetis_timing_internal_refresh_ftm_pin_routes(timing);
    else
        kinetis_timing_internal_set_ftm_routed_input(timing, instance, channel, input_high);
    return true;
}

bool kinetis_timing_set_ftm_quadrature_input(KinetisTiming* timing, uint8_t instance,
                                             uint8_t phase, bool input_high) {
    if (timing == NULL || timing->profile == NULL || instance >= KINETIS_FTM_COUNT || phase >= 2u)
        return false;
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (!ftm->quadrature_capable)
        return false;
    if (!ftm_has_independent_quadrature(timing, instance))
        return kinetis_timing_set_ftm_input(timing, instance, phase, input_high);
    if (ftm->quadrature_input[phase] != input_high) {
        ftm->quadrature_input[phase] = input_high;
        ftm->quadrature_input_age[phase] = 0u;
    }
    return true;
}

bool kinetis_timing_set_ftm_fault(KinetisTiming* timing, uint8_t instance, uint8_t input_index,
                                  bool input_high) {
    if (timing == NULL || timing->profile == NULL || instance >= KINETIS_FTM_COUNT ||
        input_index >= 4u)
        return false;
    const KinetisPeripheralId peripheral = kinetis_timing_internal_ftm_peripheral(instance);
    if (!kinetis_timing_internal_has(timing, peripheral))
        return false;
    timing->ftm_fault_pin[instance][input_index] = input_high;
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287)
        kinetis_timing_internal_refresh_ftm_pin_routes(timing);
    else
        kinetis_timing_internal_set_ftm_routed_fault(timing, instance, input_index, input_high);
    return true;
}

bool kinetis_timing_trigger_ftm_hardware(KinetisTiming* timing, uint8_t instance, uint8_t trigger) {
    if (timing == NULL || timing->profile == NULL || instance >= KINETIS_FTM_COUNT ||
        trigger >= 3u)
        return false;
    const KinetisPeripheralId peripheral = kinetis_timing_internal_ftm_peripheral(instance);
    if (!kinetis_timing_internal_has(timing, peripheral))
        return false;
    if (trigger == 0u && timing->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
        (timing->sim_sopt8 & (1u << instance)) != 0u)
        return true;
    timing->ftm[instance].hardware_trigger_pending_mask |= (uint8_t)(1u << trigger);
    return true;
}

static bool ftm_output_before_deadtime(const KinetisFtmState* ftm, uint8_t channel) {
    const uint8_t pair_shift = (uint8_t)((channel / 2u) * 8u);
    const uint32_t pair = ftm->registers[4] >> pair_shift;
    const bool combined = is_ftm_combine_mode(ftm, channel);
    const bool complementary = is_ftm_complementary_mode(ftm, channel);
    const uint8_t first_channel = channel & 0xfeu;
    bool output = ftm->channel_output[channel];
    if ((combined || complementary) && (ftm->channel_sc[channel] & 0x0cu) != 0u) {
        output = ftm->channel_output[first_channel];
        const bool inverted_pair = (ftm->registers[15] & (1u << (channel / 2u))) != 0u;
        if (complementary && (((channel & 1u) != 0u) != inverted_pair))
            output = !output;
    }
    const bool dual_capture = (pair & 4u) != 0u;
    const bool software_enabled = (ftm->registers[16] & (1u << channel)) != 0u;
    if (!is_ftm_quadrature_enabled(ftm) && !dual_capture && software_enabled) {
        output = (ftm->registers[16] & (1u << (channel + 8u))) != 0u;
        const bool pair_software_enabled =
            (ftm->registers[16] & (3u << first_channel)) == (3u << first_channel);
        if ((channel & 1u) != 0u && complementary && pair_software_enabled && output &&
            (ftm->registers[16] & (1u << (first_channel + 8u))) != 0u)
            output = false;
    }
    return output;
}

static bool is_ftm_deadtime_enabled(const KinetisFtmState* ftm, uint8_t channel) {
    const uint8_t pair_shift = (uint8_t)((channel / 2u) * 8u);
    return is_ftm_complementary_mode(ftm, channel) &&
           (ftm->registers[4] & (1u << (pair_shift + 4u))) != 0u &&
           (ftm->registers[5] & 0x3fu) != 0u;
}

uint8_t kinetis_timing_internal_ftm_fault_mode(const KinetisFtmState* ftm) {
    return (uint8_t)((ftm->registers[0] >> 5u) & 3u);
}

static bool ftm_fault_channel_enabled(const KinetisFtmState* ftm, uint8_t channel) {
    const uint8_t mode = kinetis_timing_internal_ftm_fault_mode(ftm);
    const uint8_t shift = (uint8_t)((channel / 2u) * 8u + 6u);
    return mode != 0u && (ftm->registers[4] & (1u << shift)) != 0u &&
           (mode != 1u || (channel & 1u) == 0u);
}

static bool ftm_pin_output(const KinetisTiming* timing, uint8_t instance, uint8_t channel) {
    const KinetisFtmState* ftm = &timing->ftm[instance];
    if (timing->debug_halted && ftm_debug_mode(ftm) == 1u)
        return (ftm->registers[7] & (1u << channel)) != 0u;
    if (timing->debug_halted && ftm_debug_mode(ftm) == 2u)
        return ftm->debug_output[channel];
    if (is_ftm_quadrature_enabled(ftm))
        return false;
    bool output = is_ftm_deadtime_enabled(ftm, channel) ? ftm->channel_deadtime_output[channel]
                                                        : ftm_output_before_deadtime(ftm, channel);
    if ((ftm->registers[3] & (1u << channel)) != 0u)
        output = false;
    if (ftm->fault_output_active && ftm_fault_channel_enabled(ftm, channel))
        output = false;
    if ((ftm->registers[7] & (1u << channel)) != 0u)
        output = !output;
    return output;
}

static bool ftm_carrier_output(const KinetisTiming* timing, uint32_t option) {
    const uint8_t selection = (uint8_t)((option >> 8u) & 3u);
    if (selection == 0u)
        return ftm_pin_output(timing, 1u, 1u);
    if (selection == 1u)
        return (timing->lptmr_csr & 1u) != 0u && timing->lptmr_prescaler_output;
    if (selection == 2u)
        return ftm_pin_output(timing, 5u, 1u);
    return false;
}

bool kinetis_timing_get_ftm_output(const KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                   bool* output_high) {
    if (timing == NULL || output_high == NULL || timing->profile == NULL ||
        instance >= KINETIS_FTM_COUNT ||
        channel >= kinetis_timing_internal_ftm_channel_count(timing, instance))
        return false;
    const KinetisPeripheralId peripheral = kinetis_timing_internal_ftm_peripheral(instance);
    if (!kinetis_timing_internal_has(timing, peripheral))
        return false;
    bool output = ftm_pin_output(timing, instance, channel);
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287) {
        const uint32_t option = instance < 3u ? timing->sim_sopt8 : timing->sim_sopt9;
        const bool modulatable = instance == 0u || instance == 3u || instance == 2u || instance == 4u;
        const uint8_t output_bit = instance == 0u || instance == 3u ? channel
                                                                    : (uint8_t)(channel + 6u);
        if (modulatable && (option & (1u << (16u + output_bit))) != 0u)
            output = output && ftm_carrier_output(timing, option);
    }
    *output_high = output;
    return true;
}

void kinetis_timing_internal_ftm_capture_debug_outputs(KinetisTiming* timing) {
    for (uint8_t instance = 0u; instance < KINETIS_FTM_COUNT; instance++) {
        KinetisFtmState* ftm = &timing->ftm[instance];
        const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
        for (uint8_t channel = 0u; channel < channels; channel++)
            kinetis_timing_get_ftm_output(timing, instance, channel, &ftm->debug_output[channel]);
    }
}

void kinetis_timing_internal_update_ftm_irq(const KinetisTiming* timing, uint8_t instance) {
    const KinetisFtmState* ftm = &timing->ftm[instance];
    bool asserted = (ftm->sc & 0xc0u) == 0xc0u;
    const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
    for (uint8_t channel = 0u; channel < channels; channel++)
        asserted = asserted || (ftm->channel_sc[channel] & 0xc0u) == 0xc0u;
    asserted = asserted || ((ftm->registers[0] & 0x80u) != 0u && (ftm->registers[8] & 0x80u) != 0u);
    kinetis_timing_internal_set_irq(timing, ftm_irq_for_instance(timing, instance), asserted);
}

void kinetis_timing_internal_ftm_trigger(KinetisTiming* timing, uint8_t instance) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    ftm->registers[6] |= 0x80u;
    ftm->trigger_flag_read = false;
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287)
        kinetis_timing_internal_trigger(timing, KINETIS_TIMING_TRIGGER_FTM_OUTPUT, instance, 0u,
                                        instance);
    kinetis_timing_internal_trigger_adc_alternate(timing, (uint8_t)(8u + instance));
    kinetis_timing_internal_trigger_pdb_input(timing, (uint8_t)(8u + instance));
}

static bool ftm_gate(const KinetisTiming* timing, uint8_t instance) {
    if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287)
        return (timing->sim_scgc6 & (1u << (instance < 3u ? 24u + instance : 3u + instance))) !=
               0u;
    if (instance == 3u) {
        if (timing->profile->id == KINETIS_PROFILE_MK22FN1M012 ||
            timing->profile->id == KINETIS_PROFILE_MK22FX51212)
            return (timing->sim_scgc3 & (1u << 25u)) != 0;
        return (timing->sim_scgc6 & (1u << 6u)) != 0;
    }
    return (timing->sim_scgc6 & (1u << (24u + instance))) != 0;
}

uint8_t kinetis_timing_internal_ftm_active_fault_mask(const KinetisFtmState* ftm) {
    if (kinetis_timing_internal_ftm_fault_mode(ftm) == 0u)
        return 0u;
    uint8_t active = 0u;
    const uint8_t enabled = (uint8_t)ftm->registers[10] & 0x0fu;
    for (uint8_t input = 0u; input < 4u; input++) {
        if ((enabled & (1u << input)) != 0u && ftm->fault_filtered_input[input])
            active |= (uint8_t)(1u << input);
    }
    return active;
}

void kinetis_timing_internal_ftm_update_fault_status(KinetisFtmState* ftm) {
    const uint32_t flags = ftm->registers[8] & 0x0fu;
    ftm->registers[8] &= 0x4fu;
    if (kinetis_timing_internal_ftm_active_fault_mask(ftm) != 0u)
        ftm->registers[8] |= 0x20u;
    if (flags != 0u)
        ftm->registers[8] |= 0x80u;
}

static void ftm_detect_fault(KinetisTiming* timing, uint8_t instance, uint8_t input) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    const uint8_t bit = (uint8_t)(1u << input);
    ftm->registers[8] |= bit;
    ftm->fault_flags_read_mask &= (uint8_t)~bit;
    ftm->fault_aggregate_read = false;
    ftm->fault_output_active = true;
    ftm->fault_release_pending = false;
    kinetis_timing_internal_ftm_update_fault_status(ftm);
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static bool ftm_fault_processing_active(const KinetisFtmState* ftm) {
    return (kinetis_timing_internal_ftm_fault_mode(ftm) != 0u &&
            (ftm->registers[10] & 0x0fu) != 0u) ||
           ftm->fault_output_active;
}

static void ftm_advance_fault_inputs(KinetisTiming* timing, uint8_t instance, uint32_t cycles) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (!ftm_gate(timing, instance))
        return;
    const uint64_t filter_ticks =
        kinetis_timing_internal_clock_ticks(&ftm->fault_remainder, cycles,
                                            ftm_system_clock_hz(timing), timing->core_clock_hz);
    const uint8_t enabled_fault_mask = kinetis_timing_internal_ftm_fault_mode(ftm) == 0u
                                           ? 0u
                                           : (uint8_t)ftm->registers[10] & 0x0fu;
    const uint8_t filter_enable_mask = (uint8_t)(ftm->registers[10] >> 4u) & 0x0fu;
    const uint8_t filter_length = (uint8_t)(ftm->registers[10] >> 8u) & 0x0fu;

    for (uint8_t fault_input_index = 0u; fault_input_index < 4u; fault_input_index++) {
        const uint8_t input_mask = (uint8_t)(1u << fault_input_index);
        if ((enabled_fault_mask & input_mask) == 0u) {
            ftm->fault_filtered_input[fault_input_index] = false;
            ftm->fault_input_age[fault_input_index] = 0u;
            continue;
        }
        const bool active_polarity = (ftm->registers[13] & input_mask) != 0u;
        const bool input_active = ftm->fault_input[fault_input_index] != active_polarity;
        if (input_active == ftm->fault_filtered_input[fault_input_index]) {
            ftm->fault_input_age[fault_input_index] = 0u;
            continue;
        }
        const uint32_t filter_threshold =
            (filter_enable_mask & input_mask) != 0u && filter_length != 0u ? 4u + filter_length
                                                                           : 3u;
        ftm->fault_input_age[fault_input_index] += (uint32_t)filter_ticks;
        if (ftm->fault_input_age[fault_input_index] < filter_threshold)
            continue;
        ftm->fault_filtered_input[fault_input_index] = input_active;
        ftm->fault_input_age[fault_input_index] = 0u;
        if (input_active)
            ftm_detect_fault(timing, instance, fault_input_index);
    }
    kinetis_timing_internal_ftm_update_fault_status(ftm);
    if (ftm->fault_output_active && kinetis_timing_internal_ftm_fault_mode(ftm) == 3u &&
        kinetis_timing_internal_ftm_active_fault_mask(ftm) == 0u)
        ftm->fault_release_pending = true;
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static void ftm_fault_cycle_boundary(KinetisTiming* timing, uint8_t instance, bool new_cycle) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (!new_cycle || !ftm->fault_release_pending)
        return;
    if (kinetis_timing_internal_ftm_fault_mode(ftm) == 3u)
        ftm->registers[8] &= ~0x0fu;
    ftm->fault_output_active = false;
    ftm->fault_release_pending = false;
    ftm->fault_flags_read_mask = 0u;
    ftm->fault_aggregate_read = false;
    kinetis_timing_internal_ftm_update_fault_status(ftm);
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static uint64_t ftm_phase_crossing_count(uint32_t current_phase, uint64_t elapsed_ticks,
                                         uint32_t cycle_period, uint32_t target_phase) {
    const uint32_t ticks_to_target = target_phase > current_phase
                                         ? target_phase - current_phase
                                         : cycle_period - (current_phase - target_phase);
    return elapsed_ticks < ticks_to_target ? 0u
                                           : 1u + (elapsed_ticks - ticks_to_target) / cycle_period;
}

static void ftm_channel_event(KinetisTiming* timing, uint8_t instance, uint8_t channel) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    const uint8_t debug_mode = ftm_debug_mode(ftm);
    if (timing->debug_halted && (debug_mode == 1u || debug_mode == 2u))
        return;
    ftm->channel_sc[channel] |= 1u << 7u;
    ftm->channel_flag_read[channel] = false;
    if ((ftm->channel_sc[channel] & 1u) != 0)
        kinetis_timing_internal_request_dma(
            timing, ftm_dma_source_for_channel(timing, instance, channel));
    const uint8_t trigger_bit = ftm_trigger_bit_for_channel(channel);
    if (trigger_bit != UINT8_MAX && (ftm->registers[6] & (1u << trigger_bit)) != 0u)
        kinetis_timing_internal_ftm_trigger(timing, instance);
}

static bool ftm_pair_mode_disabled(const KinetisFtmState* ftm, uint8_t channel) {
    const uint8_t pair_shift = (uint8_t)((channel / 2u) * 8u);
    return ((ftm->registers[4] >> pair_shift) & 5u) == 0u;
}

static uint32_t ftm_up_count_position(const KinetisFtmState* ftm, uint16_t value) {
    return (uint16_t)(value - ftm->initial);
}

static uint32_t ftm_up_count_period(const KinetisFtmState* ftm) {
    return ftm_up_count_position(ftm, ftm->modulo) + 1u;
}

static bool ftm_up_count_contains(const KinetisFtmState* ftm, uint16_t value) {
    return ftm_up_count_position(ftm, value) < ftm_up_count_period(ftm);
}

static void ftm_combine_pwm_advance(KinetisFtmState* ftm, uint8_t channel) {
    if ((channel & 1u) != 0u || !is_ftm_combine_mode(ftm, channel))
        return;
    const uint8_t edge_mode = (uint8_t)((ftm->channel_sc[channel] >> 2u) & 3u);
    if (edge_mode == 0u)
        return;
    const uint16_t first_channel_compare = ftm->channel_value[channel];
    const uint16_t second_channel_compare = ftm->channel_value[channel + 1u];
    const uint32_t first_position = ftm_up_count_position(ftm, first_channel_compare);
    const uint32_t second_position = ftm_up_count_position(ftm, second_channel_compare);
    const uint32_t counter_position = ftm_up_count_position(ftm, ftm->counter);
    const bool first_compare_valid = ftm_up_count_contains(ftm, first_channel_compare);
    const bool second_compare_valid = ftm_up_count_contains(ftm, second_channel_compare);
    bool output_active = false;
    if (first_compare_valid) {
        if (second_compare_valid)
            output_active = first_position < second_position && counter_position >= first_position &&
                            counter_position < second_position;
        else
            output_active = counter_position >= first_position;
    }
    ftm->channel_output[channel] = edge_mode == 2u ? output_active : !output_active;
}

bool kinetis_timing_internal_ftm_output_compare_mode(const KinetisFtmState* ftm, uint8_t channel) {
    return (ftm->sc & (1u << 5u)) == 0u && (ftm->channel_sc[channel] & 0x30u) == 0x10u &&
           !is_ftm_quadrature_enabled(ftm) && ftm_pair_mode_disabled(ftm, channel);
}

static bool ftm_edge_aligned_pwm_mode(const KinetisFtmState* ftm, uint8_t channel) {
    return (ftm->sc & (1u << 5u)) == 0u && (ftm->channel_sc[channel] & 0x20u) != 0u &&
           !is_ftm_quadrature_enabled(ftm) && ftm_pair_mode_disabled(ftm, channel);
}

static bool ftm_center_aligned_pwm_mode(const KinetisFtmState* ftm, uint8_t channel) {
    return (ftm->sc & (1u << 5u)) != 0u && !is_ftm_quadrature_enabled(ftm) &&
           ftm_pair_mode_disabled(ftm, channel);
}

static void ftm_output_compare_match(KinetisFtmState* ftm, uint8_t channel, uint64_t match_count) {
    if (match_count == 0u || !kinetis_timing_internal_ftm_output_compare_mode(ftm, channel))
        return;
    switch ((ftm->channel_sc[channel] >> 2u) & 3u) {
    case 1u:
        if ((match_count & 1u) != 0u)
            ftm->channel_output[channel] = !ftm->channel_output[channel];
        break;
    case 2u:
        ftm->channel_output[channel] = false;
        break;
    case 3u:
        ftm->channel_output[channel] = true;
        break;
    default:
        break;
    }
}

static void ftm_edge_aligned_pwm_advance(KinetisFtmState* ftm, uint8_t channel, uint64_t matches,
                                         uint64_t overflows) {
    if (!ftm_edge_aligned_pwm_mode(ftm, channel))
        return;
    const uint8_t edges = (uint8_t)((ftm->channel_sc[channel] >> 2u) & 3u);
    if (edges == 0u)
        return;
    const bool high_true = edges == 2u;
    const uint16_t compare = ftm->channel_value[channel];
    if (overflows != 0u) {
        const uint32_t compare_position = ftm_up_count_position(ftm, compare);
        const uint32_t counter_position = ftm_up_count_position(ftm, ftm->counter);
        const bool active = !ftm_up_count_contains(ftm, compare) ||
                            (compare_position != 0u && counter_position < compare_position);
        ftm->channel_output[channel] = high_true ? active : !active;
    } else if (matches != 0u) {
        ftm->channel_output[channel] = !high_true;
    }
}

static void ftm_center_aligned_pwm_advance(KinetisFtmState* ftm, uint8_t channel, uint64_t matches,
                                           uint64_t ticks) {
    if (ticks == 0u || !ftm_center_aligned_pwm_mode(ftm, channel))
        return;
    const uint8_t edges = (uint8_t)((ftm->channel_sc[channel] >> 2u) & 3u);
    if (edges == 0u)
        return;
    const bool high_true = edges == 2u;
    const uint16_t compare = ftm->channel_value[channel];
    bool active;
    if (compare <= ftm->initial || (compare & 0x8000u) != 0u)
        active = false;
    else if (compare >= ftm->modulo)
        active = true;
    else {
        if (matches == 0u)
            return;
        active = ftm->counter < compare || (ftm->counting_down && ftm->counter == compare);
    }
    ftm->channel_output[channel] = high_true ? active : !active;
}

static void ftm_overflow(KinetisTiming* timing, uint8_t instance, uint64_t overflow_count) {
    if (overflow_count == 0u)
        return;
    KinetisFtmState* ftm = &timing->ftm[instance];
    const uint8_t cycle = (uint8_t)((ftm->registers[12] & 0x1fu) + 1u);
    const uint8_t first_set =
        ftm->overflow_count == 0u ? 1u : (uint8_t)(cycle - ftm->overflow_count + 1u);
    if (overflow_count >= first_set) {
        ftm->sc |= 1u << 7u;
        ftm->overflow_flag_read = false;
    }
    ftm->overflow_count = (uint8_t)((ftm->overflow_count + overflow_count % cycle) % cycle);
}

static void ftm_apply_modulo(KinetisFtmState* ftm);
static void ftm_apply_channel_value(KinetisFtmState* ftm, uint8_t channel);
static bool ftm_pair_synchronization_enabled(const KinetisFtmState* ftm, uint8_t channel);
static void ftm_loading_point(KinetisFtmState* ftm, bool minimum, bool maximum,
                              uint8_t channel_matches);

static void ftm_apply_counter_change_values(KinetisFtmState* ftm, uint8_t channels) {
    const bool enhanced = (ftm->registers[0] & 1u) != 0u;
    for (uint8_t channel = 0u; channel < channels; channel++) {
        if (ftm->channel_value_pending[channel] &&
            kinetis_timing_internal_ftm_output_compare_mode(ftm, channel) &&
            (!enhanced || !ftm_pair_synchronization_enabled(ftm, channel)))
            ftm_apply_channel_value(ftm, channel);
    }
}

static void ftm_apply_legacy_boundary_values(KinetisFtmState* ftm, uint8_t channels) {
    if ((ftm->registers[0] & 1u) != 0u)
        return;
    ftm_apply_modulo(ftm);
    for (uint8_t channel = 0u; channel < channels; channel++) {
        if (ftm_edge_aligned_pwm_mode(ftm, channel) || ftm_center_aligned_pwm_mode(ftm, channel) ||
            is_ftm_combine_mode(ftm, channel))
            ftm_apply_channel_value(ftm, channel);
    }
}

static void advance_ftm_up_down(KinetisTiming* timing, uint8_t instance, uint64_t ticks) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    const uint32_t first = ftm->initial;
    const uint32_t last = ftm->modulo;
    if (last <= first) {
        ftm->counter = (uint16_t)first;
        ftm->counting_down = false;
        ftm_overflow(timing, instance, ticks);
        if ((ftm->registers[6] & (1u << 6u)) != 0u)
            kinetis_timing_internal_ftm_trigger(timing, instance);
        kinetis_timing_internal_update_ftm_irq(timing, instance);
        return;
    }
    const uint32_t span = last - first;
    const uint32_t period = span * 2u;
    const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
    ftm_apply_counter_change_values(ftm, channels);
    uint32_t phase;
    if (ftm->counter < first || ftm->counter > last) {
        phase = 0u;
    } else if (ftm->counting_down) {
        phase = span + last - ftm->counter;
    } else {
        phase = ftm->counter - first;
    }
    uint64_t matches[8] = {0};
    uint8_t match_mask = 0u;
    for (uint8_t channel = 0u; channel < channels; channel++) {
        const uint32_t compare = ftm->channel_value[channel];
        if (compare <= first || compare >= last)
            continue;
        const uint32_t up_phase = compare - first;
        const uint32_t down_phase = period - up_phase;
        matches[channel] = ftm_phase_crossing_count(phase, ticks, period, up_phase) +
                           ftm_phase_crossing_count(phase, ticks, period, down_phase);
        if (matches[channel] != 0u) {
            match_mask |= (uint8_t)(1u << channel);
            ftm_channel_event(timing, instance, channel);
        }
    }
    const uint64_t maximum_points = ftm_phase_crossing_count(phase, ticks, period, span + 1u);
    const uint64_t minimum_points = ftm_phase_crossing_count(phase, ticks, period, 0u);
    ftm_overflow(timing, instance, maximum_points);
    ftm_fault_cycle_boundary(timing, instance, minimum_points != 0u);
    if (minimum_points != 0u && (ftm->registers[6] & (1u << 6u)) != 0u)
        kinetis_timing_internal_ftm_trigger(timing, instance);
    phase = (uint32_t)(((uint64_t)phase + ticks) % period);
    if (phase <= span) {
        ftm->counter = (uint16_t)(first + phase);
        ftm->counting_down = false;
    } else {
        ftm->counter = (uint16_t)(last - (phase - span));
        ftm->counting_down = true;
    }
    if (maximum_points != 0u)
        ftm_apply_legacy_boundary_values(ftm, channels);
    for (uint8_t channel = 0u; channel < channels; channel++)
        ftm_center_aligned_pwm_advance(ftm, channel, matches[channel], ticks);
    ftm_loading_point(ftm, minimum_points != 0u, maximum_points != 0u, match_mask);
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static void advance_ftm_ticks(KinetisTiming* timing, uint8_t instance, uint64_t ticks) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (ticks == 0u)
        return;
    if ((ftm->sc & (1u << 5u)) != 0u) {
        advance_ftm_up_down(timing, instance, ticks);
        return;
    }
    const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
    ftm_apply_counter_change_values(ftm, channels);
    const uint32_t period = ftm_up_count_period(ftm);
    const uint32_t counter_position = ftm_up_count_position(ftm, ftm->counter);
    const uint32_t start_position = counter_position < period ? counter_position : 0u;
    const uint64_t relative = (uint64_t)start_position + ticks;
    const uint64_t overflows = relative / period;
    uint64_t matches[8] = {0};
    uint8_t match_mask = 0u;
    for (uint8_t channel = 0; channel < channels; channel++) {
        const uint16_t compare = ftm->channel_value[channel];
        const uint32_t compare_position = ftm_up_count_position(ftm, compare);
        const uint32_t distance = compare_position > start_position
                                      ? compare_position - start_position
                                      : period - (start_position - compare_position);
        const bool output_compare = kinetis_timing_internal_ftm_output_compare_mode(ftm, channel);
        const bool edge_aligned = ftm_edge_aligned_pwm_mode(ftm, channel);
        const bool combined = is_ftm_combine_mode(ftm, channel);
        const bool valid_compare =
            output_compare ? compare_position < period
                           : compare_position != 0u && compare_position < period;
        const bool combine_compare = combined && compare_position < period;
        if ((output_compare || edge_aligned || combine_compare) &&
            (valid_compare || combine_compare) && ticks >= distance) {
            matches[channel] = 1u + (ticks - distance) / period;
            match_mask |= (uint8_t)(1u << channel);
            ftm_channel_event(timing, instance, channel);
        }
    }
    ftm->counter = (uint16_t)(ftm->initial + relative % period);
    ftm_overflow(timing, instance, overflows);
    ftm_fault_cycle_boundary(timing, instance, overflows != 0u);
    if (overflows != 0u && (ftm->registers[6] & (1u << 6u)) != 0u)
        kinetis_timing_internal_ftm_trigger(timing, instance);
    if (overflows != 0u)
        ftm_apply_legacy_boundary_values(ftm, channels);
    for (uint8_t channel = 0u; channel < channels; channel++) {
        ftm_output_compare_match(ftm, channel, matches[channel]);
        ftm_edge_aligned_pwm_advance(ftm, channel, matches[channel], overflows);
        ftm_combine_pwm_advance(ftm, channel);
    }
    ftm_loading_point(ftm, overflows != 0u, overflows != 0u, match_mask);
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static void advance_ftm_counter(KinetisTiming* timing, uint8_t instance, uint32_t cycles) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (is_ftm_quadrature_enabled(ftm))
        return;
    const uint8_t clock_select = (uint8_t)((ftm->sc >> 3u) & 3u);
    if (!ftm_gate(timing, instance) || clock_select == 0u || clock_select == 3u ||
        ftm_stopped_by_debug(timing, ftm) || !ftm_global_time_base_running(timing, instance))
        return;
    uint32_t source_hz = clock_select == 1u ? ftm_system_clock_hz(timing)
                                            : kinetis_timing_internal_fixed_clock_hz(timing);
    source_hz >>= ftm->sc & 7u;
    const uint64_t ticks = kinetis_timing_internal_clock_ticks(&ftm->remainder, cycles, source_hz,
                                                               timing->core_clock_hz);
    advance_ftm_ticks(timing, instance, ticks);
}

bool kinetis_timing_set_ftm_clock_input(KinetisTiming* timing, uint8_t input_index,
                                        bool input_high) {
    if (timing == NULL || timing->profile == NULL ||
        input_index >= (timing->profile->id == KINETIS_PROFILE_MKV10Z1287 ? 3u : 2u))
        return false;
    const bool previous = timing->ftm_clock_input[input_index];
    timing->ftm_clock_input[input_index] = input_high;
    if (previous || !input_high)
        return true;
    for (uint8_t instance = 0u; instance < KINETIS_FTM_COUNT; instance++) {
        KinetisFtmState* ftm = &timing->ftm[instance];
        const KinetisPeripheralId peripheral = kinetis_timing_internal_ftm_peripheral(instance);
        uint8_t selected_input =
            (uint8_t)((timing->sim_sopt4 >> (24u + instance)) & 1u);
        if (timing->profile->id == KINETIS_PROFILE_MKV10Z1287) {
            const uint32_t options = instance < 3u ? timing->sim_sopt4 : timing->sim_sopt6;
            selected_input =
                (uint8_t)((options >> (24u + 2u * (instance % 3u))) & 3u);
        }
        if (selected_input != input_index || !kinetis_timing_internal_has(timing, peripheral) ||
            !ftm_gate(timing, instance) || ((ftm->sc >> 3u) & 3u) != 3u ||
            is_ftm_quadrature_enabled(ftm) || ftm_stopped_by_debug(timing, ftm) ||
            !ftm_global_time_base_running(timing, instance))
            continue;
        ftm->external_clock_edges++;
        const uint8_t divider = (uint8_t)(1u << (ftm->sc & 7u));
        if (ftm->external_clock_edges == divider) {
            ftm->external_clock_edges = 0u;
            advance_ftm_ticks(timing, instance, 1u);
        }
    }
    return true;
}

static uint32_t ftm_channel_input_threshold(const KinetisFtmState* ftm, uint8_t channel) {
    if (channel >= 4u)
        return 3u;
    const uint8_t filter = (uint8_t)((ftm->registers[9] >> (channel * 4u)) & 15u);
    return filter == 0u ? 3u : 4u + (uint32_t)filter * 4u;
}

static uint32_t ftm_quadrature_input_threshold(const KinetisFtmState* ftm, uint8_t phase) {
    if ((ftm->registers[11] & (1u << (7u - phase))) == 0u)
        return 3u;
    const uint8_t filter = (uint8_t)((ftm->registers[9] >> (phase * 4u)) & 15u);
    return filter == 0u ? 3u : 4u + (uint32_t)filter * 4u;
}

static uint32_t ftm_routed_input_threshold(const KinetisTiming* timing, uint8_t instance,
                                           uint8_t channel) {
    const KinetisFtmState* ftm = &timing->ftm[instance];
    if (!ftm_has_independent_quadrature(timing, instance) &&
        is_ftm_quadrature_enabled(ftm) && channel < 2u)
        return ftm_quadrature_input_threshold(ftm, channel);
    return ftm_channel_input_threshold(ftm, channel);
}

bool kinetis_timing_internal_ftm_input_capture_mode(const KinetisTiming* timing, uint8_t instance,
                                                    uint8_t channel) {
    const KinetisFtmState* ftm = &timing->ftm[instance];
    const uint8_t pair_shift = (uint8_t)((channel / 2u) * 8u);
    const uint32_t pair = ftm->registers[4] >> pair_shift;
    return (!is_ftm_quadrature_enabled(ftm) ||
            ftm_has_independent_quadrature(timing, instance)) &&
           (ftm->sc & (1u << 5u)) == 0u &&
           (ftm->channel_sc[channel] & 0x30u) == 0u && (ftm->channel_sc[channel] & 0x0cu) != 0u &&
           (pair & 5u) == 0u;
}

static bool ftm_dual_capture_mode(const KinetisFtmState* ftm, uint8_t channel) {
    const uint8_t shift = (uint8_t)((channel / 2u) * 8u);
    return (ftm->sc & (1u << 5u)) == 0u && ((ftm->registers[4] >> shift) & 5u) == 4u;
}

static bool ftm_dual_capture_edge(uint32_t channel_sc, bool current) {
    const uint8_t edge = (uint8_t)((channel_sc >> 2u) & 3u);
    return edge == (current ? 1u : 2u);
}

static void ftm_dual_capture_input(KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                   bool previous, bool current) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    const uint8_t first = channel & 0xfeu;
    const uint8_t pair = first / 2u;
    const uint8_t shift = (uint8_t)(pair * 8u);
    if ((channel & 1u) != 0u || previous == current ||
        (ftm->registers[4] & (1u << (shift + 3u))) == 0u)
        return;
    const bool continuous = (ftm->channel_sc[first] & (1u << 4u)) != 0u;
    if (!ftm->dual_capture_waiting_final[pair]) {
        if (!continuous &&
            ((ftm->channel_sc[first] | ftm->channel_sc[first + 1u]) & 0x80u) != 0u)
            return;
        if (!ftm_dual_capture_edge(ftm->channel_sc[first], current))
            return;
        ftm->dual_capture_first[pair] = ftm->counter;
        ftm->dual_capture_waiting_final[pair] = true;
        ftm_channel_event(timing, instance, first);
    } else {
        if (!ftm_dual_capture_edge(ftm->channel_sc[first + 1u], current))
            return;
        ftm->channel_value[first] = ftm->dual_capture_first[pair];
        ftm->dual_capture_second[pair] = ftm->counter;
        ftm->dual_capture_waiting_final[pair] = false;
        ftm_channel_event(timing, instance, first + 1u);
        if (!continuous)
            ftm->registers[4] &= ~(1u << (shift + 3u));
    }
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static void ftm_quadrature_step(KinetisTiming* timing, uint8_t instance, bool increment) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if ((ftm->sc & 0x18u) == 0u || ftm_stopped_by_debug(timing, ftm) ||
        !ftm_global_time_base_running(timing, instance))
        return;
    const uint16_t first = ftm->initial;
    const uint16_t last = ftm->modulo >= first ? ftm->modulo : UINT16_MAX;
    const uint16_t counter = ftm->counter < first || ftm->counter > last ? first : ftm->counter;
    bool overflow = false;
    if (increment) {
        ftm->registers[11] |= 4u;
        if (counter == last) {
            ftm->counter = first;
            ftm->registers[11] |= 2u;
            overflow = true;
        } else {
            ftm->counter = (uint16_t)(counter + 1u);
        }
    } else {
        ftm->registers[11] &= ~4u;
        if (counter == first) {
            ftm->counter = last;
            ftm->registers[11] &= ~2u;
            overflow = true;
        } else {
            ftm->counter = (uint16_t)(counter - 1u);
        }
    }
    if (overflow) {
        ftm_overflow(timing, instance, 1u);
        ftm_fault_cycle_boundary(timing, instance, true);
        if ((ftm->registers[6] & (1u << 6u)) != 0u)
            kinetis_timing_internal_ftm_trigger(timing, instance);
    }
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static void ftm_quadrature_transition(KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                      bool previous, bool current) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    const bool* inputs = ftm_has_independent_quadrature(timing, instance)
                             ? ftm->quadrature_filtered_input
                             : ftm->channel_filtered_input;
    const bool polarity = (ftm->registers[11] & (1u << (5u - channel))) != 0u;
    const bool before = previous != polarity;
    const bool after = current != polarity;
    if (before == after)
        return;
    const bool phase_a = inputs[0] != ((ftm->registers[11] & 0x20u) != 0u);
    const bool phase_b = inputs[1] != ((ftm->registers[11] & 0x10u) != 0u);
    if ((ftm->registers[11] & 8u) != 0u) {
        if (channel == 0u && !before && after)
            ftm_quadrature_step(timing, instance, phase_b);
        return;
    }
    const bool increment = channel == 0u ? after != phase_b : phase_a == after;
    ftm_quadrature_step(timing, instance, increment);
}

static void ftm_capture_input(KinetisTiming* timing, uint8_t instance, uint8_t channel,
                              bool previous, bool current) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (!ftm_has_independent_quadrature(timing, instance) &&
        is_ftm_quadrature_enabled(ftm) && channel < 2u) {
        ftm_quadrature_transition(timing, instance, channel, previous, current);
        return;
    }
    if (ftm_dual_capture_mode(ftm, channel)) {
        ftm_dual_capture_input(timing, instance, channel, previous, current);
        return;
    }
    const uint8_t edges = (uint8_t)((ftm->channel_sc[channel] >> 2u) & 3u);
    const bool edge_selected = current ? (edges & 1u) != 0u : (edges & 2u) != 0u;
    if (previous == current || !edge_selected ||
        !kinetis_timing_internal_ftm_input_capture_mode(timing, instance, channel))
        return;
    ftm->channel_value[channel] = ftm->counter;
    ftm_channel_event(timing, instance, channel);
    if ((ftm->channel_sc[channel] & 2u) != 0u) {
        ftm->counter = ftm->initial;
        ftm->counting_down = false;
        ftm->overflow_count = 0u;
        ftm->remainder = 0u;
        if ((ftm->registers[6] & (1u << 6u)) != 0u)
            kinetis_timing_internal_ftm_trigger(timing, instance);
    }
    kinetis_timing_internal_update_ftm_irq(timing, instance);
}

static void ftm_apply_outmask(KinetisFtmState* ftm) {
    if (ftm->outmask_pending) {
        ftm->registers[3] = ftm->outmask_buffer;
        ftm->outmask_pending = false;
    }
}

static void ftm_apply_invctrl(KinetisFtmState* ftm) {
    if (ftm->invctrl_pending) {
        ftm->registers[15] = ftm->invctrl_buffer;
        ftm->invctrl_pending = false;
    }
}

static void ftm_apply_swoctrl(KinetisFtmState* ftm) {
    if (ftm->swoctrl_pending) {
        ftm->registers[16] = ftm->swoctrl_buffer;
        ftm->swoctrl_pending = false;
    }
}

static void ftm_apply_modulo(KinetisFtmState* ftm) {
    if (ftm->modulo_pending) {
        ftm->modulo = ftm->modulo_buffer;
        ftm->modulo_pending = false;
    }
}

static void ftm_apply_initial(KinetisFtmState* ftm) {
    if (ftm->initial_pending) {
        ftm->initial = ftm->initial_buffer;
        ftm->initial_pending = false;
    }
}

static void ftm_apply_channel_value(KinetisFtmState* ftm, uint8_t channel) {
    if (ftm->channel_value_pending[channel]) {
        ftm->channel_value[channel] = ftm->channel_value_buffer[channel];
        ftm->channel_value_pending[channel] = false;
    }
}

static bool ftm_pair_synchronization_enabled(const KinetisFtmState* ftm, uint8_t channel) {
    return (ftm->registers[4] & (1u << ((channel / 2u) * 8u + 5u))) != 0u;
}

static void ftm_apply_synchronized_write_buffers(KinetisFtmState* ftm, bool enhanced) {
    if ((ftm->registers[0] & 1u) == 0u)
        return;
    ftm_apply_modulo(ftm);
    if (enhanced && (ftm->registers[14] & (1u << 2u)) != 0u)
        ftm_apply_initial(ftm);
    for (uint8_t channel = 0u; channel < 8u; channel++) {
        if (ftm_pair_synchronization_enabled(ftm, channel))
            ftm_apply_channel_value(ftm, channel);
    }
}

void kinetis_timing_internal_ftm_sync_bit(KinetisTiming* timing, uint8_t instance) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if ((ftm->registers[1] & (1u << 4u)) == 0u)
        return;
    ftm_apply_modulo(ftm);
    ftm_apply_initial(ftm);
    for (uint8_t channel = 0u; channel < 8u; channel++)
        ftm_apply_channel_value(ftm, channel);
    ftm_apply_outmask(ftm);
    ftm_apply_invctrl(ftm);
    ftm_apply_swoctrl(ftm);
    ftm->counter = ftm->initial;
    ftm->counting_down = false;
    ftm->overflow_count = 0u;
    ftm->remainder = 0u;
    ftm->hardware_sync_pending = false;
    ftm->hardware_trigger_pending_mask &= 0xfeu;
}

static void ftm_apply_intermediate_load(KinetisFtmState* ftm) {
    ftm_apply_modulo(ftm);
    if ((ftm->registers[14] & (1u << 2u)) != 0u)
        ftm_apply_initial(ftm);
    for (uint8_t channel = 0u; channel < 8u; channel++) {
        if (ftm_pair_synchronization_enabled(ftm, channel))
            ftm_apply_channel_value(ftm, channel);
    }
    ftm->registers[17] &= ~(1u << 9u);
}

static void ftm_loading_point(KinetisFtmState* ftm, bool minimum, bool maximum,
                              uint8_t channel_matches) {
    if (ftm->software_sync_pending && (((ftm->registers[1] & 1u) != 0u && minimum) ||
                                       ((ftm->registers[1] & 2u) != 0u && maximum))) {
        const bool enhanced = (ftm->registers[14] & (1u << 7u)) != 0u;
        const bool write_buffers = enhanced ? (ftm->registers[14] & (1u << 9u)) != 0u : true;
        if (write_buffers)
            ftm_apply_synchronized_write_buffers(ftm, enhanced);
        ftm->registers[1] &= ~0x80u;
        ftm->software_sync_pending = false;
    }
    if (ftm->hardware_sync_pending && (((ftm->registers[1] & 1u) != 0u && minimum) ||
                                       ((ftm->registers[1] & 2u) != 0u && maximum))) {
        const bool enhanced = (ftm->registers[14] & (1u << 7u)) != 0u;
        if (!enhanced || (ftm->registers[14] & (1u << 17u)) != 0u)
            ftm_apply_synchronized_write_buffers(ftm, enhanced);
        ftm->hardware_sync_pending = false;
    }
    const bool intermediate = maximum || (channel_matches & (uint8_t)ftm->registers[17]) != 0u;
    if ((ftm->registers[17] & (1u << 9u)) != 0u && intermediate)
        ftm_apply_intermediate_load(ftm);
}

static void ftm_apply_system_clock_updates(KinetisFtmState* ftm) {
    if ((ftm->registers[1] & 8u) == 0u)
        ftm_apply_outmask(ftm);
    if ((ftm->registers[14] & (1u << 4u)) == 0u)
        ftm_apply_invctrl(ftm);
    if ((ftm->registers[14] & (1u << 5u)) == 0u)
        ftm_apply_swoctrl(ftm);
    if (ftm->initial_pending &&
        (((ftm->registers[0] & 1u) == 0u) || (ftm->registers[14] & (1u << 2u)) == 0u))
        ftm_apply_initial(ftm);
}

void kinetis_timing_internal_ftm_apply_software_sync(KinetisFtmState* ftm) {
    const uint32_t synconf = ftm->registers[14];
    const bool enhanced = (synconf & (1u << 7u)) != 0u;
    if ((synconf & (1u << 12u)) != 0u && (synconf & (1u << 5u)) != 0u)
        ftm_apply_swoctrl(ftm);
    if ((synconf & (1u << 11u)) != 0u && (synconf & (1u << 4u)) != 0u)
        ftm_apply_invctrl(ftm);
    if ((enhanced && (synconf & (1u << 10u)) != 0u && (ftm->registers[1] & 8u) != 0u) ||
        (!enhanced && (ftm->registers[0] & 8u) == 0u && (ftm->registers[1] & 8u) != 0u))
        ftm_apply_outmask(ftm);
    const bool reset_counter =
        (enhanced && (synconf & (1u << 8u)) != 0u) || (!enhanced && (ftm->registers[1] & 4u) != 0u);
    if (reset_counter) {
        const bool write_buffers = enhanced ? (synconf & (1u << 9u)) != 0u : true;
        if (write_buffers)
            ftm_apply_synchronized_write_buffers(ftm, enhanced);
        ftm->counter = ftm->initial;
        ftm->counting_down = false;
        ftm->overflow_count = 0u;
        ftm->remainder = 0u;
        ftm->registers[1] &= ~0x80u;
        ftm->software_sync_pending = false;
    } else {
        ftm->software_sync_pending = true;
    }
}

static void ftm_apply_hardware_sync(KinetisFtmState* ftm) {
    const uint32_t synconf = ftm->registers[14];
    const bool enhanced = (synconf & (1u << 7u)) != 0u;
    if (!enhanced) {
        if ((ftm->registers[1] & 8u) != 0u)
            ftm_apply_outmask(ftm);
        const bool reset_counter = (ftm->registers[1] & 4u) != 0u;
        const bool synchronize_buffers = (ftm->registers[0] & 8u) == 0u;
        if (reset_counter) {
            if (synchronize_buffers)
                ftm_apply_synchronized_write_buffers(ftm, false);
            ftm->counter = ftm->initial;
            ftm->counting_down = false;
            ftm->overflow_count = 0u;
            ftm->remainder = 0u;
            ftm->hardware_sync_pending = false;
        } else {
            ftm->hardware_sync_pending = synchronize_buffers;
        }
        return;
    }
    if ((synconf & (1u << 20u)) != 0u && (synconf & (1u << 5u)) != 0u)
        ftm_apply_swoctrl(ftm);
    if ((synconf & (1u << 19u)) != 0u && (synconf & (1u << 4u)) != 0u)
        ftm_apply_invctrl(ftm);
    if ((synconf & (1u << 18u)) != 0u && (ftm->registers[1] & 8u) != 0u)
        ftm_apply_outmask(ftm);
    if ((synconf & (1u << 16u)) != 0u) {
        if ((synconf & (1u << 17u)) != 0u)
            ftm_apply_synchronized_write_buffers(ftm, true);
        ftm->counter = ftm->initial;
        ftm->counting_down = false;
        ftm->overflow_count = 0u;
        ftm->remainder = 0u;
        ftm->hardware_sync_pending = false;
    } else {
        ftm->hardware_sync_pending = true;
    }
}

static void ftm_process_hardware_triggers(KinetisTiming* timing, uint8_t instance) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (ftm->hardware_trigger_pending_mask == 0u || !ftm_gate(timing, instance))
        return;
    const uint8_t pending = ftm->hardware_trigger_pending_mask;
    ftm->hardware_trigger_pending_mask = 0u;
    const uint8_t enabled = (uint8_t)((ftm->registers[1] >> 4u) & 7u);
    const uint8_t detected = pending & enabled;
    if (detected == 0u)
        return;
    if ((ftm->registers[14] & 1u) == 0u)
        ftm->registers[1] &= ~((uint32_t)detected << 4u);
    ftm_apply_hardware_sync(ftm);
}

static bool ftm_has_deadtime(const KinetisFtmState* ftm, uint8_t channels) {
    for (uint8_t channel = 0u; channel < channels; channel += 2u) {
        if (is_ftm_deadtime_enabled(ftm, channel))
            return true;
    }
    return false;
}

static void ftm_advance_deadtime(KinetisTiming* timing, uint8_t instance, uint32_t cycles) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
    if (!ftm_gate(timing, instance))
        return;
    const uint8_t divider = (uint8_t)(ftm->registers[5] >> 6u);
    const uint8_t shift = divider < 2u ? 0u : divider == 2u ? 2u : 4u;
    const uint64_t ticks = kinetis_timing_internal_clock_ticks(
        &ftm->deadtime_remainder, cycles, ftm_system_clock_hz(timing) >> shift,
        timing->core_clock_hz);
    for (uint8_t channel = 0u; channel < channels; channel++) {
        const bool raw = ftm_output_before_deadtime(ftm, channel);
        if (!is_ftm_deadtime_enabled(ftm, channel)) {
            ftm->channel_deadtime_output[channel] = raw;
            ftm->channel_deadtime_remaining[channel] = 0u;
        } else if (!raw) {
            ftm->channel_deadtime_output[channel] = false;
            ftm->channel_deadtime_remaining[channel] = 0u;
        } else if (!ftm->channel_deadtime_output[channel] &&
                   ftm->channel_deadtime_remaining[channel] == 0u) {
            ftm->channel_deadtime_remaining[channel] = ftm->registers[5] & 0x3fu;
        } else if (!ftm->channel_deadtime_output[channel] && ticks != 0u) {
            const uint32_t remaining = ftm->channel_deadtime_remaining[channel];
            if (ticks >= remaining) {
                ftm->channel_deadtime_remaining[channel] = 0u;
                ftm->channel_deadtime_output[channel] = true;
            } else {
                ftm->channel_deadtime_remaining[channel] -= (uint32_t)ticks;
            }
        }
    }
}

void kinetis_timing_internal_advance_ftm(KinetisTiming* timing, uint8_t instance, uint32_t cycles) {
    KinetisFtmState* ftm = &timing->ftm[instance];
    ftm_apply_system_clock_updates(ftm);
    ftm_process_hardware_triggers(timing, instance);
    const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
    uint32_t remaining = cycles;
    while (remaining != 0u) {
        uint32_t segment =
            ftm_has_deadtime(ftm, channels) || ftm_fault_processing_active(ftm) ? 1u : remaining;
        for (uint8_t channel = 0u; channel < channels; channel++) {
            if (ftm->channel_input[channel] == ftm->channel_filtered_input[channel])
                continue;
            const uint32_t threshold = ftm_routed_input_threshold(timing, instance, channel);
            const uint32_t until_event = ftm->channel_input_age[channel] >= threshold
                                             ? 0u
                                             : threshold - ftm->channel_input_age[channel];
            if (until_event < segment)
                segment = until_event;
        }
        if (ftm_has_independent_quadrature(timing, instance) && is_ftm_quadrature_enabled(ftm)) {
            for (uint8_t phase = 0u; phase < 2u; phase++) {
                if (ftm->quadrature_input[phase] == ftm->quadrature_filtered_input[phase])
                    continue;
                const uint32_t threshold = ftm_quadrature_input_threshold(ftm, phase);
                const uint32_t until_event = ftm->quadrature_input_age[phase] >= threshold
                                                 ? 0u
                                                 : threshold - ftm->quadrature_input_age[phase];
                if (until_event < segment)
                    segment = until_event;
            }
        }
        ftm_advance_fault_inputs(timing, instance, segment);
        advance_ftm_counter(timing, instance, segment);
        ftm_advance_deadtime(timing, instance, segment);
        remaining -= segment;
        for (uint8_t channel = 0u; channel < channels; channel++) {
            if (ftm->channel_input[channel] == ftm->channel_filtered_input[channel])
                continue;
            ftm->channel_input_age[channel] += segment;
            if (ftm->channel_input_age[channel] <
                ftm_routed_input_threshold(timing, instance, channel))
                continue;
            const bool previous = ftm->channel_filtered_input[channel];
            ftm->channel_filtered_input[channel] = ftm->channel_input[channel];
            ftm->channel_input_age[channel] = 0u;
            ftm_capture_input(timing, instance, channel, previous,
                              ftm->channel_filtered_input[channel]);
        }
        if (ftm_has_independent_quadrature(timing, instance) && is_ftm_quadrature_enabled(ftm)) {
            for (uint8_t phase = 0u; phase < 2u; phase++) {
                if (ftm->quadrature_input[phase] == ftm->quadrature_filtered_input[phase])
                    continue;
                ftm->quadrature_input_age[phase] += segment;
                if (ftm->quadrature_input_age[phase] <
                    ftm_quadrature_input_threshold(ftm, phase))
                    continue;
                const bool previous = ftm->quadrature_filtered_input[phase];
                ftm->quadrature_filtered_input[phase] = ftm->quadrature_input[phase];
                ftm->quadrature_input_age[phase] = 0u;
                ftm_quadrature_transition(timing, instance, phase, previous,
                                          ftm->quadrature_filtered_input[phase]);
            }
        }
    }
}
