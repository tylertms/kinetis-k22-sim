#include "internal.h"

static uint32_t advance_pit_channel_ticks(KinetisTiming* timing, uint8_t pit_channel,
                                          uint64_t elapsed_ticks) {
    KinetisPitChannel* pit_channel_state = &timing->pit[pit_channel];
    if ((pit_channel_state->control & 1u) == 0 || elapsed_ticks == 0) {
        return 0;
    }
    const uint64_t ticks_to_expiration = (uint64_t)pit_channel_state->current + 1u;
    uint32_t expiration_count = 0;
    if (elapsed_ticks >= ticks_to_expiration) {
        elapsed_ticks -= ticks_to_expiration;
        expiration_count = 1u;
        const uint64_t reload_period = (uint64_t)pit_channel_state->load + 1u;
        const uint64_t repeated_expirations = elapsed_ticks / reload_period;
        expiration_count = repeated_expirations >= UINT32_MAX
                               ? UINT32_MAX
                               : expiration_count + (uint32_t)repeated_expirations;
        pit_channel_state->current =
            pit_channel_state->load - (uint32_t)(elapsed_ticks % reload_period);
        pit_channel_state->flag = true;
        if ((pit_channel_state->control & 2u) != 0) {
            kinetis_timing_internal_set_irq(timing, IRQ_PIT0 + pit_channel, true);
        }
        kinetis_timing_internal_trigger_dma(timing, pit_channel);
        kinetis_timing_internal_trigger_adc_alternate(timing, (uint8_t)(4u + pit_channel));
    } else {
        pit_channel_state->current -= (uint32_t)elapsed_ticks;
    }
    return expiration_count;
}

void kinetis_timing_internal_advance_pit(KinetisTiming* timing, uint32_t cycles) {
    if (!kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_PIT) ||
        (timing->sim_scgc6 & (1u << 23u)) == 0 || (timing->pit_mcr & 2u) != 0 ||
        ((timing->pit_mcr & 1u) != 0u && timing->debug_halted) ||
        !kinetis_timing_bus_clock_running(timing)) {
        return;
    }
    const uint64_t elapsed_ticks = kinetis_timing_internal_clock_ticks(
        &timing->pit_remainder, cycles, timing->bus_clock_hz, timing->core_clock_hz);
    uint64_t chained_ticks = elapsed_ticks;

    for (uint8_t pit_channel = 0; pit_channel < 4; pit_channel++) {
        if ((timing->pit[pit_channel].control & 4u) == 0) {
            chained_ticks = elapsed_ticks;
        }
        chained_ticks = advance_pit_channel_ticks(timing, pit_channel, chained_ticks);
    }
}

bool kinetis_timing_internal_pit_read(const KinetisTiming* timing, uint32_t address, uint8_t size,
                                      uint32_t* output_value) {
    if (size != 4) {
        return false;
    }
    if (address == PIT_BASE) {
        *output_value = timing->pit_mcr;
        return true;
    }
    if (address < PIT_CHANNEL_BASE || address >= PIT_CHANNEL_BASE + 0x40u) {
        return false;
    }
    const uint8_t channel = (uint8_t)((address - PIT_CHANNEL_BASE) / 0x10u);
    switch ((address - PIT_CHANNEL_BASE) & 0x0fu) {
    case 0:
        *output_value = timing->pit[channel].load;
        return true;
    case 4:
        *output_value = timing->pit[channel].current;
        return true;
    case 8:
        *output_value = timing->pit[channel].control;
        return true;
    case 12:
        *output_value = timing->pit[channel].flag ? 1u : 0u;
        return true;
    default:
        return false;
    }
}

bool kinetis_timing_internal_pit_write(KinetisTiming* timing, uint32_t address, uint8_t size,
                                       uint32_t write_value) {
    if (size != 4) {
        return false;
    }
    if (address == PIT_BASE) {
        timing->pit_mcr = write_value & 3u;
        return true;
    }
    if (address < PIT_CHANNEL_BASE || address >= PIT_CHANNEL_BASE + 0x40u) {
        return false;
    }
    const uint8_t channel = (uint8_t)((address - PIT_CHANNEL_BASE) / 0x10u);
    switch ((address - PIT_CHANNEL_BASE) & 0x0fu) {
    case 0:
        timing->pit[channel].load = write_value;
        return true;
    case 8: {
        KinetisPitChannel* pit = &timing->pit[channel];
        const bool was_enabled = (pit->control & 1u) != 0u;
        pit->control = write_value & (channel == 0u ? 3u : 7u);
        if (!was_enabled && (pit->control & 1u) != 0u)
            pit->current = pit->load;
        kinetis_timing_internal_set_irq(timing, IRQ_PIT0 + channel,
                                        pit->flag && (pit->control & 2u) != 0u);
        return true;
    }
    case 12:
        if ((write_value & 1u) != 0) {
            timing->pit[channel].flag = false;
            kinetis_timing_internal_set_irq(timing, IRQ_PIT0 + channel, false);
        }
        return true;
    default:
        return false;
    }
}

static uint32_t lptmr_clock_hz(const KinetisTiming* timing) {
    switch (timing->lptmr_psr & 3u) {
    case 0:
        return kinetis_timing_internal_mcgir_clock_hz(timing);
    case 1:
        return kinetis_timing_internal_lpo_clock_hz(timing);
    case 2:
        return kinetis_timing_internal_erclk32k_hz(timing);
    default:
        return kinetis_timing_internal_oscer_clock_hz(timing);
    }
}

static bool is_lptmr_running(const KinetisTiming* timing) {
    return kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_LPTMR0) &&
           (timing->sim_scgc5 & 1u) != 0 && (timing->lptmr_csr & 1u) != 0;
}

uint32_t kinetis_timing_lptmr_trigger_delay_cycles(const KinetisTiming* timing) {
    if (timing == NULL || timing->core_clock_hz == 0u)
        return 0u;
    const uint32_t source_hz = lptmr_clock_hz(timing);
    if (source_hz == 0u)
        return 0u;
    const uint32_t divider = (timing->lptmr_psr & 4u) != 0u
                                 ? 1u
                                 : 1u << (((timing->lptmr_psr >> 3u) & 15u) + 1u);
    const uint64_t numerator = (uint64_t)timing->core_clock_hz * divider;
    const uint64_t denominator = (uint64_t)source_hz * 2u;
    const uint64_t cycles = (numerator + denominator - 1u) / denominator;
    return cycles > UINT32_MAX ? UINT32_MAX : (uint32_t)cycles;
}

static void trigger_lptmr_compare(KinetisTiming* timing) {
    if (timing->signals.trigger != NULL)
        timing->signals.trigger(timing->signals.context, KINETIS_TIMING_TRIGGER_CMP, 0u, 0u,
                                0u);
    kinetis_timing_internal_trigger_adc_alternate(timing, 14u);
    kinetis_timing_internal_trigger_pdb_input(timing, 14u);
}

bool kinetis_timing_internal_lptmr_selected_active(const KinetisTiming* timing) {
    const uint8_t input_index = (uint8_t)((timing->lptmr_csr >> 4u) & 3u);
    if (input_index >= 3u)
        return false;
    const bool input_high = timing->lptmr_input[input_index];
    return (timing->lptmr_csr & 8u) == 0 ? input_high : !input_high;
}

static void advance_lptmr_counter(KinetisTiming* timing, uint64_t ticks) {
    if (ticks == 0)
        return;
    if (ticks <= timing->lptmr_sync_ticks) {
        timing->lptmr_sync_ticks -= (uint8_t)ticks;
        return;
    }
    ticks -= timing->lptmr_sync_ticks;
    timing->lptmr_sync_ticks = 0u;
    const uint32_t compare = timing->lptmr_cmr & 0xffffu;
    if ((timing->lptmr_csr & 4u) == 0) {
        const uint64_t period = (uint64_t)compare + 1u;
        const uint64_t accumulated_ticks = (uint64_t)timing->lptmr_counter + ticks;
        if (accumulated_ticks >= period) {
            timing->lptmr_csr |= 0x80u;
            if ((timing->lptmr_csr & 0x40u) != 0)
                kinetis_timing_internal_set_irq(
                    timing, kinetis_timing_internal_profile_irq(timing, IRQ_LPTMR, 28u), true);
            trigger_lptmr_compare(timing);
        }
        timing->lptmr_counter = (uint16_t)(accumulated_ticks % period);
        return;
    }
    const uint32_t distance = ((compare - timing->lptmr_counter) & 0xffffu) + 1u;
    if (ticks >= distance) {
        timing->lptmr_csr |= 0x80u;
        if ((timing->lptmr_csr & 0x40u) != 0)
            kinetis_timing_internal_set_irq(
                timing, kinetis_timing_internal_profile_irq(timing, IRQ_LPTMR, 28u), true);
        trigger_lptmr_compare(timing);
    }
    timing->lptmr_counter = (uint16_t)((uint64_t)timing->lptmr_counter + ticks);
}

static uint64_t lptmr_clock_ticks(uint64_t* remainder, uint32_t cycles, uint32_t source_hz,
                                  uint32_t core_hz, uint32_t divider, uint32_t multiplier) {
    if (source_hz == 0u || core_hz == 0u)
        return 0u;
    const uint64_t denominator = (uint64_t)core_hz * divider;
    const uint64_t accumulated =
        *remainder + (uint64_t)cycles * source_hz * multiplier;
    *remainder = accumulated % denominator;
    return accumulated / denominator;
}

static void advance_lptmr_prescaler(KinetisTiming* timing, uint32_t cycles,
                                     uint32_t source_hz, uint32_t divider) {
    const uint64_t half_edges =
        lptmr_clock_ticks(&timing->lptmr_prescaler_remainder, cycles, source_hz,
                          timing->core_clock_hz, divider, 2u);
    if ((half_edges & 1u) != 0u)
        timing->lptmr_prescaler_output = !timing->lptmr_prescaler_output;
}

static void sample_lptmr_filter(KinetisTiming* timing, uint32_t cycles) {
    const uint8_t prescale = (uint8_t)((timing->lptmr_psr >> 3u) & 15u);
    if (prescale == 0u)
        return;
    const uint64_t samples = kinetis_timing_internal_clock_ticks(
        &timing->lptmr_filter_remainder, cycles, lptmr_clock_hz(timing), timing->core_clock_hz);
    const bool active = kinetis_timing_internal_lptmr_selected_active(timing);
    if (active == timing->lptmr_observed_active) {
        timing->lptmr_filter_ticks = 0u;
        return;
    }
    const uint32_t threshold = 1u << prescale;
    const uint64_t accumulated_samples = (uint64_t)timing->lptmr_filter_ticks + samples;
    if (accumulated_samples < threshold) {
        timing->lptmr_filter_ticks = (uint32_t)accumulated_samples;
        return;
    }
    timing->lptmr_filter_ticks = 0u;
    timing->lptmr_observed_active = active;
    if (active)
        advance_lptmr_counter(timing, 1u);
}

void kinetis_timing_internal_advance_lptmr(KinetisTiming* timing, uint32_t cycles) {
    if (!is_lptmr_running(timing)) {
        return;
    }
    const uint32_t source_hz = lptmr_clock_hz(timing);
    const uint32_t divider = (timing->lptmr_psr & 4u) != 0u
                                 ? 1u
                                 : 1u << (((timing->lptmr_psr >> 3u) & 15u) + 1u);
    advance_lptmr_prescaler(timing, cycles, source_hz, divider);
    if ((timing->lptmr_csr & 2u) != 0) {
        if ((timing->lptmr_psr & 4u) == 0)
            sample_lptmr_filter(timing, cycles);
        return;
    }
    if (timing->debug_halted)
        return;
    const uint64_t ticks = lptmr_clock_ticks(&timing->lptmr_remainder, cycles, source_hz,
                                             timing->core_clock_hz, divider, 1u);
    advance_lptmr_counter(timing, ticks);
}

bool kinetis_timing_set_lptmr_input(KinetisTiming* timing, uint8_t input_index, bool input_high) {
    if (timing == NULL || timing->profile == NULL || input_index >= 3u ||
        !kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_LPTMR0))
        return false;
    timing->lptmr_input[input_index] = input_high;
    if (!is_lptmr_running(timing) || (timing->lptmr_csr & 2u) == 0 ||
        (timing->lptmr_psr & 4u) == 0 || ((timing->lptmr_csr >> 4u) & 3u) != input_index)
        return true;
    const bool active = kinetis_timing_internal_lptmr_selected_active(timing);
    if (active != timing->lptmr_observed_active) {
        timing->lptmr_observed_active = active;
        if (active)
            advance_lptmr_counter(timing, 1u);
    }
    return true;
}

uint32_t kinetis_timing_internal_rtc_access_reset(const KinetisTiming* timing) {
    return timing->profile->id == KINETIS_PROFILE_MK22FN1M012 ||
                   timing->profile->id == KINETIS_PROFILE_MK22FX51212
               ? 0xffffu
               : 0xffu;
}

void kinetis_timing_internal_update_rtc_irq(const KinetisTiming* timing) {
    const uint32_t enabled_flags = timing->rtc_ier & timing->rtc_sr & 7u;
    kinetis_timing_internal_set_irq(timing, IRQ_RTC, enabled_flags != 0u);
}

static uint32_t rtc_second_ticks(const KinetisTiming* timing) {
    const int8_t compensation = (int8_t)(timing->rtc_tcr >> 16u);
    return (uint32_t)(32768 - compensation);
}

static void rtc_complete_second(KinetisTiming* timing) {
    const uint8_t interval_counter = (uint8_t)(timing->rtc_tcr >> 24u);
    if (interval_counter == 0u) {
        timing->rtc_tcr = (timing->rtc_tcr & 0xffffu) | ((timing->rtc_tcr & 0xff00u) << 16u) |
                          ((timing->rtc_tcr & 0xffu) << 16u);
    } else {
        timing->rtc_tcr = (timing->rtc_tcr & 0xffffu) | ((uint32_t)(interval_counter - 1u) << 24u);
    }
    const bool overflow = timing->rtc_tsr == UINT32_MAX;
    if (overflow) {
        timing->rtc_tsr = 0u;
        timing->rtc_tpr = 0u;
        timing->rtc_subsecond_ticks = 0u;
        timing->rtc_sr |= 2u;
    } else {
        timing->rtc_tsr++;
    }
    kinetis_timing_internal_trigger_adc_alternate(timing, 13u);
    kinetis_timing_internal_set_irq(timing, IRQ_RTC_SECONDS, (timing->rtc_ier & 0x10u) != 0u);
    kinetis_timing_internal_set_irq(timing, IRQ_RTC_SECONDS, false);
    if (timing->rtc_tsr == timing->rtc_tar) {
        timing->rtc_sr |= 4u;
        kinetis_timing_internal_trigger_adc_alternate(timing, 12u);
    }
    kinetis_timing_internal_update_rtc_irq(timing);
}

void kinetis_timing_internal_advance_rtc(KinetisTiming* timing, uint32_t cycles) {
    if (!kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_RTC) ||
        (timing->rtc_cr & 0x100u) == 0u || (timing->rtc_sr & 0x10u) == 0 ||
        (timing->rtc_sr & 3u) != 0) {
        return;
    }
    const uint64_t ticks = kinetis_timing_internal_clock_ticks(
        &timing->rtc_remainder, cycles, timing->rtc_oscillator_hz, timing->core_clock_hz);
    uint64_t remaining = ticks;
    while (remaining != 0u && (timing->rtc_sr & 2u) == 0u) {
        const uint32_t second_ticks = rtc_second_ticks(timing);
        const uint32_t ticks_to_second = second_ticks - timing->rtc_subsecond_ticks;
        if (remaining < ticks_to_second) {
            timing->rtc_subsecond_ticks += (uint32_t)remaining;
            remaining = 0u;
        } else {
            remaining -= ticks_to_second;
            timing->rtc_subsecond_ticks = 0u;
            rtc_complete_second(timing);
        }
    }
    timing->rtc_tpr =
        (uint16_t)(timing->rtc_subsecond_ticks > 0x7fffu ? 0x7fffu : timing->rtc_subsecond_ticks);
}

static uint32_t pdb_divider(uint32_t sc) {
    static const uint16_t multipliers[4] = {1u, 10u, 20u, 40u};
    return (1u << ((sc >> 12u) & 7u)) * multipliers[(sc >> 2u) & 3u];
}

bool kinetis_timing_internal_pdb_auxiliary_offset(uint32_t offset) {
    return (offset >= 0x10u && offset <= 0x1cu && (offset & 3u) == 0) ||
           (offset >= 0x38u && offset <= 0x44u && (offset & 3u) == 0) ||
           (offset >= 0x150u && offset <= 0x15cu && (offset & 3u) == 0) ||
           (offset >= 0x190u && offset <= 0x198u && (offset & 3u) == 0);
}

bool kinetis_timing_internal_pdb_buffered_offset(uint32_t offset) {
    return offset == 0x18u || offset == 0x1cu || offset == 0x40u || offset == 0x44u ||
           offset == 0x154u || offset == 0x15cu || offset == 0x194u || offset == 0x198u;
}

void kinetis_timing_internal_load_pdb(KinetisTiming* timing, uint8_t instance) {
    timing->pdb_mod[instance] = timing->pdb_mod_buffer[instance];
    timing->pdb_idly[instance] = timing->pdb_idly_buffer[instance];
    for (uint32_t offset = 0x10u; offset < 0x1a0u; offset += 4u) {
        if (kinetis_timing_internal_pdb_buffered_offset(offset))
            timing->pdb_registers[instance][offset >> 2u] =
                timing->pdb_register_buffers[instance][offset >> 2u];
    }
    timing->pdb_sc[instance] &= ~1u;
}

void kinetis_timing_internal_refresh_pdb_irq(KinetisTiming* timing) {
    bool asserted = false;
    for (uint8_t instance = 0u; instance < 2u; instance++) {
        const KinetisPeripheralId peripheral =
            instance == 0u ? KINETIS_PERIPHERAL_PDB0 : KINETIS_PERIPHERAL_PDB1;
        if (!kinetis_timing_internal_has(timing, peripheral))
            continue;
        const bool delay_interrupt =
            (timing->pdb_sc[instance] & ((1u << 6u) | (1u << 5u) | (1u << 15u))) ==
            ((1u << 6u) | (1u << 5u));
        const bool sequence_error =
            (timing->pdb_sc[instance] & (1u << 17u)) != 0u &&
            ((timing->pdb_registers[instance][0x14u >> 2u] & 0xffu) != 0u ||
             (timing->pdb_registers[instance][0x3cu >> 2u] & 0xffu) != 0u);
        asserted |= delay_interrupt || sequence_error;
    }
    kinetis_timing_internal_set_irq(
        timing, kinetis_timing_internal_profile_irq(timing, IRQ_PDB, 29u), asserted);
}

static bool pdb_target_reached(uint16_t start, uint64_t ticks, uint16_t target, uint16_t modulo) {
    return target <= modulo && target > start && (uint64_t)target - start <= ticks;
}

static void pdb_assert_pretrigger(KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                  uint8_t pretrigger) {
    const uint32_t status_offset = 0x14u + (uint32_t)channel * 0x28u;
    if (timing->pdb_adc_locks[instance][channel] != 0u) {
        timing->pdb_registers[instance][status_offset >> 2u] |= 1u << pretrigger;
        kinetis_timing_internal_refresh_pdb_irq(timing);
        return;
    }
    timing->pdb_adc_locks[instance][channel] |= 1u << pretrigger;
    timing->pdb_channel_trigger_pending[instance][channel] |= 1u << pretrigger;
    timing->pdb_channel_trigger_cycles[instance][channel] = 1u;
}

static void pdb_bypass_triggers(KinetisTiming* timing, uint8_t instance) {
    for (uint8_t channel = 0u; channel < 2u; channel++) {
        const uint32_t control =
            timing->pdb_registers[instance][(0x10u + (uint32_t)channel * 0x28u) >> 2u];
        for (uint8_t pretrigger = 0u; pretrigger < 2u; pretrigger++) {
            const uint32_t enable = 1u << pretrigger;
            const uint32_t delayed = 1u << (8u + pretrigger);
            const uint32_t back_to_back = 1u << (16u + pretrigger);
            if ((control & enable) != 0u && (control & (delayed | back_to_back)) == 0u)
                pdb_assert_pretrigger(timing, instance, channel, pretrigger);
        }
    }
}

static void pdb_queue_delayed_trigger(KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                      uint8_t pretrigger) {
    timing->pdb_delayed_pending[instance][channel] |= 1u << pretrigger;
    timing->pdb_delayed_cycles[instance][channel][pretrigger] = 2u;
}

void kinetis_timing_internal_set_pdb_pulse(KinetisTiming* timing, uint8_t instance,
                                           uint8_t output, bool asserted) {
    if (timing->pdb_pulse_output[instance][output] == asserted)
        return;
    timing->pdb_pulse_output[instance][output] = asserted;
    if (timing->signals.pdb_pulse != NULL)
        timing->signals.pdb_pulse(timing->signals.context, instance, output, asserted);
}

static void pdb_counter_start_events(KinetisTiming* timing, uint8_t instance) {
    for (uint8_t channel = 0u; channel < 2u; channel++) {
        const uint32_t channel_base = 0x10u + (uint32_t)channel * 0x28u;
        const uint32_t control = timing->pdb_registers[instance][channel_base >> 2u];
        for (uint8_t pretrigger = 0u; pretrigger < 2u; pretrigger++) {
            const uint16_t delay = (uint16_t)timing->pdb_registers[instance]
                                                                [(channel_base + 8u +
                                                                  (uint32_t)pretrigger * 4u) >>
                                                                 2u];
            if (delay == 0u && (control & (1u << pretrigger)) != 0u &&
                (control & (1u << (8u + pretrigger))) != 0u &&
                (control & (1u << (16u + pretrigger))) == 0u) {
                pdb_queue_delayed_trigger(timing, instance, channel, pretrigger);
            }
        }
    }
    const uint32_t pulse_enable = timing->pdb_registers[instance][0x190u >> 2u];
    for (uint8_t output = 0u; output < 2u; output++) {
        if ((pulse_enable & (1u << output)) == 0u)
            continue;
        const uint32_t delays = timing->pdb_registers[instance][(0x194u + output * 4u) >> 2u];
        if ((delays >> 16u) == 0u)
            kinetis_timing_internal_set_pdb_pulse(timing, instance, output, true);
        if ((delays & 0xffffu) == 0u)
            kinetis_timing_internal_set_pdb_pulse(timing, instance, output, false);
    }
}

static void pdb_counter_events(KinetisTiming* timing, uint8_t instance, uint16_t start,
                               uint64_t ticks) {
    for (uint8_t channel = 0u; channel < 2u; channel++) {
        const uint32_t channel_base = 0x10u + (uint32_t)channel * 0x28u;
        const uint32_t control = timing->pdb_registers[instance][channel_base >> 2u];
        for (uint8_t pretrigger = 0u; pretrigger < 2u; pretrigger++) {
            const uint16_t delay = (uint16_t)timing->pdb_registers[instance]
                                                                [(channel_base + 8u +
                                                                  (uint32_t)pretrigger * 4u) >>
                                                                 2u];
            if ((control & (1u << pretrigger)) != 0u &&
                (control & (1u << (8u + pretrigger))) != 0u &&
                (control & (1u << (16u + pretrigger))) == 0u &&
                pdb_target_reached(start, ticks, delay, timing->pdb_mod[instance])) {
                pdb_queue_delayed_trigger(timing, instance, channel, pretrigger);
            }
            if (delay == start)
                timing->pdb_registers[instance][(channel_base + 4u) >> 2u] |=
                    1u << (16u + pretrigger);
        }
    }
    const uint32_t pulse_enable = timing->pdb_registers[instance][0x190u >> 2u];
    for (uint8_t output = 0u; output < 2u; output++) {
        if ((pulse_enable & (1u << output)) == 0u)
            continue;
        const uint32_t delays = timing->pdb_registers[instance][(0x194u + output * 4u) >> 2u];
        if (pdb_target_reached(start, ticks, (uint16_t)(delays >> 16u),
                               timing->pdb_mod[instance]))
            kinetis_timing_internal_set_pdb_pulse(timing, instance, output, true);
        if (pdb_target_reached(start, ticks, (uint16_t)delays, timing->pdb_mod[instance]))
            kinetis_timing_internal_set_pdb_pulse(timing, instance, output, false);
    }
    const uint8_t dac_count = timing->profile->id == KINETIS_PROFILE_MKV10Z1287 ? 1u : 2u;
    for (uint8_t dac_instance = 0u; dac_instance < dac_count; dac_instance++) {
        const KinetisPeripheralId dac =
            dac_instance == 0u ? KINETIS_PERIPHERAL_DAC0 : KINETIS_PERIPHERAL_DAC1;
        const uint32_t control_offset = 0x150u + (uint32_t)dac_instance * 8u;
        const uint16_t interval =
            (uint16_t)timing->pdb_registers[instance][(control_offset + 4u) >> 2u];
        const uint32_t control = timing->pdb_registers[instance][control_offset >> 2u];
        if (kinetis_timing_internal_has(timing, dac) && (control & 3u) == 1u &&
            pdb_target_reached(start, ticks, interval, timing->pdb_mod[instance]))
            kinetis_timing_internal_trigger(timing, KINETIS_TIMING_TRIGGER_PDB_DAC, dac_instance,
                                            0u, instance);
    }
    if (start == timing->pdb_idly[instance]) {
        timing->pdb_sc[instance] |= 1u << 6u;
        if ((timing->pdb_sc[instance] & (1u << 15u)) != 0u)
            kinetis_timing_internal_request_dma(timing, instance == 0u ? 48u : 47u);
        kinetis_timing_internal_refresh_pdb_irq(timing);
    }
}

void kinetis_timing_internal_trigger_pdb(KinetisTiming* timing, uint8_t instance, uint8_t input) {
    const KinetisPeripheralId peripheral =
        instance == 0u ? KINETIS_PERIPHERAL_PDB0 : KINETIS_PERIPHERAL_PDB1;
    const uint32_t clock_mask = instance == 0u ? 1u << 22u : 1u << 17u;
    if (!kinetis_timing_internal_has(timing, peripheral) ||
        (timing->sim_scgc6 & clock_mask) == 0u || (timing->pdb_sc[instance] & 0x80u) == 0u ||
        ((timing->pdb_sc[instance] >> 8u) & 15u) != input)
        return;
    const uint8_t load_mode = (uint8_t)((timing->pdb_sc[instance] >> 18u) & 3u);
    if ((timing->pdb_sc[instance] & 1u) != 0u && load_mode >= 2u)
        kinetis_timing_internal_load_pdb(timing, instance);
    timing->pdb_counter[instance] = 0u;
    timing->pdb_remainder[instance] = 0u;
    timing->pdb_bus_remainder[instance] = 0u;
    timing->pdb_prescaler_cycles[instance] = 0u;
    timing->pdb_running[instance] = true;
    timing->pdb_bypass_cycles[instance] = 2u;
    pdb_counter_start_events(timing, instance);
}

void kinetis_timing_internal_trigger_pdb_input(KinetisTiming* timing, uint8_t input) {
    for (uint8_t instance = 0u; instance < 2u; instance++)
        kinetis_timing_internal_trigger_pdb(timing, instance, input);
}

bool kinetis_timing_trigger_pdb_input(KinetisTiming* timing, uint8_t input) {
    if (timing == NULL || timing->profile == NULL || input >= 15u)
        return false;
    kinetis_timing_internal_trigger_pdb_input(timing, input);
    return true;
}

bool kinetis_timing_trigger_pdb_dac_input(KinetisTiming* timing, uint8_t instance, uint8_t dac) {
    if (timing == NULL || timing->profile == NULL || instance >= 2u || dac >= 2u)
        return false;
    const KinetisPeripheralId pdb =
        instance == 0u ? KINETIS_PERIPHERAL_PDB0 : KINETIS_PERIPHERAL_PDB1;
    const KinetisPeripheralId converter =
        dac == 0u ? KINETIS_PERIPHERAL_DAC0 : KINETIS_PERIPHERAL_DAC1;
    const uint32_t control = timing->pdb_registers[instance][(0x150u + dac * 8u) >> 2u];
    if (!kinetis_timing_internal_has(timing, pdb) ||
        !kinetis_timing_internal_has(timing, converter) || (control & 3u) != 3u)
        return false;
    kinetis_timing_internal_trigger(timing, KINETIS_TIMING_TRIGGER_PDB_DAC, dac, 0u, instance);
    return true;
}

void kinetis_timing_adc_complete(KinetisTiming* timing, uint8_t instance, uint8_t pretrigger) {
    if (timing == NULL || instance >= 2u || pretrigger >= 2u)
        return;
    const uint8_t acknowledgement = instance == pretrigger ? 0u : 1u;
    for (uint8_t pdb_instance = 0u; pdb_instance < 2u; pdb_instance++) {
        timing->pdb_adc_locks[pdb_instance][instance] &= (uint8_t)~(1u << pretrigger);
        const uint32_t channel_base = 0x10u + (uint32_t)acknowledgement * 0x28u;
        const uint32_t control = timing->pdb_registers[pdb_instance][channel_base >> 2u];
        const uint8_t pending =
            (uint8_t)(control & (control >> 16u) & (1u << (pretrigger ^ 1u)));
        if (pending != 0u) {
            timing->pdb_back_to_back_pending[pdb_instance][acknowledgement] |= pending;
            timing->pdb_back_to_back_cycles[pdb_instance][acknowledgement] = 2u;
        }
    }
}

static void pdb_advance_pending_triggers(KinetisTiming* timing, uint8_t instance) {
    for (uint8_t channel = 0u; channel < 2u; channel++) {
        if (timing->pdb_channel_trigger_cycles[instance][channel] == 0u)
            continue;
        timing->pdb_channel_trigger_cycles[instance][channel]--;
        if (timing->pdb_channel_trigger_cycles[instance][channel] == 0u) {
            const uint8_t pending = timing->pdb_channel_trigger_pending[instance][channel];
            timing->pdb_channel_trigger_pending[instance][channel] = 0u;
            for (uint8_t pretrigger = 0u; pretrigger < 2u; pretrigger++) {
                if ((pending & (1u << pretrigger)) != 0u)
                    kinetis_timing_internal_trigger(timing, KINETIS_TIMING_TRIGGER_PDB_ADC,
                                                    channel, pretrigger, instance);
            }
            if (channel == 1u && pending != 0u)
                kinetis_timing_internal_trigger(timing, KINETIS_TIMING_TRIGGER_PDB_FTM, instance,
                                                1u, instance);
        }
    }
    if (timing->pdb_bypass_cycles[instance] != 0u) {
        timing->pdb_bypass_cycles[instance]--;
        if (timing->pdb_bypass_cycles[instance] == 0u) {
            pdb_bypass_triggers(timing, instance);
        }
    }
    for (uint8_t channel = 0u; channel < 2u; channel++) {
        if (timing->pdb_back_to_back_cycles[instance][channel] != 0u) {
            timing->pdb_back_to_back_cycles[instance][channel]--;
            if (timing->pdb_back_to_back_cycles[instance][channel] == 0u) {
                const uint8_t pending = timing->pdb_back_to_back_pending[instance][channel];
                timing->pdb_back_to_back_pending[instance][channel] = 0u;
                for (uint8_t pretrigger = 0u; pretrigger < 2u; pretrigger++) {
                    if ((pending & (1u << pretrigger)) != 0u)
                        pdb_assert_pretrigger(timing, instance, channel, pretrigger);
                }
            }
        }
        for (uint8_t pretrigger = 0u; pretrigger < 2u; pretrigger++) {
            if (timing->pdb_delayed_cycles[instance][channel][pretrigger] == 0u)
                continue;
            timing->pdb_delayed_cycles[instance][channel][pretrigger]--;
            if (timing->pdb_delayed_cycles[instance][channel][pretrigger] == 0u) {
                timing->pdb_delayed_pending[instance][channel] &=
                    (uint8_t)~(1u << pretrigger);
                pdb_assert_pretrigger(timing, instance, channel, pretrigger);
            }
        }
    }
}

bool kinetis_timing_pdb_pulse_output(const KinetisTiming* timing, uint8_t instance,
                                     uint8_t output) {
    return timing != NULL && timing->profile != NULL && instance < 2u && output < 2u &&
           timing->pdb_pulse_output[instance][output];
}

void kinetis_timing_internal_advance_pdb(KinetisTiming* timing, uint8_t instance, uint32_t cycles) {
    const KinetisPeripheralId peripheral =
        instance == 0u ? KINETIS_PERIPHERAL_PDB0 : KINETIS_PERIPHERAL_PDB1;
    const uint32_t clock_mask = instance == 0u ? 1u << 22u : 1u << 17u;
    if (!kinetis_timing_internal_has(timing, peripheral) ||
        (timing->sim_scgc6 & clock_mask) == 0u || (timing->pdb_sc[instance] & 0x80u) == 0u ||
        !timing->pdb_running[instance] || !kinetis_timing_bus_clock_running(timing)) {
        return;
    }
    const uint64_t bus_cycles = kinetis_timing_internal_clock_ticks(
        &timing->pdb_bus_remainder[instance], cycles, timing->bus_clock_hz,
        timing->core_clock_hz);
    if (bus_cycles == 0u)
        return;
    for (uint64_t bus_cycle = 0u; bus_cycle < bus_cycles && timing->pdb_running[instance];
         bus_cycle++) {
        pdb_advance_pending_triggers(timing, instance);
        timing->pdb_prescaler_cycles[instance]++;
        if (timing->pdb_prescaler_cycles[instance] < pdb_divider(timing->pdb_sc[instance]))
            continue;
        timing->pdb_prescaler_cycles[instance] = 0u;
        const uint16_t start = timing->pdb_counter[instance];
        const uint64_t period = (uint64_t)timing->pdb_mod[instance] + 1u;
        const uint64_t until_overflow = period - start;
        pdb_counter_events(timing, instance, start, 1u);
        if (until_overflow > 1u) {
            timing->pdb_counter[instance] = (uint16_t)(start + 1u);
            continue;
        }
        timing->pdb_counter[instance] = 0u;
        const uint8_t load_mode = (uint8_t)((timing->pdb_sc[instance] >> 18u) & 3u);
        if ((timing->pdb_sc[instance] & 1u) != 0u && (load_mode == 1u || load_mode == 3u))
            kinetis_timing_internal_load_pdb(timing, instance);
        if ((timing->pdb_sc[instance] & 2u) == 0u) {
            timing->pdb_running[instance] = false;
        } else {
            pdb_counter_start_events(timing, instance);
        }
    }
}

bool kinetis_timing_internal_ftm_location(const KinetisTiming* timing, uint32_t address,
                                          uint8_t* instance, uint32_t* offset) {
    const KinetisPeripheralId ids[KINETIS_FTM_COUNT] = {
        KINETIS_PERIPHERAL_FTM0, KINETIS_PERIPHERAL_FTM1, KINETIS_PERIPHERAL_FTM2,
        KINETIS_PERIPHERAL_FTM3, KINETIS_PERIPHERAL_FTM4, KINETIS_PERIPHERAL_FTM5};
    for (uint8_t instance_index = 0; instance_index < KINETIS_FTM_COUNT; instance_index++) {
        KinetisPeripheralBlock block;
        if (kinetis_profile_peripheral_block(timing->profile, ids[instance_index], &block) &&
            address >= block.address && address < block.address + block.size) {
            *instance = instance_index;
            *offset = address - block.address;
            return true;
        }
    }
    return false;
}
