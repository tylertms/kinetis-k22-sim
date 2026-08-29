#include "internal.h"

uint8_t kinetis_timing_internal_profile_irq(const KinetisTiming* timing, uint8_t default_irq,
                                            uint8_t mkv10_irq) {
    return timing->profile->id == KINETIS_PROFILE_MKV10Z1287 ? mkv10_irq : default_irq;
}

void kinetis_timing_internal_set_irq(const KinetisTiming* timing, uint8_t interrupt_number,
                                     bool interrupt_asserted) {
    if (timing->signals.irq != NULL) {
        timing->signals.irq(timing->signals.context, interrupt_number, interrupt_asserted);
    }
}

void kinetis_timing_internal_update_pmc_irq(const KinetisTiming* timing) {
    const bool low_voltage_detect_active = (timing->pmc[0] & 0xa0u) == 0xa0u;
    const bool low_voltage_warning_active = (timing->pmc[1] & 0xa0u) == 0xa0u;
    kinetis_timing_internal_set_irq(timing, kinetis_timing_internal_profile_irq(timing, IRQ_LVD, 6u),
                                    low_voltage_detect_active || low_voltage_warning_active);
}

void kinetis_timing_internal_update_llwu_irq(const KinetisTiming* timing) {
    const bool mkv10 = timing->profile->id == KINETIS_PROFILE_MKV10Z1287;
    const bool has_pending_pin_wakeup =
        mkv10 ? (timing->llwu[9] | timing->llwu[10] | timing->llwu[11] | timing->llwu[12]) != 0u
              : (timing->llwu[5] | timing->llwu[6]) != 0u;
    const bool has_pending_module_wakeup = timing->llwu[mkv10 ? 13u : 7u] != 0u;
    const uint8_t filter_index = mkv10 ? 14u : 8u;
    const bool has_pending_filter_wakeup =
        ((timing->llwu[filter_index] | timing->llwu[filter_index + 1u]) & 0x80u) != 0u;
    kinetis_timing_internal_set_irq(timing,
                                    kinetis_timing_internal_profile_irq(timing, IRQ_LLWU, 7u),
                                    has_pending_pin_wakeup || has_pending_module_wakeup ||
                                        has_pending_filter_wakeup);
}

static uint8_t reset_pin_filter_mode(const KinetisTiming* timing) {
    if (timing->deep_sleeping) {
        const bool vlls0 = timing->smc_stop_status == 0x40u && (timing->smc[2] & 7u) == 0u;
        return !vlls0 && (timing->rcm[4] & 4u) != 0u ? 2u : 0u;
    }
    const uint8_t mode = timing->rcm[4] & 3u;
    return mode < 3u ? mode : 0u;
}

void kinetis_timing_internal_reset_pin_filter(KinetisTiming* timing) {
    timing->reset_pin_filter_remainder = 0u;
    timing->reset_pin_filter_ticks = 0u;
}

static bool accept_reset_pin(KinetisTiming* timing) {
    timing->reset_pin_filtered_high = timing->reset_pin_input_high;
    kinetis_timing_internal_reset_pin_filter(timing);
    if (!timing->reset_pin_filtered_high) {
        timing->reset_pin_held = true;
        kinetis_timing_internal_signal_reset(timing, 0x40u, 0u);
        return true;
    }
    timing->reset_pin_held = false;
    return false;
}

bool kinetis_timing_internal_advance_reset_pin(KinetisTiming* timing, uint32_t core_cycles) {
    if (timing->reset_pin_input_high == timing->reset_pin_filtered_high)
        return false;
    const uint8_t mode = reset_pin_filter_mode(timing);
    if (mode == 0u)
        return accept_reset_pin(timing);
    const uint32_t source_hz = mode == 1u && kinetis_timing_bus_clock_running(timing)
                                   ? timing->bus_clock_hz
                                   : mode == 2u ? timing->lpo_hz : 0u;
    const uint64_t ticks = kinetis_timing_internal_clock_ticks(
        &timing->reset_pin_filter_remainder, core_cycles, source_hz, timing->core_clock_hz);
    const uint8_t required_ticks = mode == 1u ? (timing->rcm[5] & 0x1fu) + 1u : 5u;
    if (ticks >= required_ticks - timing->reset_pin_filter_ticks)
        return accept_reset_pin(timing);
    timing->reset_pin_filter_ticks += (uint8_t)ticks;
    return false;
}

bool kinetis_timing_set_reset_pin(KinetisTiming* timing, bool high) {
    if (timing == NULL || timing->profile == NULL ||
        !kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_RCM))
        return false;
    if (timing->reset_pin_input_high == high)
        return true;
    timing->reset_pin_input_high = high;
    kinetis_timing_internal_reset_pin_filter(timing);
    if (reset_pin_filter_mode(timing) == 0u)
        (void)accept_reset_pin(timing);
    return true;
}

bool kinetis_timing_set_reset_state(KinetisTiming* timing, uint8_t srs0, bool ackiso) {
    if (timing == NULL)
        return false;
    timing->rcm[0] = srs0 & 0xefu;
    timing->pmc[2] = (timing->pmc[2] & 0xf7u) | (ackiso ? 8u : 0u);
    return true;
}

void kinetis_timing_internal_request_dma(const KinetisTiming* timing, uint8_t request_source) {
    if (timing->signals.dma != NULL) {
        timing->signals.dma(timing->signals.context, request_source);
    }
}

void kinetis_timing_internal_trigger_dma(const KinetisTiming* timing, uint8_t dma_channel) {
    if (timing->signals.dma_trigger != NULL)
        timing->signals.dma_trigger(timing->signals.context, dma_channel);
}

void kinetis_timing_internal_trigger(KinetisTiming* timing, KinetisTimingTrigger trigger_type,
                                     uint8_t peripheral_instance, uint8_t peripheral_channel,
                                     uint8_t source_instance) {
    if (timing->signals.trigger != NULL)
        timing->signals.trigger(timing->signals.context, trigger_type, peripheral_instance,
                                peripheral_channel, source_instance);
}

void kinetis_timing_internal_trigger_adc_alternate(KinetisTiming* timing, uint8_t adc_source) {
    kinetis_timing_internal_trigger(timing, KINETIS_TIMING_TRIGGER_ADC_ALTERNATE, adc_source, 0u,
                                    0u);
}

bool kinetis_timing_internal_has(const KinetisTiming* timing, KinetisPeripheralId peripheral) {
    if (timing->profile == NULL)
        return false;
    return kinetis_profile_has_peripheral(timing->profile, peripheral);
}

bool kinetis_timing_internal_contains(const KinetisTiming* timing, KinetisPeripheralId peripheral,
                                      uint32_t access_address, uint8_t access_size) {
    KinetisPeripheralLocation resolved_location;
    if (!kinetis_profile_resolve_peripheral(timing->profile, access_address, access_size,
                                            &resolved_location))
        return false;
    return resolved_location.id == peripheral;
}
