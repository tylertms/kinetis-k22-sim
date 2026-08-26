#include "internal.h"

void kinetis_timing_internal_set_irq(const KinetisTiming* timing, uint8_t interrupt_number,
                                     bool interrupt_asserted) {
    if (timing->signals.irq != NULL) {
        timing->signals.irq(timing->signals.context, interrupt_number, interrupt_asserted);
    }
}

void kinetis_timing_internal_update_pmc_irq(const KinetisTiming* timing) {
    const bool low_voltage_detect_active = (timing->pmc[0] & 0xa0u) == 0xa0u;
    const bool low_voltage_warning_active = (timing->pmc[1] & 0xa0u) == 0xa0u;
    kinetis_timing_internal_set_irq(timing, IRQ_LVD,
                                    low_voltage_detect_active || low_voltage_warning_active);
}

void kinetis_timing_internal_update_llwu_irq(const KinetisTiming* timing) {
    const bool has_pending_pin_wakeup = (timing->llwu[5] | timing->llwu[6]) != 0u;
    const bool has_pending_module_wakeup = timing->llwu[7] != 0u;
    const bool has_pending_filter_wakeup = ((timing->llwu[8] | timing->llwu[9]) & 0x80u) != 0u;
    kinetis_timing_internal_set_irq(timing, IRQ_LLWU,
                                    has_pending_pin_wakeup || has_pending_module_wakeup ||
                                        has_pending_filter_wakeup);
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
                                     uint8_t peripheral_instance, uint8_t peripheral_channel) {
    if (timing->signals.trigger != NULL)
        timing->signals.trigger(timing->signals.context, trigger_type, peripheral_instance,
                                peripheral_channel);
}

void kinetis_timing_internal_trigger_adc_alternate(KinetisTiming* timing, uint8_t adc_source) {
    kinetis_timing_internal_trigger(timing, KINETIS_TIMING_TRIGGER_ADC_ALTERNATE, adc_source, 0u);
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
