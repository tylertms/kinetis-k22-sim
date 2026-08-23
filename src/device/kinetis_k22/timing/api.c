#include "internal.h"

void k22_timing_internal_set_irq(const K22Timing* timing, uint8_t irq, bool asserted) {
    if (timing->signals.irq != NULL) {
        timing->signals.irq(timing->signals.context, irq, asserted);
    }
}

void k22_timing_internal_update_pmc_irq(const K22Timing* timing) {
    const bool low_voltage_detect_active = (timing->pmc[0] & 0xa0u) == 0xa0u;
    const bool low_voltage_warning_active = (timing->pmc[1] & 0xa0u) == 0xa0u;
    k22_timing_internal_set_irq(timing, IRQ_LVD,
                                low_voltage_detect_active || low_voltage_warning_active);
}

void k22_timing_internal_update_llwu_irq(const K22Timing* timing) {
    const bool has_pending_pin_wakeup = (timing->llwu[5] | timing->llwu[6]) != 0u;
    const bool has_pending_module_wakeup = timing->llwu[7] != 0u;
    const bool has_pending_filter_wakeup = ((timing->llwu[8] | timing->llwu[9]) & 0x80u) != 0u;
    k22_timing_internal_set_irq(timing, IRQ_LLWU,
                                has_pending_pin_wakeup || has_pending_module_wakeup ||
                                    has_pending_filter_wakeup);
}

void k22_timing_internal_request_dma(const K22Timing* timing, uint8_t source) {
    if (timing->signals.dma != NULL) {
        timing->signals.dma(timing->signals.context, source);
    }
}

void k22_timing_internal_trigger_dma(const K22Timing* timing, uint8_t channel) {
    if (timing->signals.dma_trigger != NULL)
        timing->signals.dma_trigger(timing->signals.context, channel);
}

void k22_timing_internal_trigger(K22Timing* timing, K22TimingTrigger trigger_type, uint8_t instance,
                                 uint8_t channel) {
    if (timing->signals.trigger != NULL)
        timing->signals.trigger(timing->signals.context, trigger_type, instance, channel);
}

void k22_timing_internal_trigger_adc_alternate(K22Timing* timing, uint8_t source) {
    k22_timing_internal_trigger(timing, K22_TIMING_TRIGGER_ADC_ALTERNATE, source, 0u);
}

bool k22_timing_internal_has(const K22Timing* timing, K22PeripheralId peripheral) {
    if (timing->profile == NULL)
        return false;
    return k22_profile_has_peripheral(timing->profile, peripheral);
}

bool k22_timing_internal_contains(const K22Timing* timing, K22PeripheralId peripheral,
                                  uint32_t address, uint8_t size) {
    K22PeripheralLocation location;
    if (!k22_profile_resolve_peripheral(timing->profile, address, size, &location))
        return false;
    return location.id == peripheral;
}
