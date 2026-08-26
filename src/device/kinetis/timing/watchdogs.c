#include "internal.h"

static uint32_t watchdog_timeout_value(const KinetisTiming* timing) {
    return ((uint32_t)timing->wdog[2] << 16u) | timing->wdog[3];
}

void kinetis_timing_internal_signal_reset(KinetisTiming* timing, uint8_t srs0, uint8_t srs1) {
    const uint64_t generation = timing->reset_generation;
    if (timing->signals.reset != NULL) {
        timing->signals.reset(timing->signals.context, srs0, srs1);
    }
    if (timing->reset_generation == generation)
        kinetis_timing_warm_reset(timing, srs0, srs1);
}

static void update_watchdog_interrupt(const KinetisTiming* timing) {
    const bool watchdog_interrupt_enabled = (timing->wdog[1] & 0x8000u) != 0u;
    const bool external_monitor_asserted = timing->ewm_output && (timing->ewm_ctrl & 8u) != 0u;
    kinetis_timing_internal_set_irq(timing, IRQ_WDOG_EWM,
                                    watchdog_interrupt_enabled || external_monitor_asserted);
}

static bool handle_watchdog_expiration(KinetisTiming* timing) {
    timing->wdog_counter = 0u;
    if ((timing->wdog[0] & 4u) == 0u) {
        kinetis_timing_internal_signal_reset(timing, 0x20u, 0u);
        return true;
    }
    if (timing->wdog_reset_pending) {
        kinetis_timing_internal_signal_reset(timing, 0x20u, 0u);
        return true;
    }
    timing->wdog[1] |= 0x8000u;
    timing->wdog_reset_pending = true;
    timing->wdog_reset_deadline = timing->wdog_bus_cycles + 256u;
    update_watchdog_interrupt(timing);
    return false;
}

static void apply_watchdog_update(KinetisTiming* timing) {
    const uint16_t immediate_control_mask = 0x00e4u;
    const uint16_t test_mode_bits = timing->wdog[0] & 0x4000u;
    for (uint8_t byte_index = 0u; byte_index < 24u; byte_index++) {
        if ((timing->wdog_update_mask & (UINT32_C(1) << byte_index)) == 0u)
            continue;
        const uint8_t register_index = byte_index / 2u;
        const uint16_t byte_mask = byte_index % 2u == 0u ? 0x00ffu : 0xff00u;
        timing->wdog[register_index] = (timing->wdog[register_index] & (uint16_t)~byte_mask) |
                                       (timing->wdog_pending[register_index] & byte_mask);
    }
    timing->wdog[0] = (timing->wdog[0] & immediate_control_mask) |
                      (timing->wdog_pending[0] & (uint16_t)~immediate_control_mask);
    timing->wdog[0] = (timing->wdog[0] & 0x7cffu) | test_mode_bits;
    timing->wdog[1] = (timing->wdog[1] & 0x8000u) | 1u;
    timing->wdog[11] &= 0x0700u;
    timing->wdog_update_open = false;
    timing->wdog_update_mask = 0u;
    timing->wdog_counter = 0u;
    timing->wdog_remainder = 0u;
}

static bool process_watchdog_bus_time(KinetisTiming* timing) {
    if (timing->wdog_reset_pending && timing->wdog_bus_cycles >= timing->wdog_reset_deadline) {
        kinetis_timing_internal_signal_reset(timing, 0x20u, 0u);
        return true;
    }
    if (timing->wdog_unlock_stage == 1u &&
        timing->wdog_bus_cycles > timing->wdog_sequence_deadline) {
        timing->wdog_unlock_stage = 0u;
        return handle_watchdog_expiration(timing);
    }
    if (timing->wdog_refresh_stage == 1u &&
        timing->wdog_bus_cycles > timing->wdog_sequence_deadline) {
        timing->wdog_refresh_stage = 0u;
        return handle_watchdog_expiration(timing);
    }
    if (timing->wdog_initial_unlock_required && !timing->wdog_initial_debug_pause &&
        timing->wdog_unlock_stage == 0u && timing->wdog_bus_cycles > timing->wdog_update_deadline) {
        timing->wdog_initial_unlock_required = false;
        return handle_watchdog_expiration(timing);
    }
    if (timing->wdog_update_open && timing->wdog_bus_cycles >= timing->wdog_update_deadline) {
        if (!timing->wdog_update_written) {
            timing->wdog_update_open = false;
            return handle_watchdog_expiration(timing);
        }
        apply_watchdog_update(timing);
    }
    return false;
}

static bool advance_watchdog_bus_time(KinetisTiming* timing, uint64_t elapsed_bus_ticks) {
    timing->wdog_bus_cycles += elapsed_bus_ticks;
    return process_watchdog_bus_time(timing);
}

static bool is_watchdog_running(const KinetisTiming* timing) {
    if ((timing->wdog[0] & 1u) == 0u)
        return false;
    if (timing->debug_halted && (timing->wdog[0] & 0x20u) == 0u)
        return false;
    if (!timing->cpu_sleeping)
        return true;
    if (timing->deep_sleeping)
        return (timing->wdog[0] & 0x40u) != 0u;
    return (timing->wdog[0] & 0x80u) != 0u;
}

static uint32_t effective_watchdog_timeout(const KinetisTiming* timing) {
    const uint32_t configured_timeout = watchdog_timeout_value(timing);
    if ((timing->wdog[0] & 0x4c00u) != 0x0c00u)
        return configured_timeout;
    const uint8_t timeout_byte_index = (uint8_t)((timing->wdog[0] >> 12u) & 3u);
    return (configured_timeout >> (timeout_byte_index * 8u)) & 0xffu;
}

void kinetis_timing_watchdog_advance(KinetisTiming* timing, uint32_t elapsed_watchdog_ticks) {
    if (timing == NULL || timing->profile == NULL || elapsed_watchdog_ticks == 0u ||
        !kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_WDOG) ||
        !is_watchdog_running(timing))
        return;

    const uint32_t timeout_ticks = effective_watchdog_timeout(timing);
    if (timeout_ticks == 0u || timing->wdog_counter >= timeout_ticks ||
        elapsed_watchdog_ticks >= timeout_ticks - timing->wdog_counter) {
        (void)handle_watchdog_expiration(timing);
    } else {
        timing->wdog_counter += elapsed_watchdog_ticks;
    }
}

void kinetis_timing_internal_advance_wdog(KinetisTiming* timing, uint32_t cycles) {
    if (!kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_WDOG))
        return;
    const bool update_was_open = timing->wdog_update_open;
    const uint32_t bus_clock_hz =
        kinetis_timing_bus_clock_running(timing) ? timing->bus_clock_hz : 0u;
    const uint64_t elapsed_bus_ticks = kinetis_timing_internal_clock_ticks(
        &timing->wdog_bus_remainder, cycles, bus_clock_hz, timing->core_clock_hz);
    if (advance_watchdog_bus_time(timing, elapsed_bus_ticks) ||
        (update_was_open && !timing->wdog_update_open) || !is_watchdog_running(timing))
        return;
    const bool is_test_mode = (timing->wdog[0] & 0x4400u) == 0x0400u;
    const uint32_t source_hz =
        is_test_mode || (timing->wdog[0] & (1u << 13u)) != 0u ? bus_clock_hz : timing->lpo_hz;
    const uint32_t divider = ((timing->wdog[11] & 0x700u) >> 8u) + 1u;
    const uint64_t elapsed_watchdog_ticks = kinetis_timing_internal_clock_ticks(
        &timing->wdog_remainder, cycles, source_hz / divider, timing->core_clock_hz);
    kinetis_timing_watchdog_advance(timing, elapsed_watchdog_ticks > UINT32_MAX
                                                ? UINT32_MAX
                                                : (uint32_t)elapsed_watchdog_ticks);
}

static void trigger_ewm_output(KinetisTiming* timing) {
    timing->ewm_output = true;
    update_watchdog_interrupt(timing);
}

void kinetis_timing_internal_advance_ewm(KinetisTiming* timing, uint32_t cycles) {
    if (!kinetis_timing_internal_has(timing, KINETIS_PERIPHERAL_EWM) ||
        (timing->ewm_ctrl & 1u) == 0u || timing->ewm_output || timing->cpu_sleeping)
        return;
    const uint32_t source_hz = timing->lpo_hz / ((uint32_t)timing->ewm_prescaler + 1u);
    const uint64_t ticks = kinetis_timing_internal_clock_ticks(&timing->ewm_remainder, cycles,
                                                               source_hz, timing->core_clock_hz);
    const uint32_t increment = ticks > UINT32_MAX ? UINT32_MAX : (uint32_t)ticks;
    timing->ewm_counter = increment >= UINT32_MAX - timing->ewm_counter
                              ? UINT32_MAX
                              : timing->ewm_counter + increment;
    if (timing->ewm_cmph != 0xffu && timing->ewm_counter >= timing->ewm_cmph)
        trigger_ewm_output(timing);
}

bool kinetis_timing_internal_read_wdog(const KinetisTiming* timing, uint32_t address, uint8_t size,
                                       uint32_t* output_value) {
    if ((size != 1u && size != 2u) || address < WDOG_BASE || address + size > WDOG_BASE + 0x18u ||
        (size == 2u && (address & 1u) != 0u)) {
        return false;
    }
    const uint8_t register_index = (uint8_t)((address - WDOG_BASE) / 2u);
    uint16_t register_value;
    if (register_index == 8u) {
        register_value = (uint16_t)(timing->wdog_counter >> 16u);
    } else if (register_index == 9u) {
        register_value = (uint16_t)timing->wdog_counter;
    } else {
        register_value = timing->wdog[register_index];
    }
    *output_value = size == 1u ? (register_value >> (((address - WDOG_BASE) & 1u) * 8u)) & 0xffu
                               : register_value;
    return true;
}

bool kinetis_timing_projected_watchdog_read(const KinetisTiming* timing, uint32_t address,
                                            uint8_t size, uint32_t* output_value) {
    if (timing == NULL || output_value == NULL) {
        return false;
    }
    if (!timing->wdog_update_open || !timing->wdog_update_written) {
        return kinetis_timing_internal_read_wdog(timing, address, size, output_value);
    }
    KinetisTiming projected = *timing;
    apply_watchdog_update(&projected);
    return kinetis_timing_internal_read_wdog(&projected, address, size, output_value);
}

static bool is_valid_sequence_byte(uint8_t lane, uint8_t byte_value, uint16_t first,
                                   uint16_t second) {
    const uint8_t shift = lane * 8u;
    return byte_value == ((first >> shift) & 0xffu) || byte_value == ((second >> shift) & 0xffu);
}

static uint16_t merge_watchdog_write(uint16_t previous_value, uint32_t address, uint8_t size,
                                     uint32_t write_value) {
    if (size == 2u)
        return (uint16_t)write_value;
    const uint8_t shift = (uint8_t)((address & 1u) * 8u);
    const uint16_t byte_mask = (uint16_t)(0xffu << shift);
    return (previous_value & (uint16_t)~byte_mask) | (uint16_t)(((uint8_t)write_value) << shift);
}

static void accept_wdog_unlock(KinetisTiming* timing, uint16_t sequence_value) {
    if (sequence_value == 0xc520u) {
        timing->wdog_unlock_stage = 1u;
        timing->wdog_sequence_deadline = timing->wdog_bus_cycles + 20u;
        return;
    }
    if (sequence_value == 0xd928u && timing->wdog_unlock_stage == 1u &&
        timing->wdog_bus_cycles <= timing->wdog_sequence_deadline) {
        timing->wdog_unlock_stage = 0u;
        timing->wdog_initial_unlock_required = false;
        timing->wdog_update_open = true;
        timing->wdog_update_written = false;
        timing->wdog_update_mask = 0u;
        timing->wdog_update_ready = timing->wdog_bus_cycles + 1u;
        timing->wdog_update_deadline = timing->wdog_update_ready + 256u;
        memcpy(timing->wdog_pending, timing->wdog, sizeof(timing->wdog));
        return;
    }
    timing->wdog_unlock_stage = 0u;
    (void)handle_watchdog_expiration(timing);
}

static void accept_wdog_refresh(KinetisTiming* timing, uint16_t sequence_value) {
    if (sequence_value == 0xa602u) {
        timing->wdog_refresh_stage = 1u;
        timing->wdog_sequence_deadline = timing->wdog_bus_cycles + 20u;
        return;
    }
    if (sequence_value == 0xb480u && timing->wdog_refresh_stage == 1u &&
        timing->wdog_bus_cycles <= timing->wdog_sequence_deadline) {
        const uint32_t refresh_window = ((uint32_t)timing->wdog[4] << 16u) | timing->wdog[5];
        if ((timing->wdog[0] & (1u << 3u)) != 0u && timing->wdog_counter < refresh_window)
            (void)handle_watchdog_expiration(timing);
        else
            timing->wdog_counter = 0u;
        timing->wdog_refresh_stage = 0u;
        return;
    }
    timing->wdog_refresh_stage = 0u;
    (void)handle_watchdog_expiration(timing);
}

bool kinetis_timing_internal_write_wdog(KinetisTiming* timing, uint32_t address, uint8_t size,
                                        uint32_t write_value) {
    if ((size != 1u && size != 2u) || address < WDOG_BASE || address + size > WDOG_BASE + 0x18u ||
        (size == 2u && (address & 1u) != 0u)) {
        return false;
    }
    if (advance_watchdog_bus_time(timing, 1u))
        return true;
    const uint8_t register_index = (uint8_t)((address - WDOG_BASE) / 2u);
    const uint8_t lane = (uint8_t)((address - WDOG_BASE) & 1u);
    if (register_index == 7u) {
        if (timing->wdog_refresh_stage != 0u || timing->wdog_update_open)
            return true;
        if (!timing->wdog_initial_unlock_required && (timing->wdog[0] & 0x10u) == 0u)
            return true;
        timing->wdog[register_index] =
            merge_watchdog_write(timing->wdog[register_index], address, size, write_value);
        if (size == 1u && !is_valid_sequence_byte(lane, (uint8_t)write_value, 0xc520u, 0xd928u)) {
            (void)handle_watchdog_expiration(timing);
        } else if (size == 2u || timing->wdog[register_index] == 0xc520u ||
                   timing->wdog[register_index] == 0xd928u) {
            accept_wdog_unlock(timing, timing->wdog[register_index]);
        }
        return true;
    }
    if (register_index == 6u) {
        if (timing->wdog_unlock_stage != 0u || timing->wdog_update_open)
            return true;
        timing->wdog[register_index] =
            merge_watchdog_write(timing->wdog[register_index], address, size, write_value);
        if (size == 1u && !is_valid_sequence_byte(lane, (uint8_t)write_value, 0xa602u, 0xb480u)) {
            (void)handle_watchdog_expiration(timing);
        } else if (size == 2u || timing->wdog[register_index] == 0xa602u ||
                   timing->wdog[register_index] == 0xb480u) {
            accept_wdog_refresh(timing, timing->wdog[register_index]);
        }
        return true;
    }
    if (register_index == 1u) {
        const uint16_t written_value =
            size == 1u ? (uint16_t)((uint8_t)write_value << (lane * 8u)) : (uint16_t)write_value;
        if ((written_value & 0x8000u) != 0u) {
            timing->wdog[1] &= 0x7fffu;
            update_watchdog_interrupt(timing);
        }
        if (timing->wdog_update_open && timing->wdog_bus_cycles >= timing->wdog_update_ready)
            timing->wdog_update_written = true;
        return true;
    }
    if (register_index == 10u) {
        const uint16_t written_value =
            size == 1u ? (uint16_t)((uint8_t)write_value << (lane * 8u)) : (uint16_t)write_value;
        timing->wdog[10] &= (uint16_t)~written_value;
        return true;
    }
    if (register_index == 8u || register_index == 9u)
        return true;
    const uint8_t update_byte = (uint8_t)(register_index * 2u + lane);
    const uint32_t update_bits =
        size == 2u ? UINT32_C(3) << (register_index * 2u) : UINT32_C(1) << update_byte;
    if (!timing->wdog_update_open || timing->wdog_bus_cycles < timing->wdog_update_ready ||
        register_index >= 12u || (timing->wdog_update_mask & update_bits) != 0u) {
        return true;
    }
    timing->wdog_pending[register_index] =
        merge_watchdog_write(timing->wdog_pending[register_index], address, size, write_value);
    timing->wdog_update_mask |= update_bits;
    timing->wdog_update_written = true;
    if (register_index == 0u) {
        const uint16_t immediate_control_mask = 0x00e4u;
        timing->wdog[0] = (timing->wdog[0] & (uint16_t)~immediate_control_mask) |
                          (timing->wdog_pending[0] & immediate_control_mask);
    }
    return true;
}

bool kinetis_timing_internal_read_ewm(const KinetisTiming* timing, uint32_t address, uint8_t size,
                                      uint32_t* output_value) {
    if (size != 1 || address < EWM_BASE || address > EWM_BASE + 5u || address == EWM_BASE + 4u) {
        return false;
    }
    switch (address - EWM_BASE) {
    case 0:
        *output_value = timing->ewm_ctrl;
        return true;
    case 1:
        *output_value = 0;
        return true;
    case 2:
        *output_value = timing->ewm_cmpl;
        return true;
    case 3:
        *output_value = timing->ewm_cmph;
        return true;
    case 5:
        *output_value = timing->ewm_prescaler;
        return true;
    default:
        return false;
    }
}

bool kinetis_timing_internal_write_ewm(KinetisTiming* timing, uint32_t address, uint8_t size,
                                       uint32_t write_value) {
    if (size != 1) {
        return false;
    }
    switch (address - EWM_BASE) {
    case 0:
        if (!timing->ewm_control_written) {
            timing->ewm_ctrl = (uint8_t)write_value & 15u;
            timing->ewm_control_written = true;
            if ((timing->ewm_ctrl & 1u) != 0u) {
                timing->ewm_counter = 0u;
                timing->ewm_output = false;
            }
        } else if (((uint8_t)write_value & 7u) != (timing->ewm_ctrl & 7u)) {
            return false;
        } else {
            timing->ewm_ctrl = (timing->ewm_ctrl & 7u) | ((uint8_t)write_value & 8u);
        }
        update_watchdog_interrupt(timing);
        return true;
    case 1:
        if (write_value == 0xb4u) {
            timing->ewm_service_stage = 1u;
            timing->ewm_service_deadline = timing->wdog_bus_cycles + 15u;
            timing->ewm_service_paused = false;
        } else if (write_value == 0x2cu && timing->ewm_service_stage == 1u &&
                   !timing->ewm_service_paused &&
                   timing->wdog_bus_cycles <= timing->ewm_service_deadline) {
            const bool input_asserted = (timing->ewm_ctrl & 4u) != 0u &&
                                        timing->ewm_input == ((timing->ewm_ctrl & 2u) != 0u);
            if (timing->ewm_counter > timing->ewm_cmpl && timing->ewm_counter < timing->ewm_cmph &&
                !input_asserted)
                timing->ewm_counter = 0u;
            else
                trigger_ewm_output(timing);
            timing->ewm_service_stage = 0u;
            timing->ewm_service_paused = false;
        } else {
            timing->ewm_service_stage = 0u;
            timing->ewm_service_paused = false;
        }
        return true;
    case 2:
        if (timing->ewm_cmpl_written)
            return false;
        timing->ewm_cmpl = (uint8_t)write_value;
        timing->ewm_cmpl_written = true;
        return true;
    case 3:
        if (timing->ewm_cmph_written)
            return false;
        timing->ewm_cmph = (uint8_t)write_value;
        timing->ewm_cmph_written = true;
        return true;
    case 5:
        if (timing->ewm_prescaler_written)
            return false;
        timing->ewm_prescaler = (uint8_t)write_value;
        timing->ewm_prescaler_written = true;
        return true;
    default:
        return false;
    }
}

bool kinetis_timing_internal_ftm_read(KinetisTiming* timing, uint8_t instance, uint32_t offset,
                                      uint8_t size, uint32_t* output_value) {
    if (size != 4 || (offset & 3u) != 0) {
        return false;
    }
    KinetisFtmState* ftm = &timing->ftm[instance];
    if (offset == 0) {
        *output_value = ftm->sc;
        if ((*output_value & 0x80u) != 0u)
            ftm->overflow_flag_read = true;
    } else if (offset == 4)
        *output_value = ftm->counter;
    else if (offset == 8)
        *output_value = ftm->modulo;
    else if (offset >= 0x0cu && offset < 0x4cu) {
        const uint8_t channel = (uint8_t)((offset - 0x0cu) / 8u);
        if (channel >= kinetis_timing_internal_ftm_channel_count(timing, instance))
            return false;
        if (((offset - 0x0cu) & 4u) == 0u) {
            *output_value = ftm->channel_sc[channel];
            if ((*output_value & 0x80u) != 0u)
                ftm->channel_flag_read[channel] = true;
        } else {
            *output_value = ftm->channel_value[channel];
        }
    } else if (offset == 0x4cu)
        *output_value = ftm->initial;
    else if (offset == 0x50u) {
        uint32_t status = 0;
        const uint8_t channels = kinetis_timing_internal_ftm_channel_count(timing, instance);
        for (uint8_t channel = 0; channel < channels; channel++) {
            status |= ((ftm->channel_sc[channel] >> 7u) & 1u) << channel;
        }
        *output_value = status;
    } else if (offset >= 0x54u && offset <= 0x98u) {
        *output_value = ftm->registers[(offset - 0x54u) / 4u];
        if (offset == 0x6cu && (*output_value & 0x80u) != 0u)
            ftm->trigger_flag_read = true;
        if (offset == 0x74u) {
            if ((*output_value & 0x40u) != 0u)
                ftm->write_protection_read = true;
            ftm->fault_flags_read_mask |= (uint8_t)(*output_value & 0x0fu);
            if ((*output_value & 0x80u) != 0u)
                ftm->fault_aggregate_read = true;
        }
    } else
        return false;
    return true;
}
