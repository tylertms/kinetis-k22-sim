#include "architecture/cortex_m4/internal.h"

#include <string.h>

#define CCR_NONBASETHRDENA 1u
#define SCR_SLEEPONEXIT (1u << 1)
#define CFSR_MUNSTKERR (1u << 3)
#define CFSR_MSTKERR (1u << 4)
#define CFSR_MLSPERR (1u << 5)
#define CFSR_UNSTKERR (1u << 11)
#define CFSR_STKERR (1u << 12)
#define CFSR_LSPERR (1u << 13)
#define CFSR_IMPRECISERR (1u << 10)
#define HFSR_VECTTBL (1u << 1)
#define XPSR_EXCEPTION_MASK 0x1ffu
#define XPSR_ICI_IT_MASK 0x0600fc00u
#define XPSR_ICI_HIGH_MASK 0x06000000u
#define XPSR_ICI_LOW_MASK 0x00000c00u

static bool external_pending(const CortexM4* cpu, uint16_t exception) {
    const uint16_t irq = exception - 16u;
    const uint32_t irq_mask = 1u << (irq & 31u);
    return (cpu->irq_pending[irq / 32u] & cpu->irq_enabled[irq / 32u] & irq_mask) != 0;
}

static bool system_pending(const CortexM4* cpu, uint16_t exception) {
    return (cpu->system_pending & (1u << exception)) != 0;
}

static bool exception_pending(const CortexM4* cpu, uint16_t exception) {
    return exception < 16u ? system_pending(cpu, exception) : external_pending(cpu, exception);
}

bool cortex_m4_exception_advanced_active(const CortexM4* cpu, uint16_t exception) {
    if (cpu == NULL || exception < 2u) {
        return false;
    }
    for (uint8_t active_index = 0; active_index < cpu->exception_depth; active_index++) {
        if (cpu->active_exceptions[active_index] == exception) {
            return true;
        }
    }
    return false;
}

static uint16_t selected_pending(const CortexM4* cpu, uint16_t preemption_target,
                                 uint16_t returning_exception) {
    uint16_t selected_exception = 0;
    for (uint16_t exception = 2u; exception < cpu->external_irq_count + 16u; exception++) {
        if (exception == 7u || exception == 8u || exception == 9u || exception == 10u ||
            exception == 13u || !exception_pending(cpu, exception) ||
            !cortex_m4_system_exception_can_preempt(cpu, exception, preemption_target)) {
            continue;
        }
        if (cortex_m4_exception_advanced_active(cpu, exception) &&
            exception != returning_exception) {
            continue;
        }
        if (selected_exception == 0u ||
            cortex_m4_system_exception_before(cpu, exception, selected_exception)) {
            selected_exception = exception;
        }
    }
    return selected_exception;
}

static void clear_pending(CortexM4* cpu, uint16_t exception) {
    cortex_m4_system_set_pending(cpu, exception, false);
}

static void activate_external(CortexM4* cpu, uint16_t exception) {
    if (exception < 16u) {
        return;
    }
    const uint16_t irq = exception - 16u;
    cpu->irq_active[irq / 32u] |= 1u << (irq & 31u);
}

static void deactivate_external(CortexM4* cpu, uint16_t exception) {
    if (exception < 16u) {
        return;
    }
    const uint16_t irq = exception - 16u;
    const uint32_t irq_mask = 1u << (irq & 31u);
    cpu->irq_active[irq / 32u] &= ~irq_mask;
    if ((cpu->irq_level[irq / 32u] & irq_mask) != 0u) {
        cpu->irq_pending[irq / 32u] |= irq_mask;
    }
}

void cortex_m4_exception_advanced_reset(CortexM4* cpu) {
    if (cpu == NULL) {
        return;
    }
    memset(cpu->active_exceptions, 0, sizeof(cpu->active_exceptions));
    cpu->exception_depth = 0;
    cpu->ici_address = 0;
    cpu->ici_register = 0;
    cpu->ici_valid = false;
}

uint16_t cortex_m4_exception_advanced_late_arrival(CortexM4* cpu, uint16_t entering_exception) {
    if (cpu == NULL || entering_exception < 2u) {
        return entering_exception;
    }
    const uint16_t candidate = selected_pending(cpu, entering_exception, 0);
    if (candidate == 0u || !cortex_m4_system_exception_before(cpu, candidate, entering_exception)) {
        return entering_exception;
    }
    return candidate;
}

void cortex_m4_exception_advanced_commit_entry(CortexM4* cpu, uint16_t exception) {
    if (cpu == NULL || exception < 2u || cpu->exception_depth >= CORTEX_M4_EXCEPTION_FRAME_LIMIT) {
        if (cpu != NULL) {
            cpu->stop = CORTEX_M4_STOP_LOCKUP;
        }
        return;
    }
    cpu->active_exceptions[cpu->exception_depth++] = exception;
    clear_pending(cpu, exception);
    activate_external(cpu, exception);
    cpu->ici_address = 0;
    cpu->ici_register = 0;
    cpu->ici_valid = false;
}

void cortex_m4_exception_advanced_commit_return(CortexM4* cpu, uint16_t current_exception,
                                                bool returns_to_thread) {
    if (cpu == NULL || cpu->exception_depth == 0u ||
        cpu->active_exceptions[cpu->exception_depth - 1u] != current_exception) {
        if (cpu != NULL) {
            cpu->cfsr |= 1u << 18;
        }
        return;
    }
    deactivate_external(cpu, current_exception);
    cpu->active_exceptions[--cpu->exception_depth] = 0;
    if (current_exception != 2u) {
        cpu->faultmask = 0;
    }
    if (returns_to_thread && (cpu->scr & SCR_SLEEPONEXIT) != 0u) {
        cortex_m4_system_wait_for_interrupt(cpu);
    }
}

bool cortex_m4_exception_advanced_valid_return(const CortexM4* cpu, uint32_t exception_return) {
    if (cpu == NULL || cpu->exception_depth == 0u || cpu->exception_frame_depth == 0u ||
        (cpu->xpsr & XPSR_EXCEPTION_MASK) == 0u) {
        return false;
    }
    const bool basic = exception_return == 0xfffffff1u || exception_return == 0xfffffff9u ||
                       exception_return == 0xfffffffdu;
    const bool canonical = basic ||
                           (cpu->architecture != CORTEX_M4_ARCHITECTURE_ARMV6_M &&
                            (exception_return == 0xffffffe1u || exception_return == 0xffffffe9u ||
                             exception_return == 0xffffffedu));
    if (!canonical ||
        cpu->active_exceptions[cpu->exception_depth - 1u] != (cpu->xpsr & XPSR_EXCEPTION_MASK)) {
        return false;
    }
    const CortexM4ExceptionFrame* const frame =
        &cpu->exception_frames[cpu->exception_frame_depth - 1u];
    if (((frame->return_value ^ exception_return) & 0x14u) != 0u) {
        return false;
    }
    const bool returns_to_thread = (exception_return & (1u << 3)) != 0u;
    if (returns_to_thread) {
        return cpu->exception_depth == 1u || (cpu->ccr & CCR_NONBASETHRDENA) != 0u;
    }
    return cpu->exception_depth > 1u && (exception_return & (1u << 2)) == 0u;
}

bool cortex_m4_exception_advanced_valid_stacked_xpsr(const CortexM4* cpu, uint32_t stacked_xpsr,
                                                     uint32_t exception_return) {
    if (cpu == NULL || (stacked_xpsr & CORTEX_M4_XPSR_T) == 0u) {
        return false;
    }
    const bool returns_to_thread = (exception_return & (1u << 3)) != 0u;
    const uint16_t stacked_exception = (uint16_t)(stacked_xpsr & XPSR_EXCEPTION_MASK);
    const bool has_ici_state = cpu->architecture != CORTEX_M4_ARCHITECTURE_ARMV6_M &&
                               (stacked_xpsr & (XPSR_ICI_HIGH_MASK | XPSR_ICI_LOW_MASK)) == 0u &&
                               ((stacked_xpsr >> 12u) & 15u) != 0u;
    if (has_ici_state) {
        if (cpu->exception_frame_depth == 0u) {
            return false;
        }
        const CortexM4ExceptionFrame* const frame =
            &cpu->exception_frames[cpu->exception_frame_depth - 1u];
        if (!frame->ici_valid || frame->ici_register != (uint8_t)((stacked_xpsr >> 12u) & 15u)) {
            return false;
        }
    }
    if (returns_to_thread) {
        return stacked_exception == 0u;
    }
    return cpu->exception_depth > 1u &&
           stacked_exception == cpu->active_exceptions[cpu->exception_depth - 2u];
}

static bool record_fault_status(CortexM4* cpu, CortexM4ExceptionFaultStage stage,
                                bool memory_management_fault) {
    static const uint32_t memory_bits[] = {CFSR_MSTKERR, CFSR_MUNSTKERR, CFSR_MLSPERR};
    static const uint32_t bus_bits[] = {CFSR_STKERR, CFSR_UNSTKERR, CFSR_LSPERR};
    if (cpu == NULL || stage > CORTEX_M4_FAULT_LAZY_FP) {
        return false;
    }
    cpu->cfsr |= memory_management_fault ? memory_bits[stage] : bus_bits[stage];
    return true;
}

void cortex_m4_exception_advanced_fault(CortexM4* cpu, CortexM4ExceptionFaultStage stage,
                                        bool memory_management_fault) {
    if (!record_fault_status(cpu, stage, memory_management_fault)) {
        return;
    }
    cortex_m4_raise_fault(cpu, memory_management_fault ? 4u : 5u);
}

void cortex_m4_exception_advanced_entry_fault(CortexM4* cpu, uint16_t entering_exception,
                                              CortexM4ExceptionFaultStage stage,
                                              bool memory_management_fault) {
    if (!record_fault_status(cpu, stage, memory_management_fault)) {
        return;
    }
    const uint16_t fault_exception = memory_management_fault ? 4u : 5u;
    if (entering_exception == 2u || entering_exception == 3u) {
        cpu->stop = CORTEX_M4_STOP_LOCKUP;
        return;
    }
    if (entering_exception == fault_exception) {
        cpu->hfsr |= 1u << 30;
        cortex_m4_system_set_pending(cpu, 3u, true);
        return;
    }
    cortex_m4_raise_fault(cpu, (uint8_t)fault_exception);
}

void cortex_m4_exception_advanced_vector_fault(CortexM4* cpu) {
    if (cpu == NULL) {
        return;
    }
    const uint16_t current_exception = (uint16_t)(cpu->xpsr & XPSR_EXCEPTION_MASK);
    cpu->hfsr |= HFSR_VECTTBL;
    if (current_exception == 2u || current_exception == 3u) {
        cpu->stop = CORTEX_M4_STOP_LOCKUP;
        return;
    }
    cortex_m4_system_set_pending(cpu, 3u, true);
}

bool cortex_m4_exception_advanced_hardfault_vector(CortexM4* cpu, uint32_t* vector_address) {
    if (cpu == NULL || vector_address == NULL || cpu->stop == CORTEX_M4_STOP_LOCKUP) {
        return false;
    }
    cortex_m4_exception_vector_fetch(cpu);
    if (!cortex_m4_bus_read(cpu, cpu->vtor + 12u, 4, CORTEX_M4_ACCESS_DATA, vector_address) ||
        (*vector_address & 1u) == 0u) {
        cpu->stop = CORTEX_M4_STOP_LOCKUP;
        return false;
    }
    return true;
}

void cortex_m4_exception_advanced_imprecise_fault(CortexM4* cpu) {
    if (cpu == NULL) {
        return;
    }
    cpu->cfsr |= CFSR_IMPRECISERR;
    cortex_m4_raise_fault(cpu, 5u);
}

CortexM4ExceptionChain cortex_m4_exception_advanced_tail_chain(CortexM4* cpu,
                                                               uint32_t exception_return,
                                                               uint16_t current_exception) {
    if (!cortex_m4_exception_advanced_valid_return(cpu, exception_return)) {
        return CORTEX_M4_EXCEPTION_CHAIN_FAULT;
    }
    const bool returns_to_thread = (exception_return & (1u << 3)) != 0u;
    const uint16_t preemption_target =
        returns_to_thread ? 0u : cpu->active_exceptions[cpu->exception_depth - 2u];
    if (current_exception >= 16u) {
        const uint16_t irq = current_exception - 16u;
        const uint32_t irq_mask = 1u << (irq & 31u);
        if ((cpu->irq_level[irq / 32u] & irq_mask) != 0u) {
            cpu->irq_pending[irq / 32u] |= irq_mask;
        }
    }
    uint16_t selected_exception = selected_pending(cpu, preemption_target, current_exception);
    if (selected_exception == 0u) {
        return CORTEX_M4_EXCEPTION_CHAIN_NONE;
    }
    uint32_t vector_address = 0;
    cortex_m4_exception_vector_fetch(cpu);
    if (!cortex_m4_bus_read(cpu, cpu->vtor + selected_exception * 4u, 4, CORTEX_M4_ACCESS_DATA,
                            &vector_address) ||
        (vector_address & 1u) == 0u) {
        cortex_m4_exception_advanced_vector_fault(cpu);
        if (!cortex_m4_exception_advanced_hardfault_vector(cpu, &vector_address)) {
            return CORTEX_M4_EXCEPTION_CHAIN_FAULT;
        }
        selected_exception = 3u;
    }
    deactivate_external(cpu, current_exception);
    clear_pending(cpu, selected_exception);
    activate_external(cpu, selected_exception);
    cpu->active_exceptions[cpu->exception_depth - 1u] = selected_exception;
    cpu->registers[14] = exception_return;
    cpu->registers[15] = vector_address & ~1u;
    cpu->xpsr = (cpu->xpsr & ~XPSR_EXCEPTION_MASK) | selected_exception | CORTEX_M4_XPSR_T;
    cpu->it_state = 0;
    cpu->ici_register = 0;
    cpu->ici_valid = false;
    cpu->sleeping = false;
    cpu->exclusive_valid = false;
    if (current_exception != 2u) {
        cpu->faultmask = 0;
    }
    cortex_m4_debug_exception(cpu, selected_exception);
    cortex_m4_debug_exception_cycles(cpu, 6u);
    cortex_m4_timing_exception(cpu, CORTEX_M4_TIMING_EXCEPTION_TAIL_CHAIN);
    return CORTEX_M4_EXCEPTION_CHAIN_TAKEN;
}

uint8_t cortex_m4_exception_advanced_multiple_resume(const CortexM4* cpu) {
    return cpu != NULL && cpu->architecture != CORTEX_M4_ARCHITECTURE_ARMV6_M && cpu->ici_valid
               ? cpu->ici_register
               : 0u;
}

uint32_t cortex_m4_exception_advanced_multiple_address(const CortexM4* cpu,
                                                       uint32_t initial_address) {
    return cpu != NULL && cpu->architecture != CORTEX_M4_ARCHITECTURE_ARMV6_M && cpu->ici_valid
               ? cpu->ici_address
               : initial_address;
}

bool cortex_m4_exception_advanced_multiple_suspend(CortexM4* cpu, uint8_t next_register,
                                                   uint8_t instruction_size,
                                                   uint32_t next_address) {
    if (cpu == NULL || next_register == 0u || next_register > 15u ||
        (instruction_size != 2u && instruction_size != 4u)) {
        return false;
    }
    const uint16_t current_exception = (uint16_t)(cpu->xpsr & XPSR_EXCEPTION_MASK);
    if (selected_pending(cpu, current_exception, 0u) == 0u) {
        return false;
    }
    cpu->registers[15] -= instruction_size;
    if (cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M) {
        cpu->ici_address = 0u;
        cpu->ici_register = 0u;
        cpu->ici_valid = false;
        return true;
    }
    cpu->ici_address = next_address;
    cpu->ici_register = next_register;
    cpu->ici_valid = true;
    return true;
}

void cortex_m4_exception_advanced_multiple_complete(CortexM4* cpu) {
    if (cpu != NULL) {
        cpu->ici_address = 0;
        cpu->ici_register = 0;
        cpu->ici_valid = false;
    }
}

uint32_t cortex_m4_exception_advanced_xpsr(const CortexM4* cpu, uint32_t xpsr_value) {
    if (cpu == NULL || cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M || !cpu->ici_valid) {
        return xpsr_value;
    }
    return (xpsr_value & ~XPSR_ICI_IT_MASK) | ((uint32_t)cpu->ici_register << 12u);
}

void cortex_m4_exception_advanced_load_xpsr(CortexM4* cpu, uint32_t xpsr_value) {
    if (cpu == NULL) {
        return;
    }
    const bool ici = cpu->architecture != CORTEX_M4_ARCHITECTURE_ARMV6_M &&
                     (xpsr_value & (XPSR_ICI_HIGH_MASK | XPSR_ICI_LOW_MASK)) == 0u &&
                     ((xpsr_value >> 12u) & 15u) != 0u;
    cpu->ici_valid = ici;
    cpu->ici_register = ici ? (uint8_t)((xpsr_value >> 12u) & 15u) : 0u;
    if (ici && cpu->exception_frame_depth != 0u) {
        const CortexM4ExceptionFrame* const frame =
            &cpu->exception_frames[cpu->exception_frame_depth - 1u];
        cpu->ici_address =
            frame->ici_valid && frame->ici_register == cpu->ici_register ? frame->ici_address : 0u;
    } else if (!ici) {
        cpu->ici_address = 0;
    }
    if (ici) {
        cpu->it_state = 0;
    } else if (cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M) {
        cpu->it_state = 0;
    }
}
