#include "architecture/cortex_m4/internal.h"

#include <string.h>

#include "test.h"

typedef struct {
    uint8_t memory[4096];
    bool reject_reads;
    bool reject_writes;
    uint32_t reads;
    uint32_t writes;
    uint32_t fail_read;
    uint32_t fail_write;
} ExceptionBus;

static bool bus_read(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                     uint32_t* value) {
    ExceptionBus* bus = context;
    (void)access;
    bus->reads++;
    if (bus->reject_reads || bus->reads == bus->fail_read ||
        (uint64_t)address + size > sizeof(bus->memory)) {
        return false;
    }
    *value = 0u;
    memcpy(value, bus->memory + address, size);
    return true;
}

static bool bus_write(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                      uint32_t value) {
    ExceptionBus* bus = context;
    (void)access;
    bus->writes++;
    if (bus->reject_writes || bus->writes == bus->fail_write ||
        (uint64_t)address + size > sizeof(bus->memory)) {
        return false;
    }
    memcpy(bus->memory + address, &value, size);
    return true;
}

static CortexM4* create_cpu(TestState* state, ExceptionBus* bus) {
    CortexM4* cpu = cortex_m4_create((CortexM4Bus){bus, bus_read, bus_write, NULL, NULL});
    expect(state, cpu != NULL, "cpu != NULL");
    cortex_m4_system_reset(cpu);
    cpu->xpsr = CORTEX_M4_XPSR_T;
    cpu->msp = 0x800u;
    cpu->psp = 0x900u;
    cpu->registers[15] = 0x100u;
    return cpu;
}

static void write_vector(ExceptionBus* bus, uint16_t exception, uint32_t vector) {
    memcpy(bus->memory + exception * 4u, &vector, sizeof(vector));
}

static void write_word(ExceptionBus* bus, uint32_t address, uint32_t value) {
    memcpy(bus->memory + address, &value, sizeof(value));
}

static void write_halfword(ExceptionBus* bus, uint32_t address, uint16_t value) {
    memcpy(bus->memory + address, &value, sizeof(value));
}

static uint32_t read_word(const ExceptionBus* bus, uint32_t address) {
    uint32_t value = 0u;
    memcpy(&value, bus->memory + address, sizeof(value));
    return value;
}

static void seed_core_state(CortexM4* cpu) {
    cpu->registers[0] = 0x101u;
    cpu->registers[1] = 0x202u;
    cpu->registers[2] = 0x303u;
    cpu->registers[3] = 0x404u;
    cpu->registers[12] = 0x1212u;
    cpu->registers[14] = 0x1414u;
    cpu->registers[15] = 0x1514u;
}

static void write_core_frame(ExceptionBus* bus, uint32_t address) {
    static const uint32_t frame[] = {
        0x501u, 0x602u, 0x703u, 0x804u, 0x5212u, 0x5414u, 0x5515u, CORTEX_M4_XPSR_T,
    };
    memcpy(bus->memory + address, frame, sizeof(frame));
}

static void deny_mpu_region(CortexM4* cpu, uint32_t address) {
    cpu->mpu_control = 5u;
    cpu->mpu_region_base[0] = address;
    cpu->mpu_region_attributes[0] = (4u << 1u) | 1u;
}

static void prepare_active(CortexM4* cpu, uint16_t exception, uint32_t return_value,
                           uint32_t stack_pointer) {
    cpu->irq_pending[(exception - 16u) / 32u] |= 1u << ((exception - 16u) & 31u);
    cortex_m4_exception_advanced_commit_entry(cpu, exception);
    cpu->xpsr = CORTEX_M4_XPSR_T | exception;
    cpu->msp = stack_pointer;
    CortexM4ExceptionFrame* frame = &cpu->exception_frames[cpu->exception_frame_depth++];
    memset(frame, 0, sizeof(*frame));
    frame->address = stack_pointer;
    frame->return_value = return_value;
}

static void test_fault_lockup(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->xpsr = CORTEX_M4_XPSR_T | 3u;
    cortex_m4_raise_fault(cpu, 4u);
    expect(state, cpu->stop == CORTEX_M4_STOP_LOCKUP, "cpu->stop == CORTEX_M4_STOP_LOCKUP");
    cortex_m4_destroy(cpu);
}

static void test_psp_lifecycle(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->control = CORTEX_M4_CONTROL_SPSEL;
    cpu->irq_enabled[0] = 1u;
    cpu->irq_pending[0] = 1u;
    write_vector(&bus, 16u, 0x301u);
    expect(state, cortex_m4_take_pending_exception(cpu), "cortex_m4_take_pending_exception(cpu)");
    expect(state, cpu->psp == 0x8e0u, "cpu->psp == 0x8e0u");
    expect(state, cpu->registers[14] == 0xfffffffdu, "cpu->registers[14] == 0xfffffffdu");
    expect(state, cortex_m4_exception_return(cpu, cpu->registers[14]),
           "cortex_m4_exception_return(cpu, cpu->registers[14])");
    expect(state, cpu->psp == 0x900u, "cpu->psp == 0x900u");
    expect(state, (cpu->xpsr & 0x1ffu) == 0u, "(cpu->xpsr & 0x1ffu) == 0u");
    cortex_m4_destroy(cpu);
}

static void test_entry_failures(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->irq_enabled[0] = 1u;
    cpu->irq_pending[0] = 1u;
    write_vector(&bus, 16u, 0x301u);
    bus.reject_writes = true;
    expect(state, !cortex_m4_take_pending_exception(cpu), "!cortex_m4_take_pending_exception(cpu)");
    expect(state, (cpu->cfsr & (1u << 12)) != 0u, "(cpu->cfsr & (1u << 12)) != 0u");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    cpu->irq_enabled[0] = 1u;
    cpu->irq_pending[0] = 1u;
    bus.reject_reads = true;
    expect(state, !cortex_m4_take_pending_exception(cpu), "!cortex_m4_take_pending_exception(cpu)");
    expect(state, (cpu->hfsr & (1u << 1)) != 0u, "(cpu->hfsr & (1u << 1)) != 0u");
    cortex_m4_destroy(cpu);
}

static void test_step_entry_failure(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->irq_enabled[0] = 1u;
    cpu->irq_pending[0] = 1u;
    write_halfword(&bus, 0x100u, 0x2001u);
    write_vector(&bus, 16u, 0x301u);
    bus.fail_write = 1u;

    const CortexM4Result result = cortex_m4_step(cpu);

    expect(state, result.stop == CORTEX_M4_STOP_RUNNING,
           "stacking fault leaves the escalated HardFault runnable");
    expect(state, result.instructions == 0u && result.pc == 0x100u && cpu->registers[0] == 0u,
           "failed exception entry does not retire the interrupted instruction");
    expect(state,
           (cpu->cfsr & (1u << 12u)) != 0u && (cpu->hfsr & (1u << 30u)) != 0u &&
               (cpu->system_pending & (1u << 3u)) != 0u,
           "failed exception entry escalates its stacking fault");
    cortex_m4_destroy(cpu);
}

static void test_core_entry_write_failures(TestState* state) {
    static const uint32_t frame[] = {
        0x101u, 0x202u, 0x303u, 0x404u, 0x1212u, 0x1414u, 0x1514u, CORTEX_M4_XPSR_T,
    };
    static const uint32_t failed_writes[] = {4u, 8u};
    for (size_t failure = 0; failure < sizeof(failed_writes) / sizeof(failed_writes[0]);
         failure++) {
        ExceptionBus bus = {0};
        CortexM4* cpu = create_cpu(state, &bus);
        seed_core_state(cpu);
        cpu->irq_enabled[0] = 1u;
        cpu->irq_pending[0] = 1u;
        write_vector(&bus, 16u, 0x301u);
        bus.fail_write = failed_writes[failure];

        expect(state, !cortex_m4_take_pending_exception(cpu),
               "core frame write fault rejects exception entry");
        expect(state, cpu->msp == 0x800u && cpu->exception_frame_depth == 0u,
               "core frame write fault preserves stack state");
        expect(state, (cpu->cfsr & (1u << 12)) != 0u && (cpu->hfsr & (1u << 30)) != 0u,
               "core frame write fault records stacking escalation");
        expect(state, (cpu->system_pending & (1u << 3)) != 0u,
               "core frame write fault pends HardFault");
        for (uint32_t index = 0; index < 8u; index++) {
            const uint32_t expected = index + 1u < failed_writes[failure] ? frame[index] : 0u;
            expect(state, read_word(&bus, 0x7e0u + index * 4u) == expected,
                   "core frame write fault preserves only completed transfers");
        }
        cortex_m4_destroy(cpu);
    }
}

static void test_fp_entry_write_failures(TestState* state) {
    static const uint32_t core_frame[] = {
        0x101u, 0x202u, 0x303u, 0x404u, 0x1212u, 0x1414u, 0x1514u, CORTEX_M4_XPSR_T,
    };
    static const uint32_t failed_writes[] = {8u, 18u, 22u, 26u};
    for (size_t failure = 0; failure < sizeof(failed_writes) / sizeof(failed_writes[0]);
         failure++) {
        ExceptionBus bus = {0};
        CortexM4* cpu = create_cpu(state, &bus);
        seed_core_state(cpu);
        cpu->control = CORTEX_M4_CONTROL_FPCA;
        cpu->fpccr &= ~(1u << 30);
        for (uint32_t index = 0; index < 16u; index++) {
            cpu->fp_registers[index] = 0xf000u + index;
        }
        cpu->fpscr = 0xf5c00000u;
        const uint32_t fpccr = cpu->fpccr;
        const uint32_t fpcar = cpu->fpcar;
        cpu->irq_enabled[0] = 1u;
        cpu->irq_pending[0] = 1u;
        write_vector(&bus, 16u, 0x301u);
        bus.fail_write = failed_writes[failure];

        expect(state, !cortex_m4_take_pending_exception(cpu),
               "FP frame write fault rejects exception entry");
        expect(state, cpu->msp == 0x800u && cpu->exception_frame_depth == 0u,
               "FP frame write fault preserves stack state");
        expect(state, cpu->fpccr == fpccr && cpu->fpcar == fpcar,
               "FP frame write fault preserves FP stack bookkeeping");
        expect(state, (cpu->cfsr & (1u << 12)) != 0u && (cpu->hfsr & (1u << 30)) != 0u,
               "FP frame write fault records stacking escalation");
        for (uint32_t index = 0; index < 16u; index++) {
            const uint32_t expected = index + 1u < failed_writes[failure] ? 0xf000u + index : 0u;
            expect(state, read_word(&bus, 0x798u + index * 4u) == expected,
                   "FP frame write fault preserves only completed transfers");
        }
        expect(state, read_word(&bus, 0x7d8u) == (failed_writes[failure] > 17u ? 0xf5c00000u : 0u),
               "FP frame write fault preserves completed FPSCR transfer");
        expect(state, read_word(&bus, 0x7dcu) == 0u,
               "FP frame write fault leaves its reserved word clear");
        for (uint32_t index = 0u; index < 8u; index++) {
            const uint32_t expected = 19u + index < failed_writes[failure] ? core_frame[index] : 0u;
            expect(state, read_word(&bus, 0x7e0u + index * 4u) == expected,
                   "extended entry fault preserves only completed core writes");
        }
        cortex_m4_destroy(cpu);
    }
}

static void test_mpu_entry_failures(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->msp = 0x804u;
    cpu->irq_enabled[0] = 1u;
    cpu->irq_pending[0] = 1u;
    write_vector(&bus, 16u, 0x301u);
    deny_mpu_region(cpu, 0x7e0u);
    expect(state, !cortex_m4_take_pending_exception(cpu),
           "core frame MPU fault rejects exception entry");
    expect(state,
           (cpu->cfsr & (1u << 4u)) != 0u && (cpu->cfsr & (1u << 12u)) == 0u &&
               (cpu->cfsr & (1u << 1u)) != 0u && cpu->mmfar == 0x7e0u,
           "core frame MPU fault records MSTKERR at the denied word");
    expect(state, cpu->msp == 0x804u && cpu->exception_frame_depth == 0u,
           "core frame MPU fault preserves stack state");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    cpu->control = CORTEX_M4_CONTROL_FPCA;
    cpu->fpccr &= ~(1u << 30u);
    cpu->fp_registers[0] = 0xf000u;
    cpu->fp_registers[1] = 0xf001u;
    cpu->irq_enabled[0] = 1u;
    cpu->irq_pending[0] = 1u;
    write_vector(&bus, 16u, 0x301u);
    deny_mpu_region(cpu, 0x7a0u);
    expect(state, !cortex_m4_take_pending_exception(cpu),
           "extended frame MPU fault rejects exception entry");
    expect(state,
           read_word(&bus, 0x798u) == 0xf000u && read_word(&bus, 0x79cu) == 0xf001u &&
               read_word(&bus, 0x7a0u) == 0u,
           "extended frame MPU fault preserves completed writes");
    expect(state,
           (cpu->cfsr & (1u << 4u)) != 0u && (cpu->cfsr & (1u << 12u)) == 0u &&
               (cpu->cfsr & (1u << 1u)) != 0u && cpu->mmfar == 0x7a0u,
           "extended frame MPU fault records MSTKERR at the denied word");
    expect(state, cpu->msp == 0x800u && cpu->exception_frame_depth == 0u,
           "extended frame MPU fault preserves stack state");
    cortex_m4_destroy(cpu);
}

static void test_exception_selection(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    cpu->system_pending = (1u << 14) | (1u << 15);
    cpu->system_priority[10] = 0x80u;
    cpu->system_priority[11] = 0x10u;
    write_vector(&bus, 14u, 0x301u);
    write_vector(&bus, 15u, 0x321u);
    expect(state, cortex_m4_take_pending_exception(cpu), "cortex_m4_take_pending_exception(cpu)");
    expect(state, (cpu->xpsr & 0x1ffu) == 15u, "(cpu->xpsr & 0x1ffu) == 15u");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    cpu->irq_enabled[0] = 3u;
    cpu->irq_pending[0] = 3u;
    cpu->irq_priority[0] = 0x80u;
    cpu->irq_priority[1] = 0x10u;
    write_vector(&bus, 16u, 0x301u);
    write_vector(&bus, 17u, 0x321u);
    expect(state, cortex_m4_take_pending_exception(cpu), "cortex_m4_take_pending_exception(cpu)");
    expect(state, (cpu->xpsr & 0x1ffu) == 17u, "(cpu->xpsr & 0x1ffu) == 17u");
    cortex_m4_destroy(cpu);
}

static void test_invalid_return(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    prepare_active(cpu, 16u, 0xfffffff9u, 0x400u);
    expect(state, !cortex_m4_exception_return(cpu, 0xfffffff8u),
           "!cortex_m4_exception_return(cpu, 0xfffffff8u)");
    expect(state, (cpu->cfsr & (1u << 18)) != 0u, "(cpu->cfsr & (1u << 18)) != 0u");
    cortex_m4_destroy(cpu);
}

static void test_return_failures(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    prepare_active(cpu, 16u, 0xfffffff9u, 0x400u);
    cpu->irq_enabled[0] = 3u;
    cpu->irq_pending[0] |= 2u;
    bus.reject_reads = true;
    expect(state, cortex_m4_exception_return(cpu, 0xfffffff9u),
           "cortex_m4_exception_return(cpu, 0xfffffff9u)");
    expect(state, (cpu->hfsr & (1u << 1)) != 0u, "(cpu->hfsr & (1u << 1)) != 0u");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    prepare_active(cpu, 16u, 0xfffffff9u, 0x400u);
    bus.reject_reads = true;
    expect(state, cortex_m4_exception_return(cpu, 0xfffffff9u),
           "cortex_m4_exception_return(cpu, 0xfffffff9u)");
    expect(state, cpu->exception_unstack_memory_fault, "cpu->exception_unstack_memory_fault");
    expect(state, (cpu->cfsr & (1u << 11)) != 0u, "(cpu->cfsr & (1u << 11)) != 0u");
    cortex_m4_destroy(cpu);

    memset(&bus, 0, sizeof(bus));
    cpu = create_cpu(state, &bus);
    prepare_active(cpu, 16u, 0xfffffff9u, 0x400u);
    expect(state, cortex_m4_exception_return(cpu, 0xfffffff9u),
           "cortex_m4_exception_return(cpu, 0xfffffff9u)");
    expect(state, !cpu->exception_unstack_memory_fault, "!cpu->exception_unstack_memory_fault");
    expect(state, (cpu->cfsr & (1u << 17)) != 0u, "(cpu->cfsr & (1u << 17)) != 0u");
    cortex_m4_destroy(cpu);
}

static void test_core_return_read_failures(TestState* state) {
    static const uint32_t failed_reads[] = {4u, 8u};
    for (size_t failure = 0; failure < sizeof(failed_reads) / sizeof(failed_reads[0]); failure++) {
        ExceptionBus bus = {0};
        CortexM4* cpu = create_cpu(state, &bus);
        prepare_active(cpu, 16u, 0xfffffff9u, 0x400u);
        seed_core_state(cpu);
        write_core_frame(&bus, 0x400u);
        const uint8_t frame_depth = cpu->exception_frame_depth;
        bus.fail_read = failed_reads[failure];

        expect(state, cortex_m4_exception_return(cpu, 0xfffffff9u),
               "core frame read fault completes fault transition");
        expect(state, cpu->exception_unstack_memory_fault && (cpu->cfsr & (1u << 11)) != 0u,
               "core frame read fault records unstacking failure");
        expect(state, (cpu->hfsr & (1u << 30)) != 0u && (cpu->system_pending & (1u << 3)) != 0u,
               "core frame read fault escalates to HardFault");
        expect(state, cpu->msp == 0x400u && cpu->exception_frame_depth == frame_depth,
               "core frame read fault preserves stack metadata");
        expect(state,
               cpu->registers[0] == 0x101u && cpu->registers[3] == 0x404u &&
                   cpu->registers[12] == 0x1212u && cpu->registers[15] == 0x1514u,
               "core frame read fault does not commit staged registers");
        cortex_m4_destroy(cpu);
    }
}

static void test_fp_return_read_failures(TestState* state) {
    static const uint32_t failed_reads[] = {16u, 25u};
    for (size_t failure = 0; failure < sizeof(failed_reads) / sizeof(failed_reads[0]); failure++) {
        ExceptionBus bus = {0};
        CortexM4* cpu = create_cpu(state, &bus);
        prepare_active(cpu, 16u, 0xffffffe9u, 0x400u);
        cpu->exception_frames[0].extended = true;
        seed_core_state(cpu);
        write_core_frame(&bus, 0x448u);
        for (uint32_t index = 0; index < 16u; index++) {
            write_word(&bus, 0x400u + index * 4u, 0xa000u + index);
            cpu->fp_registers[index] = 0xb000u + index;
        }
        write_word(&bus, 0x440u, 0xa5c00000u);
        cpu->fpscr = 0xb5c00000u;
        const uint8_t frame_depth = cpu->exception_frame_depth;
        bus.fail_read = failed_reads[failure];

        expect(state, cortex_m4_exception_return(cpu, 0xffffffe9u),
               "FP frame read fault completes fault transition");
        expect(state, cpu->exception_unstack_memory_fault && (cpu->cfsr & (1u << 11)) != 0u,
               "FP frame read fault records unstacking failure");
        expect(state, (cpu->hfsr & (1u << 30)) != 0u && (cpu->system_pending & (1u << 3)) != 0u,
               "FP frame read fault escalates to HardFault");
        expect(state, cpu->msp == 0x400u && cpu->exception_frame_depth == frame_depth,
               "FP frame read fault preserves stack metadata");
        expect(state, cpu->registers[0] == 0x101u && cpu->registers[15] == 0x1514u,
               "FP frame read fault does not commit staged core registers");
        bool fp_unchanged = cpu->fpscr == 0xb5c00000u;
        for (uint32_t index = 0; index < 16u; index++) {
            fp_unchanged = fp_unchanged && cpu->fp_registers[index] == 0xb000u + index;
        }
        expect(state, fp_unchanged, "FP frame read fault does not commit staged FP registers");
        cortex_m4_destroy(cpu);
    }
}

static void test_mpu_return_failure(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    prepare_active(cpu, 16u, 0xfffffff9u, 0x7e4u);
    seed_core_state(cpu);
    write_core_frame(&bus, 0x7e4u);
    const uint8_t frame_depth = cpu->exception_frame_depth;
    deny_mpu_region(cpu, 0x800u);

    expect(state, cortex_m4_exception_return(cpu, 0xfffffff9u),
           "late unstack MPU fault completes fault transition");
    expect(state,
           (cpu->cfsr & (1u << 3u)) != 0u && (cpu->cfsr & (1u << 11u)) == 0u &&
               (cpu->cfsr & (1u << 1u)) != 0u && cpu->mmfar == 0x800u,
           "late unstack MPU fault records MUNSTKERR at the denied word");
    expect(state,
           cpu->exception_unstack_memory_fault && cpu->msp == 0x7e4u &&
               cpu->exception_frame_depth == frame_depth && cpu->registers[0] == 0x101u &&
               cpu->registers[15] == 0x1514u,
           "late unstack MPU fault preserves staged state");
    cortex_m4_destroy(cpu);
}

static void test_tail_chain_return(TestState* state) {
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    prepare_active(cpu, 16u, 0xfffffff9u, 0x400u);
    cpu->irq_enabled[0] = 3u;
    cpu->irq_pending[0] |= 2u;
    cpu->registers[14] = 0xfffffff9u;
    write_vector(&bus, 17u, 0x321u);
    const uint32_t stack_pointer = cpu->msp;
    const uint8_t frame_depth = cpu->exception_frame_depth;

    expect(state, cortex_m4_execute_thumb16(cpu, UINT16_C(0x4770)),
           "BX LR performs an exception-return tail chain");
    expect(state,
           (cpu->xpsr & 0x1ffu) == 17u && cpu->registers[15] == 0x320u &&
               cpu->registers[14] == 0xfffffff9u,
           "tail chain enters the pending exception vector");
    expect(state,
           cpu->msp == stack_pointer && cpu->exception_frame_depth == frame_depth &&
               cpu->exception_depth == 1u,
           "tail chain reuses the existing exception frame");
    expect(state,
           (cpu->irq_active[0] & 1u) == 0u && (cpu->irq_active[0] & 2u) != 0u &&
               (cpu->irq_pending[0] & 2u) == 0u,
           "tail chain transfers active and pending state");
    expect(state, cpu->cycles == 6u, "tail chain uses architectural exception timing");
    cortex_m4_destroy(cpu);
}

static void test_armv6_m_motor_interrupt_arbitration(TestState* state) {
    enum {
        ADC0_IRQ = 15u,
        ADC1_IRQ = 16u,
        FTM3_IRQ = 22u,
        PDB_IRQ = 29u,
    };
    ExceptionBus bus = {0};
    CortexM4* cpu = create_cpu(state, &bus);
    expect(state, cortex_m4_configure_architecture(cpu, CORTEX_M4_ARCHITECTURE_ARMV6_M),
           "configure ARMv6-M motor interrupt processor");
    expect(state, cortex_m4_configure_implementation(cpu, 32u, 2u, 0u),
           "configure two-bit motor interrupt priorities");
    cpu->irq_enabled[0] = (1u << ADC0_IRQ) | (1u << ADC1_IRQ) | (1u << FTM3_IRQ) | (1u << PDB_IRQ);
    cpu->irq_priority[ADC0_IRQ] = 0x00u;
    cpu->irq_priority[ADC1_IRQ] = 0x80u;
    cpu->irq_priority[FTM3_IRQ] = 0x40u;
    cpu->irq_priority[PDB_IRQ] = 0x80u;
    write_vector(&bus, ADC0_IRQ + 16u, 0x301u);
    write_vector(&bus, ADC1_IRQ + 16u, 0x321u);
    write_vector(&bus, FTM3_IRQ + 16u, 0x341u);
    write_vector(&bus, PDB_IRQ + 16u, 0x361u);
    cortex_m4_set_irq_level(cpu, ADC0_IRQ, true);
    cortex_m4_set_irq_level(cpu, ADC1_IRQ, true);
    cortex_m4_set_irq_level(cpu, FTM3_IRQ, true);
    cortex_m4_set_irq_level(cpu, PDB_IRQ, true);
    cortex_m4_set_irq(cpu, ADC1_IRQ, false);
    cortex_m4_set_irq_level(cpu, ADC1_IRQ, true);
    expect(state, cortex_m4_get_irq_pending(cpu, ADC1_IRQ),
           "asserted inactive level restores cleared pending state");

    expect(state, cortex_m4_take_pending_exception(cpu), "take highest-priority motor interrupt");
    expect(state, (cpu->xpsr & 0x1ffu) == ADC0_IRQ + 16u,
           "ADC0 wins simultaneous motor interrupts");
    expect(state, !cortex_m4_take_pending_exception(cpu),
           "lower-priority motor interrupts do not preempt ADC0");

    cortex_m4_set_irq_level(cpu, ADC0_IRQ, true);
    expect(state, !cortex_m4_get_irq_pending(cpu, ADC0_IRQ),
           "repeated active level assertion does not queue a duplicate interrupt");
    expect(state, cortex_m4_exception_return(cpu, cpu->registers[14]),
           "continuously asserted ADC0 tail-chains");
    expect(state, (cpu->xpsr & 0x1ffu) == ADC0_IRQ + 16u,
           "continuously asserted ADC0 retains execution priority");

    cortex_m4_set_irq_level(cpu, ADC0_IRQ, false);
    expect(state, cortex_m4_exception_return(cpu, cpu->registers[14]),
           "deasserted ADC0 releases the pending motor interrupts");
    expect(state, (cpu->xpsr & 0x1ffu) == FTM3_IRQ + 16u,
           "FTM3 wins over continuously pending ADC1 and PDB");
    cortex_m4_destroy(cpu);
}

int main(void) {
    TestState state = {0};
    test_fault_lockup(&state);
    test_psp_lifecycle(&state);
    test_entry_failures(&state);
    test_step_entry_failure(&state);
    test_core_entry_write_failures(&state);
    test_fp_entry_write_failures(&state);
    test_mpu_entry_failures(&state);
    test_exception_selection(&state);
    test_invalid_return(&state);
    test_return_failures(&state);
    test_core_return_read_failures(&state);
    test_fp_return_read_failures(&state);
    test_mpu_return_failure(&state);
    test_tail_chain_return(&state);
    test_armv6_m_motor_interrupt_arbitration(&state);
    return test_finish(&state);
}
