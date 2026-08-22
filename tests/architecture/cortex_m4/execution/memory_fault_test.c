#include "architecture/cortex_m4/internal.h"

#include <string.h>

#include "test.h"

enum { MEMORY_BASE = 0x20000000u, MEMORY_SIZE = 4096u };

typedef struct {
    uint8_t memory[MEMORY_SIZE];
    uint32_t reads;
    uint32_t writes;
    uint32_t fail_read;
    uint32_t fail_write;
} FaultBus;

static bool bus_read(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                     uint32_t* value) {
    FaultBus* bus = context;
    (void)access;
    bus->reads++;
    if (bus->reads == bus->fail_read || address < MEMORY_BASE ||
        (uint64_t)(address - MEMORY_BASE) + size > sizeof(bus->memory)) {
        return false;
    }
    *value = 0u;
    memcpy(value, bus->memory + address - MEMORY_BASE, size);
    return true;
}

static bool bus_write(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                      uint32_t value) {
    FaultBus* bus = context;
    (void)access;
    bus->writes++;
    if (bus->writes == bus->fail_write || address < MEMORY_BASE ||
        (uint64_t)(address - MEMORY_BASE) + size > sizeof(bus->memory)) {
        return false;
    }
    memcpy(bus->memory + address - MEMORY_BASE, &value, size);
    return true;
}

static CortexM4 create_cpu(FaultBus* bus) {
    CortexM4 cpu;
    memset(&cpu, 0, sizeof(cpu));
    cpu.bus = (CortexM4Bus){bus, bus_read, bus_write, NULL, NULL};
    cpu.external_irq_count = CORTEX_M4_IRQ_COUNT;
    cpu.priority_bits = 8u;
    cpu.msp = MEMORY_BASE + MEMORY_SIZE;
    cpu.xpsr = CORTEX_M4_XPSR_T;
    return cpu;
}

static uint32_t memory_word(const FaultBus* bus, uint32_t address) {
    uint32_t value = 0u;
    memcpy(&value, bus->memory + address - MEMORY_BASE, sizeof(value));
    return value;
}

static void set_memory_word(FaultBus* bus, uint32_t address, uint32_t value) {
    memcpy(bus->memory + address - MEMORY_BASE, &value, sizeof(value));
}

static void test_push_pop_faults(TestState* state) {
    FaultBus bus = {.fail_write = 1u};
    CortexM4 cpu = create_cpu(&bus);
    expect(state, cortex_m4_execute_thumb16(&cpu, UINT16_C(0xb401)),
           "PUSH encoding is recognized when its write faults");
    expect(state, (cpu.cfsr & (1u << 9u)) != 0u, "PUSH records a precise bus fault");

    memset(&bus, 0, sizeof(bus));
    bus.fail_read = 1u;
    cpu = create_cpu(&bus);
    cpu.msp = MEMORY_BASE;
    expect(state, cortex_m4_execute_thumb16(&cpu, UINT16_C(0xbc01)),
           "POP encoding is recognized when its read faults");
    expect(state, (cpu.cfsr & (1u << 9u)) != 0u, "POP records a precise bus fault");

    memset(&bus, 0, sizeof(bus));
    bus.fail_write = 3u;
    cpu = create_cpu(&bus);
    cpu.registers[0] = 0x11111111u;
    cpu.registers[1] = 0x22222222u;
    cpu.registers[14] = 0xeeeeeeeeu;
    expect(state, cortex_m4_execute_thumb16(&cpu, UINT16_C(0xb503)),
           "PUSH encoding is recognized when the LR write faults");
    expect(state, bus.writes == 3u && cpu.bfar == MEMORY_BASE + MEMORY_SIZE - 4u,
           "PUSH reports the late failed LR address");
    expect(state,
           memory_word(&bus, MEMORY_BASE + MEMORY_SIZE - 12u) == 0x11111111u &&
               memory_word(&bus, MEMORY_BASE + MEMORY_SIZE - 8u) == 0x22222222u &&
               cpu.msp == MEMORY_BASE + MEMORY_SIZE,
           "PUSH preserves writes completed before a late fault");

    memset(&bus, 0, sizeof(bus));
    bus.fail_read = 2u;
    set_memory_word(&bus, MEMORY_BASE, 0x11111111u);
    set_memory_word(&bus, MEMORY_BASE + 4u, 0x22222222u);
    cpu = create_cpu(&bus);
    cpu.msp = MEMORY_BASE;
    cpu.registers[1] = 0xaaaaaaaau;
    expect(state, cortex_m4_execute_thumb16(&cpu, UINT16_C(0xbc03)),
           "POP encoding is recognized when a later read faults");
    expect(state,
           bus.reads == 2u && cpu.registers[0] == 0x11111111u && cpu.registers[1] == 0xaaaaaaaau &&
               cpu.msp == MEMORY_BASE && cpu.bfar == MEMORY_BASE + 4u,
           "POP commits only reads completed before a late fault");
}

static void test_multiple_transfer_faults(TestState* state) {
    FaultBus bus = {.fail_read = 1u};
    CortexM4 cpu = create_cpu(&bus);
    cpu.registers[0] = MEMORY_BASE;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe890), UINT16_C(0x0001)),
           "LDM encoding is recognized when its read faults");
    expect(state, cpu.bfar == MEMORY_BASE, "LDM reports the failed source address");

    memset(&bus, 0, sizeof(bus));
    bus.fail_write = 1u;
    cpu = create_cpu(&bus);
    cpu.registers[0] = MEMORY_BASE;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe880), UINT16_C(0x0001)),
           "STM encoding is recognized when its write faults");
    expect(state, cpu.bfar == MEMORY_BASE, "STM reports the failed destination address");

    memset(&bus, 0, sizeof(bus));
    bus.fail_read = 2u;
    set_memory_word(&bus, MEMORY_BASE, 0x11111111u);
    set_memory_word(&bus, MEMORY_BASE + 4u, 0x22222222u);
    cpu = create_cpu(&bus);
    cpu.registers[0] = MEMORY_BASE;
    cpu.registers[2] = 0xaaaaaaaau;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe8b0), UINT16_C(0x000e)),
           "LDM encoding is recognized when a later read faults");
    expect(state,
           cpu.registers[1] == 0x11111111u && cpu.registers[2] == 0xaaaaaaaau &&
               cpu.registers[0] == MEMORY_BASE && cpu.bfar == MEMORY_BASE + 4u,
           "LDM commits only reads completed before a late fault");

    memset(&bus, 0, sizeof(bus));
    bus.fail_write = 2u;
    cpu = create_cpu(&bus);
    cpu.registers[0] = MEMORY_BASE;
    cpu.registers[1] = 0x11111111u;
    cpu.registers[2] = 0x22222222u;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe8a0), UINT16_C(0x000e)),
           "STM encoding is recognized when a later write faults");
    expect(state,
           memory_word(&bus, MEMORY_BASE) == 0x11111111u &&
               memory_word(&bus, MEMORY_BASE + 4u) == 0u && cpu.registers[0] == MEMORY_BASE &&
               cpu.bfar == MEMORY_BASE + 4u,
           "STM commits only writes completed before a late fault");
}

static void test_decrement_before_faults(TestState* state) {
    for (uint32_t failed = 2u; failed <= 3u; failed++) {
        FaultBus bus = {.fail_read = failed};
        const uint32_t address = MEMORY_BASE + 0x14u;
        for (uint32_t index = 0u; index < 3u; index++) {
            set_memory_word(&bus, address + index * 4u, 0x11111111u * (index + 1u));
        }
        CortexM4 cpu = create_cpu(&bus);
        cpu.registers[4] = MEMORY_BASE + 0x20u;
        cpu.registers[0] = 0xaaaaaaaau;
        cpu.registers[1] = 0xbbbbbbbbu;
        cpu.registers[2] = 0xccccccccu;
        expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe934), UINT16_C(0x0007)),
               "LDMDB encoding is recognized when a later read faults");
        bool load_state = cpu.registers[4] == MEMORY_BASE + 0x20u;
        for (uint32_t index = 0u; index < 3u; index++) {
            const uint32_t initial[] = {0xaaaaaaaau, 0xbbbbbbbbu, 0xccccccccu};
            const uint32_t expected =
                index < failed - 1u ? 0x11111111u * (index + 1u) : initial[index];
            load_state = load_state && cpu.registers[index] == expected;
        }
        expect(state,
               load_state && (cpu.cfsr & (1u << 9u)) != 0u &&
                   cpu.bfar == address + (failed - 1u) * 4u,
               "LDMDB preserves partial loads and suppresses faulting writeback");

        memset(&bus, 0, sizeof(bus));
        bus.fail_write = failed;
        cpu = create_cpu(&bus);
        cpu.registers[4] = MEMORY_BASE + 0x20u;
        cpu.registers[0] = 0x11111111u;
        cpu.registers[1] = 0x22222222u;
        cpu.registers[2] = 0x33333333u;
        expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe924), UINT16_C(0x0007)),
               "STMDB encoding is recognized when a later write faults");
        bool store_state = cpu.registers[4] == MEMORY_BASE + 0x20u;
        for (uint32_t index = 0u; index < 3u; index++) {
            const uint32_t expected = index < failed - 1u ? 0x11111111u * (index + 1u) : 0u;
            store_state = store_state && memory_word(&bus, address + index * 4u) == expected;
        }
        expect(state,
               store_state && (cpu.cfsr & (1u << 9u)) != 0u &&
                   cpu.bfar == address + (failed - 1u) * 4u,
               "STMDB preserves partial stores and suppresses faulting writeback");
    }
}

static void test_doubleword_faults(TestState* state) {
    FaultBus bus = {.fail_read = 2u};
    const uint32_t load_address = MEMORY_BASE + 0x24u;
    set_memory_word(&bus, load_address, 0x11111111u);
    set_memory_word(&bus, load_address + 4u, 0x22222222u);
    CortexM4 cpu = create_cpu(&bus);
    cpu.registers[3] = 0xaaaaaaaau;
    cpu.registers[4] = 0xbbbbbbbbu;
    cpu.registers[5] = MEMORY_BASE + 0x30u;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe975), UINT16_C(0x3403)),
           "LDRD encoding is recognized when its final read faults");
    expect(state,
           cpu.registers[3] == 0xaaaaaaaau && cpu.registers[4] == 0xbbbbbbbbu &&
               cpu.registers[5] == MEMORY_BASE + 0x30u && cpu.bfar == load_address + 4u,
           "LDRD stages both loads and suppresses faulting writeback");

    memset(&bus, 0, sizeof(bus));
    bus.fail_write = 2u;
    cpu = create_cpu(&bus);
    cpu.registers[6] = 0x11111111u;
    cpu.registers[7] = 0x22222222u;
    cpu.registers[8] = MEMORY_BASE + 0x40u;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe8e8), UINT16_C(0x6704)),
           "STRD encoding is recognized when its final write faults");
    expect(state,
           memory_word(&bus, MEMORY_BASE + 0x40u) == 0x11111111u &&
               memory_word(&bus, MEMORY_BASE + 0x44u) == 0u &&
               cpu.registers[8] == MEMORY_BASE + 0x40u && cpu.bfar == MEMORY_BASE + 0x44u,
           "STRD preserves its first store and suppresses faulting writeback");
}

static void test_floating_multiple_faults(TestState* state) {
    for (uint32_t failed = 2u; failed <= 4u; failed += 2u) {
        FaultBus bus = {.fail_read = failed};
        const uint32_t load_address = MEMORY_BASE + 0x180u;
        for (uint32_t index = 0u; index < 4u; index++) {
            set_memory_word(&bus, load_address + index * 4u, 0x10000004u + index);
        }
        CortexM4 cpu = create_cpu(&bus);
        cpu.cpacr = 0x00f00000u;
        cpu.registers[6] = load_address;
        for (uint32_t index = 0u; index < 4u; index++) {
            cpu.fp_registers[4u + index] = 0xaaaa0000u + index;
        }
        expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xecb6), UINT16_C(0x2a04)),
               "VLDM encoding is recognized when a later read faults");
        bool load_state = cpu.registers[6] == load_address;
        for (uint32_t index = 0u; index < 4u; index++) {
            const uint32_t expected =
                index < failed - 1u ? 0x10000004u + index : 0xaaaa0000u + index;
            load_state = load_state && cpu.fp_registers[4u + index] == expected;
        }
        expect(state,
               load_state && cpu.bfar == load_address + (failed - 1u) * 4u &&
                   (cpu.cfsr & (1u << 9u)) != 0u,
               "VLDM preserves partial loads and suppresses faulting writeback");

        memset(&bus, 0, sizeof(bus));
        bus.fail_write = failed;
        const uint32_t store_address = MEMORY_BASE + 0x100u;
        cpu = create_cpu(&bus);
        cpu.cpacr = 0x00f00000u;
        cpu.registers[7] = store_address;
        for (uint32_t index = 0u; index < 4u; index++) {
            cpu.fp_registers[8u + index] = 0x20000008u + index;
        }
        expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xeca7), UINT16_C(0x4a04)),
               "VSTM encoding is recognized when a later write faults");
        bool store_state = cpu.registers[7] == store_address;
        for (uint32_t index = 0u; index < 4u; index++) {
            const uint32_t expected = index < failed - 1u ? 0x20000008u + index : 0u;
            store_state = store_state && memory_word(&bus, store_address + index * 4u) == expected;
        }
        expect(state,
               store_state && cpu.bfar == store_address + (failed - 1u) * 4u &&
                   (cpu.cfsr & (1u << 9u)) != 0u,
               "VSTM preserves partial stores and suppresses faulting writeback");
    }
}

static void test_exclusive_store_faults(TestState* state) {
    FaultBus bus = {0};
    bus.memory[0x100u] = 0x5au;
    CortexM4 cpu = create_cpu(&bus);
    cpu.registers[1] = MEMORY_BASE + 0x100u;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe8d1), UINT16_C(0x0f4f)),
           "LDREXB establishes a byte reservation");
    expect(state, cpu.exclusive_valid, "LDREXB reservation is active");

    bus.fail_write = 1u;
    cpu.registers[0] = 0xaaaaaaaau;
    cpu.registers[2] = 0xa5u;
    expect(state, cortex_m4_execute_thumb32(&cpu, UINT16_C(0xe8c1), UINT16_C(0x2f40)),
           "STREXB encoding is recognized when its write faults");
    expect(state, !cpu.exclusive_valid && cpu.registers[0] == 0xaaaaaaaau,
           "faulting STREXB consumes the reservation without writing status");
    expect(state, (cpu.cfsr & (1u << 9u)) != 0u && cpu.bfar == MEMORY_BASE + 0x100u,
           "faulting STREXB records the failed address");
}

static void test_lazy_fp_preservation_faults(TestState* state) {
    static const uint32_t failed_writes[] = {8u, 18u};
    for (size_t failure = 0u; failure < sizeof(failed_writes) / sizeof(failed_writes[0]);
         failure++) {
        FaultBus bus = {.fail_write = failed_writes[failure]};
        CortexM4 cpu = create_cpu(&bus);
        const uint32_t frame_address = MEMORY_BASE + 0x200u;
        cpu.cpacr = 0x00f00000u;
        cpu.fpccr = (1u << 31u) | (1u << 30u) | 1u;
        cpu.fpcar = frame_address;
        cpu.fpscr = 0xf5c00000u;
        cpu.exception_frame_depth = 1u;
        cpu.exception_frames[0].address = frame_address;
        cpu.exception_frames[0].return_value = 0xffffffe9u;
        cpu.exception_frames[0].extended = true;
        cpu.exception_frames[0].lazy = true;
        for (uint32_t index = 0u; index < 16u; index++) {
            cpu.fp_registers[index] = 0xa000u + index;
        }
        const CortexM4ExceptionFrame frame = cpu.exception_frames[0];
        const uint32_t stack_pointer = cpu.msp;

        expect(state, cortex_m4_execute_fpu(&cpu, UINT16_C(0xeeb0), UINT16_C(0x0a67)),
               "FP instruction recognizes a lazy-preservation write fault");
        for (uint32_t index = 0u; index < 16u; index++) {
            const uint32_t expected = index + 1u < failed_writes[failure] ? 0xa000u + index : 0u;
            expect(state, memory_word(&bus, frame_address + index * 4u) == expected,
                   "lazy FP fault preserves only completed register writes");
        }
        expect(state,
               memory_word(&bus, frame_address + 64u) ==
                   (failed_writes[failure] > 17u ? 0xf5c00000u : 0u),
               "lazy FP fault preserves the completed FPSCR write");
        expect(state,
               cpu.fp_registers[0] == 0xa000u && cpu.exception_frame_depth == 1u &&
                   cpu.msp == stack_pointer && cpu.fpcar == frame_address,
               "lazy FP fault prevents instruction and stack-state commits");
        expect(state,
               cpu.exception_frames[0].address == frame.address &&
                   cpu.exception_frames[0].return_value == frame.return_value &&
                   cpu.exception_frames[0].extended == frame.extended &&
                   cpu.exception_frames[0].lazy == frame.lazy && (cpu.fpccr & 1u) != 0u,
               "lazy FP fault retains lazy frame metadata");
        expect(state,
               (cpu.cfsr & (1u << 13u)) != 0u && (cpu.cfsr & (1u << 5u)) == 0u &&
                   (cpu.hfsr & (1u << 30u)) != 0u && (cpu.system_pending & (1u << 3u)) != 0u,
               "lazy FP bus fault records LSPERR and escalates");
    }

    FaultBus bus = {0};
    CortexM4 cpu = create_cpu(&bus);
    const uint32_t frame_address = MEMORY_BASE + 0x200u;
    cpu.cpacr = 0x00f00000u;
    cpu.fpccr = (1u << 31u) | (1u << 30u) | 1u;
    cpu.fpcar = frame_address;
    cpu.fpscr = 0xf5c00000u;
    cpu.exception_frame_depth = 1u;
    cpu.exception_frames[0].address = frame_address;
    cpu.exception_frames[0].return_value = 0xffffffe9u;
    cpu.exception_frames[0].extended = true;
    cpu.exception_frames[0].lazy = true;
    cpu.mpu_region_count = CORTEX_M4_MPU_REGION_COUNT;
    cpu.mpu_control = 5u;
    cpu.mpu_region_base[0] = frame_address + 0x20u;
    cpu.mpu_region_attributes[0] = (4u << 1u) | 1u;
    for (uint32_t index = 0u; index < 16u; index++) {
        cpu.fp_registers[index] = 0xa000u + index;
    }

    expect(state, cortex_m4_execute_fpu(&cpu, UINT16_C(0xeeb0), UINT16_C(0x0a67)),
           "FP instruction recognizes a late lazy-preservation MPU fault");
    bool partial_frame = true;
    for (uint32_t index = 0u; index < 16u; index++) {
        const uint32_t expected = index < 8u ? 0xa000u + index : 0u;
        partial_frame = partial_frame && memory_word(&bus, frame_address + index * 4u) == expected;
    }
    expect(state, partial_frame, "lazy FP MPU fault preserves completed writes");
    expect(state,
           (cpu.cfsr & (1u << 5u)) != 0u && (cpu.cfsr & (1u << 13u)) == 0u &&
               (cpu.cfsr & (1u << 1u)) != 0u && cpu.mmfar == frame_address + 0x20u,
           "lazy FP MPU fault records MLSPERR at the denied word");
    expect(state,
           cpu.fp_registers[0] == 0xa000u && cpu.exception_frames[0].lazy &&
               (cpu.fpccr & 1u) != 0u && (cpu.hfsr & (1u << 30u)) != 0u &&
               (cpu.system_pending & (1u << 3u)) != 0u,
           "lazy FP MPU fault preserves lazy state and escalates");
}

int main(void) {
    TestState state = {0u, 0u, 0u};
    test_push_pop_faults(&state);
    test_multiple_transfer_faults(&state);
    test_decrement_before_faults(&state);
    test_doubleword_faults(&state);
    test_floating_multiple_faults(&state);
    test_exclusive_store_faults(&state);
    test_lazy_fp_preservation_faults(&state);
    return test_finish(&state);
}
