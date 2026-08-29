#include "architecture/cortex_m4/internal.h"

bool cortex_m4_bus_read(CortexM4* cpu, uint32_t address, uint8_t byte_count, CortexM4Access access,
                        uint32_t* output_value) {
    if (output_value == NULL || (byte_count != 1 && byte_count != 2 && byte_count != 4)) {
        return false;
    }
    if (cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M &&
        access == CORTEX_M4_ACCESS_INSTRUCTION &&
        ((address >= 0x40000000u && address < 0x60000000u) || address >= 0xa0000000u))
        return false;
    cortex_m4_timing_access(cpu, address, byte_count, access, false);
    const CortexM4SystemAccess mpu_access =
        cortex_m4_mpu_read(cpu, address, byte_count, access, output_value);
    if (mpu_access == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED) {
        return true;
    }
    if (mpu_access == CORTEX_M4_SYSTEM_ACCESS_REJECTED) {
        return false;
    }
    if (!cortex_m4_mpu_check(cpu, address, byte_count, access, false)) {
        return false;
    }
    const CortexM4SystemAccess system_access =
        cortex_m4_system_read(cpu, address, byte_count, access, output_value);
    if (system_access == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED) {
        return true;
    }
    if (system_access == CORTEX_M4_SYSTEM_ACCESS_REJECTED) {
        return false;
    }
    return cpu->bus.read(cpu->bus.context, address, byte_count, access, output_value);
}

bool cortex_m4_bus_write(CortexM4* cpu, uint32_t address, uint8_t byte_count, CortexM4Access access,
                         uint32_t write_value) {
    if (byte_count != 1 && byte_count != 2 && byte_count != 4) {
        return false;
    }
    cortex_m4_timing_access(cpu, address, byte_count, access, true);
    const CortexM4SystemAccess mpu_access =
        cortex_m4_mpu_write(cpu, address, byte_count, access, write_value);
    if (mpu_access == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED) {
        cpu->exclusive_valid = false;
        cortex_m4_timing_observe_write(cpu, address, byte_count);
        return true;
    }
    if (mpu_access == CORTEX_M4_SYSTEM_ACCESS_REJECTED) {
        return false;
    }
    if (!cortex_m4_mpu_check(cpu, address, byte_count, access, true)) {
        return false;
    }
    const CortexM4SystemAccess system_access =
        cortex_m4_system_write(cpu, address, byte_count, access, write_value);
    if (system_access == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED) {
        cortex_m4_timing_observe_write(cpu, address, byte_count);
        return true;
    }
    if (system_access == CORTEX_M4_SYSTEM_ACCESS_REJECTED) {
        return false;
    }
    const bool written = cpu->bus.write(cpu->bus.context, address, byte_count, access, write_value);
    if (written) {
        cortex_m4_timing_observe_write(cpu, address, byte_count);
    }
    return written;
}

bool cortex_m4_data_read(CortexM4* cpu, uint32_t address, uint8_t byte_count, CortexM4Access access,
                         uint32_t* output_value) {
    if (byte_count > 1 && (address & (byte_count - 1u)) != 0 &&
        (cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M || (cpu->ccr & (1u << 3)) != 0)) {
        cpu->cfsr |= 1u << 24;
        cortex_m4_raise_fault(cpu, 6);
        return false;
    }
    if (!cortex_m4_mpu_check(cpu, address, byte_count, access, false)) {
        return false;
    }
    if (cortex_m4_bus_read(cpu, address, byte_count, access, output_value)) {
        cortex_m4_debug_memory_access(cpu, address, byte_count, false, *output_value);
        return true;
    }
    cpu->bfar = address;
    cpu->cfsr |= (1u << 15) | (1u << 9);
    cortex_m4_raise_fault(cpu, 5);
    return false;
}

bool cortex_m4_data_write(CortexM4* cpu, uint32_t address, uint8_t byte_count,
                          CortexM4Access access, uint32_t write_value) {
    if (byte_count > 1 && (address & (byte_count - 1u)) != 0 &&
        (cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M || (cpu->ccr & (1u << 3)) != 0)) {
        cpu->cfsr |= 1u << 24;
        cortex_m4_raise_fault(cpu, 6);
        return false;
    }
    if (!cortex_m4_mpu_check(cpu, address, byte_count, access, true)) {
        return false;
    }
    if (cortex_m4_bus_write(cpu, address, byte_count, access, write_value)) {
        cortex_m4_debug_memory_access(cpu, address, byte_count, true, write_value);
        return true;
    }
    cpu->bfar = address;
    cpu->cfsr |= (1u << 15) | (1u << 9);
    cortex_m4_raise_fault(cpu, 5);
    return false;
}

bool cortex_m4_require_alignment(CortexM4* cpu, uint32_t address, uint8_t alignment) {
    if ((address & (alignment - 1u)) == 0) {
        return true;
    }
    cpu->cfsr |= 1u << 24;
    cortex_m4_raise_fault(cpu, 6);
    return false;
}

void cortex_m4_advance(CortexM4* cpu, uint32_t cycles) {
    const bool sleeping = cpu->sleeping;
    for (uint32_t cycle_index = 0; cycle_index < cycles; cycle_index++) {
        if (sleeping && cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M)
            break;
        bool systick_clock = cpu->architecture != CORTEX_M4_ARCHITECTURE_ARMV6_M ||
                             (cpu->systick_control & (1u << 2)) != 0;
        if (!systick_clock && cpu->architecture == CORTEX_M4_ARCHITECTURE_ARMV6_M) {
            cpu->systick_external_phase = (uint8_t)((cpu->systick_external_phase + 1u) & 15u);
            systick_clock = cpu->systick_external_phase == 0;
        }
        if ((cpu->systick_control & 1u) != 0 && systick_clock) {
            if (cpu->systick_current == 0) {
                cpu->systick_current = cpu->systick_reload;
            } else {
                cpu->systick_current--;
                if (cpu->systick_current == 0) {
                    cpu->systick_control |= 1u << 16;
                    if ((cpu->systick_control & 2u) != 0) {
                        cortex_m4_system_set_pending(cpu, 15, true);
                        cpu->sleeping = false;
                    }
                }
            }
        }
    }

    cpu->cycles += cycles;

    cortex_m4_debug_advance(cpu, cycles, sleeping);

    if (cpu->bus.advance != NULL) {
        cpu->bus.advance(cpu->bus.context, cycles);
    }
}
