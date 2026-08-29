#include "device/kinetis/internal.h"

#define MTB_POSITION UINT32_C(0xf0000000)
#define MTB_MASTER UINT32_C(0xf0000004)
#define MTB_FLOW UINT32_C(0xf0000008)
#define MTB_BASE UINT32_C(0xf000000c)
#define MTBDWT_CTRL UINT32_C(0xf0001000)
#define MTBDWT_COMP0 UINT32_C(0xf0001020)
#define MTBDWT_MASK0 UINT32_C(0xf0001024)
#define MTBDWT_FCT0 UINT32_C(0xf0001028)
#define MTBDWT_COMP1 UINT32_C(0xf0001030)
#define MTBDWT_MASK1 UINT32_C(0xf0001034)
#define MTBDWT_FCT1 UINT32_C(0xf0001038)
#define MTBDWT_TBCTRL UINT32_C(0xf0001200)
#define MTB_RAM_BASE UINT32_C(0x1ffff000)
#define MTB_ENABLE (UINT32_C(1) << 31u)

enum {
    MTB_HALT_REQUEST = 1u << 9u,
    MTB_RAM_PRIVILEGED = 1u << 8u,
    MTB_SFR_WRITE_PRIVILEGED = 1u << 7u,
    MTB_TRACE_STOP_ENABLE = 1u << 6u,
    MTB_TRACE_START_ENABLE = 1u << 5u,
    MTB_AUTOSTOP = 1u,
    MTB_AUTOHALT = 2u,
    MTBDWT_MATCHED = 1u << 24u,
};

static uint32_t mtb_load(const Kinetis* device, uint32_t address) {
    return kinetis_internal_raw_load(device, address, 4u);
}

static void mtb_store(Kinetis* device, uint32_t address, uint32_t value) {
    kinetis_internal_raw_store(device, address, 4u, value);
}

static uint8_t mtb_mask(const Kinetis* device) {
    const uint8_t mask = (uint8_t)(mtb_load(device, MTB_MASTER) & 0x1fu);
    return mask > 11u ? 11u : mask;
}

static void mtb_set_enabled(Kinetis* device, bool enabled) {
    uint32_t master = mtb_load(device, MTB_MASTER);
    const bool was_enabled = (master & MTB_ENABLE) != 0u;
    master = enabled ? master | MTB_ENABLE : master & ~MTB_ENABLE;
    mtb_store(device, MTB_MASTER, master);
    if (enabled && !was_enabled) {
        device->mtb_first_packet = true;
        device->mtb_previous_valid = false;
    }
}

static void mtb_apply_watermark(Kinetis* device) {
    const uint32_t position = mtb_load(device, MTB_POSITION) & 0x7ff8u;
    const uint32_t flow = mtb_load(device, MTB_FLOW);
    if ((flow & 0x7ff8u) != position)
        return;
    uint32_t master = mtb_load(device, MTB_MASTER);
    if ((flow & MTB_AUTOHALT) != 0u) {
        master |= MTB_HALT_REQUEST;
        cortex_m4_debug_request_halt(device->cpu);
    }
    if ((flow & MTB_AUTOSTOP) != 0u) {
        master &= ~MTB_ENABLE;
        device->mtb_autostop_latched = true;
    }
    mtb_store(device, MTB_MASTER, master);
}

static uint32_t mtb_trace_address(uint32_t pointer) {
    return pointer >> 13u == 3u ? 0x1fff8000u + pointer : 0x20000000u + pointer;
}

static void mtb_write_word(Kinetis* device, uint32_t address, uint32_t value) {
    if (address < device->sram_base || address - device->sram_base >
                                            device->configuration.sram_size - sizeof(value))
        return;
    const uint32_t offset = address - device->sram_base;
    for (uint8_t index = 0u; index < sizeof(value); index++)
        device->sram[offset + index] = (uint8_t)(value >> (index * 8u));
    memset(device->sram_initialized + offset, 1, sizeof(value));
}

static void mtb_write_packet(Kinetis* device, uint32_t source, uint32_t destination,
                             bool exception) {
    if ((mtb_load(device, MTB_MASTER) & MTB_ENABLE) == 0u)
        return;
    uint32_t position = mtb_load(device, MTB_POSITION);
    const uint32_t pointer = position & 0x7ff8u;
    const uint32_t packet_address = mtb_trace_address(pointer);
    mtb_write_word(device, packet_address, (source & 0xfffffffeu) | exception);
    mtb_write_word(device, packet_address + 4u,
                   (destination & 0xfffffffeu) | device->mtb_first_packet);
    device->mtb_first_packet = false;
    const uint32_t buffer_size = 1u << (mtb_mask(device) + 4u);
    const uint32_t buffer_mask = buffer_size - 1u;
    uint32_t next = pointer + 8u;
    if ((next & buffer_mask) == 0u) {
        next = pointer & ~buffer_mask;
        position |= 4u;
    }
    position = (position & ~0x7ff8u) | (next & 0x7ff8u);
    mtb_store(device, MTB_POSITION, position);
    mtb_apply_watermark(device);
}

static uint32_t mtb_exception_return_address(const Kinetis* device, uint32_t fallback) {
    const uint32_t exception_return = cortex_m4_get_register(device->cpu, 14u);
    const uint32_t stack_pointer = (exception_return & 4u) != 0u ? device->cpu->psp : device->cpu->msp;
    const uint32_t address = stack_pointer + 24u;
    if (address < device->sram_base || address - device->sram_base >
                                            device->configuration.sram_size - sizeof(uint32_t))
        return fallback;
    const uint8_t* bytes = device->sram + address - device->sram_base;
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8u) | ((uint32_t)bytes[2] << 16u) |
           ((uint32_t)bytes[3] << 24u);
}

static bool mtbdwt_address_match(uint32_t address, uint32_t comparator, uint8_t mask,
                                 bool instruction) {
    if (mask > 24u)
        mask = 24u;
    const uint32_t ignored = mask == 0u ? 0u : (1u << mask) - 1u;
    const uint32_t instruction_bit = instruction ? 1u : 0u;
    return ((address ^ comparator) & ~ignored & ~instruction_bit) == 0u;
}

static bool mtbdwt_function_matches(uint32_t function, bool write) {
    const uint8_t selector = (uint8_t)(function & 0x0fu);
    return selector == 7u || selector == (write ? 6u : 5u);
}

static bool mtbdwt_value_matches(uint32_t comparator, uint32_t value, uint8_t size) {
    if (size == 1u)
        return comparator == (value & 0xffu) * 0x01010101u;
    if (size == 2u) {
        const uint32_t halfword = value & 0xffffu;
        return comparator == ((halfword << 16u) | halfword);
    }
    return comparator == value;
}

static bool mtbdwt_apply_matches(Kinetis* device, uint8_t matches) {
    const uint32_t master = mtb_load(device, MTB_MASTER);
    const uint32_t routing = mtb_load(device, MTBDWT_TBCTRL);
    bool starts = false;
    bool stops = false;
    for (uint8_t comparator = 0u; comparator < 2u; comparator++) {
        if ((matches & (1u << comparator)) == 0u)
            continue;
        const uint32_t function_address = comparator == 0u ? MTBDWT_FCT0 : MTBDWT_FCT1;
        mtb_store(device, function_address, mtb_load(device, function_address) | MTBDWT_MATCHED);
        if ((routing & (1u << comparator)) != 0u)
            starts = true;
        else
            stops = true;
    }
    if (starts && (master & MTB_TRACE_START_ENABLE) != 0u) {
        if (!device->mtb_autostop_latched)
            mtb_set_enabled(device, true);
        device->mtb_stop_pending = false;
        return true;
    }
    if (stops && (master & MTB_TRACE_STOP_ENABLE) != 0u)
        device->mtb_stop_pending = true;
    return false;
}

static bool mtbdwt_fetch(Kinetis* device, uint32_t address) {
    const uint32_t fct0 = mtb_load(device, MTBDWT_FCT0);
    const uint32_t fct1 = mtb_load(device, MTBDWT_FCT1);
    uint8_t matches = 0u;
    if ((fct0 & 0x0fu) == 4u &&
        mtbdwt_address_match(address, mtb_load(device, MTBDWT_COMP0),
                             (uint8_t)mtb_load(device, MTBDWT_MASK0), true))
        matches |= 1u;
    if ((fct1 & 0x0fu) == 4u &&
        mtbdwt_address_match(address, mtb_load(device, MTBDWT_COMP1),
                             (uint8_t)mtb_load(device, MTBDWT_MASK1), true))
        matches |= 2u;
    return mtbdwt_apply_matches(device, matches);
}

void kinetis_internal_mtb_trace(void* context, uint32_t address, uint32_t opcode, bool executed) {
    Kinetis* device = context;
    if (device == NULL || !executed)
        return;
    const bool stop_after_boundary = device->mtb_stop_pending;
    device->mtb_stop_pending = false;
    const bool started = mtbdwt_fetch(device, address);
    const uint16_t exception = (uint16_t)(cortex_m4_get_xpsr(device->cpu) & 0x1ffu);
    if (device->mtb_previous_valid) {
        const uint32_t sequential = device->mtb_previous_address +
                                    (device->mtb_previous_opcode > UINT16_MAX ? 4u : 2u);
        if (exception == 0u && device->mtb_previous_exception != 0u) {
            mtb_write_packet(device, device->mtb_previous_address, device->mtb_previous_lr,
                             false);
            mtb_write_packet(device, device->mtb_previous_lr, address, true);
        } else if (exception != device->mtb_previous_exception) {
            mtb_write_packet(device, mtb_exception_return_address(device, sequential), address,
                             true);
        } else if (address != sequential) {
            mtb_write_packet(device, device->mtb_previous_address, address, false);
        }
    }
    if (stop_after_boundary && !started)
        mtb_set_enabled(device, false);
    device->mtb_previous_address = address;
    device->mtb_previous_opcode = opcode;
    device->mtb_previous_lr = cortex_m4_get_register(device->cpu, 14u);
    device->mtb_previous_exception = exception;
    device->mtb_previous_valid = true;
}

void kinetis_internal_mtbdwt_data_access(Kinetis* device, uint32_t address, uint8_t byte_count,
                                         bool write, uint32_t value) {
    if (device == NULL || byte_count == 0u || byte_count > 4u)
        return;
    const uint32_t fct0 = mtb_load(device, MTBDWT_FCT0);
    const bool data_value = (fct0 & (1u << 8u)) != 0u;
    bool match0 = false;
    if (data_value && mtbdwt_function_matches(fct0, write)) {
        const uint8_t configured_size = (uint8_t)((fct0 >> 10u) & 3u);
        const uint8_t expected_size = configured_size == 0u ? 1u : configured_size == 1u ? 2u : 4u;
        match0 = byte_count == expected_size &&
                 mtbdwt_value_matches(mtb_load(device, MTBDWT_COMP0), value, expected_size);
        if (match0 && (fct0 & (1u << 12u)) != 0u)
            match0 = mtbdwt_address_match(address, mtb_load(device, MTBDWT_COMP1),
                                          (uint8_t)mtb_load(device, MTBDWT_MASK1), false);
    } else if (!data_value && mtbdwt_function_matches(fct0, write)) {
        match0 = mtbdwt_address_match(address, mtb_load(device, MTBDWT_COMP0),
                                      (uint8_t)mtb_load(device, MTBDWT_MASK0), false);
    }
    uint8_t matches = match0 ? 1u : 0u;
    const uint32_t fct1 = mtb_load(device, MTBDWT_FCT1);
    const bool linked = data_value && (fct0 & (1u << 12u)) != 0u;
    if (!linked && mtbdwt_function_matches(fct1, write) &&
        mtbdwt_address_match(address, mtb_load(device, MTBDWT_COMP1),
                             (uint8_t)mtb_load(device, MTBDWT_MASK1), false))
        matches |= 2u;
    mtbdwt_apply_matches(device, matches);
}

bool kinetis_internal_mtb_read(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                               uint8_t byte_count, uint32_t* output_value) {
    if (device == NULL || output_value == NULL || byte_count != 4u)
        return false;
    if (id == KINETIS_PERIPHERAL_MTB) {
        if (address == MTB_BASE) {
            *output_value = MTB_RAM_BASE;
            return true;
        }
        if (address == MTB_POSITION || address == MTB_MASTER || address == MTB_FLOW) {
            *output_value = mtb_load(device, address);
            return true;
        }
        return false;
    }
    if (id != KINETIS_PERIPHERAL_MTBDWT)
        return false;
    if (address == MTBDWT_CTRL) {
        *output_value = 0x2f000000u;
        return true;
    }
    if (address == MTBDWT_COMP0 || address == MTBDWT_MASK0 || address == MTBDWT_COMP1 ||
        address == MTBDWT_MASK1 || address == MTBDWT_TBCTRL) {
        *output_value = mtb_load(device, address);
        return true;
    }
    if (address == MTBDWT_FCT0 || address == MTBDWT_FCT1) {
        *output_value = mtb_load(device, address);
        mtb_store(device, address, *output_value & ~MTBDWT_MATCHED);
        return true;
    }
    return false;
}

static void mtb_write_master(Kinetis* device, uint32_t write_value) {
    const uint32_t previous = mtb_load(device, MTB_MASTER);
    uint32_t value = write_value & 0x800003ffu;
    if (device->mtb_autostop_latched)
        value &= ~MTB_ENABLE;
    if ((value & MTB_ENABLE) != 0u && (previous & MTB_ENABLE) == 0u) {
        device->mtb_first_packet = true;
        device->mtb_previous_valid = false;
    }
    mtb_store(device, MTB_MASTER, value);
    if ((value & MTB_HALT_REQUEST) != 0u)
        cortex_m4_debug_request_halt(device->cpu);
}

static void mtbdwt_write_function(Kinetis* device, uint32_t address, uint32_t write_value) {
    const uint32_t matched = mtb_load(device, address) & MTBDWT_MATCHED;
    if (address == MTBDWT_FCT0) {
        const uint32_t data_fields = write_value & ((1u << 12u) | (3u << 10u) | (1u << 8u));
        mtb_store(device, address, matched | data_fields | (write_value & 0x0fu));
    } else {
        mtb_store(device, address, matched | (write_value & 0x0fu));
    }
}

bool kinetis_internal_mtb_write(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                                uint8_t byte_count, uint32_t write_value) {
    if (device == NULL || byte_count != 4u)
        return false;
    if (id == KINETIS_PERIPHERAL_MTB) {
        if (address == MTB_POSITION) {
            const uint32_t previous_pointer = mtb_load(device, address) & 0x7ff8u;
            const uint32_t position = write_value & 0x7ffcu;
            mtb_store(device, address, position);
            const uint32_t buffer_mask = (1u << (mtb_mask(device) + 4u)) - 1u;
            if ((position & 0x7ff8u) != previous_pointer &&
                ((position & 0x7ff8u) & buffer_mask) == 0u)
                device->mtb_autostop_latched = false;
            return true;
        }
        if (address == MTB_MASTER) {
            mtb_write_master(device, write_value);
            return true;
        }
        if (address == MTB_FLOW) {
            const uint32_t previous = mtb_load(device, address);
            mtb_store(device, address, write_value & 0x7ffbu);
            if ((write_value & MTB_AUTOSTOP) == 0u ||
                ((previous ^ write_value) & 0x7ff8u) != 0u)
                device->mtb_autostop_latched = false;
            return true;
        }
        return address == MTB_BASE;
    }
    if (id != KINETIS_PERIPHERAL_MTBDWT)
        return false;
    if (address == MTBDWT_COMP0 || address == MTBDWT_COMP1) {
        mtb_store(device, address, write_value);
        return true;
    }
    if (address == MTBDWT_MASK0 || address == MTBDWT_MASK1) {
        mtb_store(device, address, (write_value & 0x1fu) > 24u ? 24u : write_value & 0x1fu);
        return true;
    }
    if (address == MTBDWT_FCT0 || address == MTBDWT_FCT1) {
        mtbdwt_write_function(device, address, write_value);
        return true;
    }
    if (address == MTBDWT_TBCTRL) {
        mtb_store(device, address, 0x20000000u | (write_value & 3u));
        return true;
    }
    return address == MTBDWT_CTRL;
}

bool kinetis_internal_mtb_unprivileged_ram_blocked(const Kinetis* device, uint32_t address,
                                                   uint8_t byte_count) {
    if (device == NULL || byte_count == 0u ||
        (mtb_load(device, MTB_MASTER) & MTB_RAM_PRIVILEGED) == 0u)
        return false;
    return address >= device->sram_base &&
           (uint64_t)address + byte_count <=
               (uint64_t)device->sram_base + device->configuration.sram_size;
}
