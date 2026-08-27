#include "kinetis.h"

#include <stdlib.h>
#include <string.h>

#include "device/kinetis/internal.h"

static bool valid_memory_range(uint32_t address, uint8_t access_size, uint32_t region_base,
                               size_t region_size) {
    if (access_size == 0u || address < region_base)
        return false;
    return (uint64_t)address + access_size <= (uint64_t)region_base + region_size;
}

static uint32_t read_little_endian(const uint8_t* input_bytes, uint8_t byte_count) {
    uint32_t output_value = 0u;
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
        output_value |= (uint32_t)input_bytes[byte_index] << (byte_index * 8u);
    }
    return output_value;
}

static void write_little_endian(uint8_t* output_bytes, uint8_t byte_count, uint32_t write_value) {
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
        output_bytes[byte_index] = (uint8_t)(write_value >> (byte_index * 8u));
    }
}

static bool is_flash_access_allowed(const Kinetis* device, CortexM4Access access, uint8_t master,
                                    bool write_access) {
    if (access == CORTEX_M4_ACCESS_DEBUG) {
        return true;
    }
    const uint32_t offset = 0x4001f000u - KINETIS_PERIPHERAL_BASE;
    const uint32_t protection = read_little_endian(device->peripheral + offset, 4u);
    const uint8_t permission = (uint8_t)((protection >> (master * 2u)) & 3u);
    return write_access ? (permission & 2u) != 0u : (permission & 1u) != 0u;
}

static uint32_t fmc_raw_load(const Kinetis* device, uint32_t address) {
    return read_little_endian(device->peripheral + address - KINETIS_PERIPHERAL_BASE, 4u);
}

static void fmc_raw_store(Kinetis* device, uint32_t address, uint32_t write_value) {
    write_little_endian(device->peripheral + address - KINETIS_PERIPHERAL_BASE, 4u, write_value);
}

static uint32_t fmc_tag_address(const Kinetis* device, uint8_t cache_way, uint8_t cache_set) {
    return 0x4001f100u + ((uint32_t)cache_way * device->profile->fmc_set_count + cache_set) * 4u;
}

static uint32_t fmc_data_address(const Kinetis* device, uint8_t cache_way, uint8_t cache_set,
                                 uint8_t word_index) {
    const uint8_t word_count = device->profile->fmc_line_size / 4u;
    return 0x4001f200u +
           ((uint32_t)cache_way * device->profile->fmc_set_count + cache_set) *
               device->profile->fmc_line_size +
           (word_count - 1u - word_index) * 4u;
}

static uint32_t fmc_tag_value(const Kinetis* device, uint32_t cache_line_address);

static void fmc_clear_entry(Kinetis* device, uint8_t cache_way, uint8_t cache_set) {
    fmc_raw_store(device, fmc_tag_address(device, cache_way, cache_set), 0u);
    const uint8_t word_count = device->profile->fmc_line_size / 4u;
    for (uint8_t word_index = 0u; word_index < word_count; word_index++) {
        fmc_raw_store(device, fmc_data_address(device, cache_way, cache_set, word_index), 0u);
    }
    device->fmc_bank[cache_way][cache_set] = 0u;
    device->fmc_age[cache_way][cache_set] = 0u;
}

static void fmc_clear_cache(Kinetis* device) {
    for (uint8_t cache_way = 0u; cache_way < KINETIS_FMC_WAY_COUNT; cache_way++) {
        for (uint8_t cache_set = 0u; cache_set < device->profile->fmc_set_count; cache_set++) {
            fmc_clear_entry(device, cache_way, cache_set);
        }
    }
    device->fmc_access_count = 0u;
}

static void fmc_invalidate_line(Kinetis* device, uint8_t flash_bank, uint32_t flash_offset) {
    const uint8_t line_size = device->profile->fmc_line_size;
    const uint8_t cache_set =
        (uint8_t)((flash_offset / line_size) % device->profile->fmc_set_count);
    const uint32_t tag = fmc_tag_value(device, flash_offset & ~(line_size - 1u));
    for (uint8_t cache_way = 0u; cache_way < KINETIS_FMC_WAY_COUNT; cache_way++) {
        if (device->fmc_bank[cache_way][cache_set] == flash_bank &&
            fmc_raw_load(device, fmc_tag_address(device, cache_way, cache_set)) == tag) {
            fmc_clear_entry(device, cache_way, cache_set);
        }
    }
}

static uint32_t fmc_tag_value(const Kinetis* device, uint32_t cache_line_address) {
    const KinetisRegisterDescriptor* descriptor =
        kinetis_register_manifest_lookup(device->profile->id, 0x4001f100u, 32u);
    return (cache_line_address & descriptor->write_mask & ~1u) | 1u;
}

static bool fmc_way_allowed(uint8_t replacement_mode, bool instruction_access, uint8_t cache_way) {
    if (replacement_mode == 2u)
        return instruction_access ? cache_way < 2u : cache_way >= 2u;
    if (replacement_mode == 3u)
        return instruction_access ? cache_way < 3u : cache_way == 3u;
    return true;
}

static uint8_t fmc_replacement_way(Kinetis* device, uint8_t cache_set, bool instruction_access) {
    const uint32_t cache_control = fmc_raw_load(device, 0x4001f004u);
    const uint8_t locked_way_mask = (uint8_t)((cache_control >> 24u) & 0x0fu);
    const uint8_t replacement_mode = (uint8_t)((cache_control >> 5u) & 7u);
    for (uint8_t cache_way = 0u; cache_way < KINETIS_FMC_WAY_COUNT; cache_way++) {
        if ((locked_way_mask & (1u << cache_way)) == 0u &&
            fmc_way_allowed(replacement_mode, instruction_access, cache_way) &&
            (fmc_raw_load(device, fmc_tag_address(device, cache_way, cache_set)) & 1u) == 0u) {
            return cache_way;
        }
    }
    uint8_t selected_cache_way = UINT8_MAX;
    uint64_t oldest_age = UINT64_MAX;
    for (uint8_t cache_way = 0u; cache_way < KINETIS_FMC_WAY_COUNT; cache_way++) {
        if ((locked_way_mask & (1u << cache_way)) == 0u &&
            fmc_way_allowed(replacement_mode, instruction_access, cache_way) &&
            device->fmc_age[cache_way][cache_set] < oldest_age) {
            selected_cache_way = cache_way;
            oldest_age = device->fmc_age[cache_way][cache_set];
        }
    }
    return selected_cache_way;
}

static uint8_t fmc_source_byte(Kinetis* device, uint8_t flash_bank, uint32_t flash_offset) {
    if (flash_bank == 0u)
        return flash_offset < device->configuration.flash_size
                   ? device->flash[kinetis_data_program_flash_address(device->data, flash_offset)]
                   : UINT8_MAX;
    uint32_t output_value = UINT32_MAX;
    return kinetis_data_read(device->data, device->profile->flexnvm_address + flash_offset, 1u,
                             &output_value)
               ? (uint8_t)output_value
               : UINT8_MAX;
}

static uint8_t fmc_cached_byte(Kinetis* device, uint8_t flash_bank, uint32_t flash_offset,
                               CortexM4Access access) {
    const uint8_t line_size = device->profile->fmc_line_size;
    const uint8_t word_count = line_size / 4u;
    const uint32_t cache_line_address = flash_offset & ~(line_size - 1u);
    const uint8_t cache_set =
        (uint8_t)((flash_offset / line_size) % device->profile->fmc_set_count);
    const uint32_t tag = fmc_tag_value(device, cache_line_address);
    uint8_t cache_way = UINT8_MAX;
    for (uint8_t candidate_way = 0u; candidate_way < KINETIS_FMC_WAY_COUNT; candidate_way++) {
        if (device->fmc_bank[candidate_way][cache_set] == flash_bank &&
            fmc_raw_load(device, fmc_tag_address(device, candidate_way, cache_set)) == tag) {
            cache_way = candidate_way;
            break;
        }
    }
    if (cache_way == UINT8_MAX) {
        cache_way = fmc_replacement_way(device, cache_set, access == CORTEX_M4_ACCESS_INSTRUCTION);
        if (cache_way == UINT8_MAX)
            return fmc_source_byte(device, flash_bank, flash_offset);
        for (uint8_t word_index = 0u; word_index < word_count; word_index++) {
            uint32_t cache_word = 0u;
            for (uint8_t byte_index = 0u; byte_index < 4u; byte_index++) {
                const uint32_t source_address =
                    cache_line_address + (uint32_t)word_index * 4u + byte_index;
                cache_word |= (uint32_t)fmc_source_byte(device, flash_bank, source_address)
                              << (byte_index * 8u);
            }
            fmc_raw_store(device, fmc_data_address(device, cache_way, cache_set, word_index),
                          cache_word);
        }
        fmc_raw_store(device, fmc_tag_address(device, cache_way, cache_set), tag);
        device->fmc_bank[cache_way][cache_set] = flash_bank;
    }
    device->fmc_age[cache_way][cache_set] = ++device->fmc_access_count;
    const uint8_t word_index = (uint8_t)((flash_offset >> 2u) & (word_count - 1u));
    const uint32_t cache_word =
        fmc_raw_load(device, fmc_data_address(device, cache_way, cache_set, word_index));
    return (uint8_t)(cache_word >> ((flash_offset & 3u) * 8u));
}

static bool fmc_cache_enabled(const Kinetis* device, uint8_t bank, CortexM4Access access) {
    if (access == CORTEX_M4_ACCESS_DEBUG)
        return false;
    const uint32_t cache_control = fmc_raw_load(device, 0x4001f004u + (uint32_t)bank * 4u);
    return (cache_control & (access == CORTEX_M4_ACCESS_INSTRUCTION ? 8u : 16u)) != 0u;
}

static bool sysmpu_access_allowed(Kinetis* device, uint32_t address, uint8_t access_size,
                                  uint8_t master, bool supervisor, KinetisSysMpuAccess access) {
    if (!kinetis_profile_has_peripheral(device->profile, KINETIS_PERIPHERAL_SYSMPU) ||
        !kinetis_io_clock_enabled(&device->io, KINETIS_PERIPHERAL_SYSMPU))
        return true;
    if (!kinetis_io_sysmpu_access(&device->io, address, master, supervisor, access))
        return false;
    return kinetis_io_sysmpu_access(&device->io, address + access_size - 1u, master, supervisor,
                                    access);
}

static bool memory_read_unprotected(Kinetis* device, uint32_t address, uint8_t access_size,
                                    CortexM4Access access, uint8_t flash_master_id,
                                    uint32_t* output_value) {
    if (valid_memory_range(address, access_size, KINETIS_FLASH_BASE,
                           device->configuration.flash_size)) {
        if (!is_flash_access_allowed(device, access, flash_master_id, false)) {
            return false;
        }
        if (!kinetis_data_flash_read(device->data, false, address, access_size)) {
            *output_value = access_size == 4u ? UINT32_MAX : (1u << (access_size * 8u)) - 1u;
            return true;
        }
        if (fmc_cache_enabled(device, 0u, access)) {
            *output_value = 0u;
            for (uint8_t byte_index = 0u; byte_index < access_size; byte_index++) {
                *output_value |= (uint32_t)fmc_cached_byte(device, 0u, address + byte_index, access)
                                 << (byte_index * 8u);
            }
        } else {
            *output_value = 0u;
            for (uint8_t byte_index = 0u; byte_index < access_size; byte_index++) {
                *output_value |= (uint32_t)device->flash[kinetis_data_program_flash_address(
                                     device->data, address + byte_index)]
                                 << (byte_index * 8u);
            }
        }
        return true;
    }
    if (valid_memory_range(address, access_size, device->sram_base,
                           device->configuration.sram_size)) {
        const uint32_t offset = address - device->sram_base;
        if (access != CORTEX_M4_ACCESS_DEBUG) {
            for (uint8_t byte_index = 0u; byte_index < access_size; byte_index++) {
                if (device->sram_initialized[offset + byte_index] == 0u) {
                    if (device->uninitialized_sram_read_count == 0u) {
                        device->first_uninitialized_sram_read = address + byte_index;
                    }
                    device->uninitialized_sram_read_count++;
                }
            }
        }
        *output_value = read_little_endian(device->sram + address - device->sram_base, access_size);
        return true;
    }
    if (device->profile->flexnvm_size != 0u &&
        valid_memory_range(address, access_size, device->profile->flexnvm_address,
                           device->profile->flexnvm_size)) {
        if (!is_flash_access_allowed(device, access, flash_master_id, false))
            return false;
        const uint32_t offset = address - device->profile->flexnvm_address;
        if (!kinetis_data_flash_read(device->data, true, offset, access_size)) {
            *output_value = access_size == 4u ? UINT32_MAX : (1u << (access_size * 8u)) - 1u;
            return true;
        }
        if (fmc_cache_enabled(device, 1u, access)) {
            *output_value = 0u;
            for (uint8_t byte_index = 0u; byte_index < access_size; byte_index++) {
                *output_value |= (uint32_t)fmc_cached_byte(device, 1u, offset + byte_index, access)
                                 << (byte_index * 8u);
            }
            return true;
        }
        return kinetis_data_read(device->data, address, access_size, output_value);
    }
    if (device->profile->flexram_size != 0u &&
        valid_memory_range(address, access_size, device->profile->flexram_address,
                           device->profile->flexram_size)) {
        if (!is_flash_access_allowed(device, access, flash_master_id, false))
            return false;
        return kinetis_data_read(device->data, address, access_size, output_value);
    }
    if (device->flexbus_memory != NULL &&
        valid_memory_range(address, access_size, device->flexbus_address, device->flexbus_size) &&
        kinetis_io_flexbus_transfer(&device->io, address, access_size, false, 0u)) {
        *output_value = read_little_endian(
            device->flexbus_memory + address - device->flexbus_address, access_size);
        return true;
    }
    return kinetis_peripheral_read(device, address, access_size, access, output_value);
}

static bool memory_write_unprotected(Kinetis* device, uint32_t address, uint8_t access_size,
                                     CortexM4Access access, uint8_t flash_master_id,
                                     uint32_t write_value) {
    if (valid_memory_range(address, access_size, KINETIS_FLASH_BASE,
                           device->configuration.flash_size)) {
        if (access != CORTEX_M4_ACCESS_DEBUG) {
            return false;
        }
        for (uint8_t byte_index = 0u; byte_index < access_size; byte_index++) {
            device->flash[kinetis_data_program_flash_address(device->data, address + byte_index)] =
                (uint8_t)(write_value >> (byte_index * 8u));
        }
        fmc_invalidate_line(device, 0u, address);
        fmc_invalidate_line(device, 0u, address + access_size - 1u);
        return true;
    }
    if (valid_memory_range(address, access_size, device->sram_base,
                           device->configuration.sram_size)) {
        const uint32_t offset = address - device->sram_base;
        write_little_endian(device->sram + offset, access_size, write_value);
        memset(device->sram_initialized + offset, 1, access_size);
        return true;
    }
    if (device->profile->flexnvm_size != 0u &&
        valid_memory_range(address, access_size, device->profile->flexnvm_address,
                           device->profile->flexnvm_size)) {
        if (access != CORTEX_M4_ACCESS_DEBUG ||
            !kinetis_data_write(device->data, address, access_size, write_value))
            return false;
        const uint32_t offset = address - device->profile->flexnvm_address;
        fmc_invalidate_line(device, 1u, offset);
        fmc_invalidate_line(device, 1u, offset + access_size - 1u);
        return true;
    }
    if (device->profile->flexram_size != 0u &&
        valid_memory_range(address, access_size, device->profile->flexram_address,
                           device->profile->flexram_size)) {
        if (!is_flash_access_allowed(device, access, flash_master_id, true))
            return false;
        return kinetis_data_write(device->data, address, access_size, write_value);
    }
    if (device->flexbus_memory != NULL && !device->flexbus_read_only &&
        valid_memory_range(address, access_size, device->flexbus_address, device->flexbus_size) &&
        kinetis_io_flexbus_transfer(&device->io, address, access_size, true, write_value)) {
        write_little_endian(device->flexbus_memory + address - device->flexbus_address, access_size,
                            write_value);
        return true;
    }
    return kinetis_peripheral_write(device, address, access_size, access, write_value);
}

bool kinetis_memory_read(Kinetis* device, uint32_t address, uint8_t access_size,
                         CortexM4Access access, uint32_t* output_value) {
    if (device == NULL || output_value == NULL ||
        (access_size != 1u && access_size != 2u && access_size != 4u))
        return false;
    if (access != CORTEX_M4_ACCESS_DEBUG) {
        const uint8_t master_id = access == CORTEX_M4_ACCESS_INSTRUCTION ? 0u : 1u;
        const bool supervisor = access != CORTEX_M4_ACCESS_UNPRIVILEGED_DATA &&
                                (access != CORTEX_M4_ACCESS_INSTRUCTION || device->cpu == NULL ||
                                 (cortex_m4_get_xpsr(device->cpu) & 0x1ffu) != 0u ||
                                 (cortex_m4_get_control(device->cpu) & 1u) == 0u);
        const KinetisSysMpuAccess operation =
            access == CORTEX_M4_ACCESS_INSTRUCTION ? KINETIS_SYSMPU_EXECUTE : KINETIS_SYSMPU_READ;
        if (!sysmpu_access_allowed(device, address, access_size, master_id, supervisor, operation))
            return false;
    }
    const uint8_t master_id = access == CORTEX_M4_ACCESS_INSTRUCTION ? 0u : 1u;
    return memory_read_unprotected(device, address, access_size, access, master_id, output_value);
}

bool kinetis_memory_write(Kinetis* device, uint32_t address, uint8_t access_size,
                          CortexM4Access access, uint32_t write_value) {
    if (device == NULL || (access_size != 1u && access_size != 2u && access_size != 4u))
        return false;
    if (access != CORTEX_M4_ACCESS_DEBUG) {
        const bool supervisor = access != CORTEX_M4_ACCESS_UNPRIVILEGED_DATA;
        if (!sysmpu_access_allowed(device, address, access_size, 1u, supervisor,
                                   KINETIS_SYSMPU_WRITE))
            return false;
    }
    return memory_write_unprotected(device, address, access_size, access, 1u, write_value);
}

bool kinetis_dma_read(Kinetis* device, uint32_t address, uint8_t access_size,
                      uint32_t* output_value) {
    if (device == NULL || output_value == NULL ||
        (access_size != 1u && access_size != 2u && access_size != 4u) ||
        !sysmpu_access_allowed(device, address, access_size, 2u, true, KINETIS_SYSMPU_READ))
        return false;
    return memory_read_unprotected(device, address, access_size, CORTEX_M4_ACCESS_DATA, 2u,
                                   output_value);
}

bool kinetis_dma_write(Kinetis* device, uint32_t address, uint8_t access_size,
                       uint32_t write_value) {
    if (device == NULL || (access_size != 1u && access_size != 2u && access_size != 4u) ||
        !sysmpu_access_allowed(device, address, access_size, 2u, true, KINETIS_SYSMPU_WRITE))
        return false;
    return memory_write_unprotected(device, address, access_size, CORTEX_M4_ACCESS_DATA, 2u,
                                    write_value);
}

bool kinetis_flash_controller_write(Kinetis* device, uint32_t address, uint8_t access_size,
                                    uint32_t write_value) {
    if (device == NULL || (access_size != 1u && access_size != 2u && access_size != 4u) ||
        !valid_memory_range(address, access_size, KINETIS_FLASH_BASE,
                            device->configuration.flash_size))
        return false;
    write_little_endian(device->flash + address, access_size, write_value);
    return true;
}

static const KinetisRegisterDescriptor* bit_band_descriptor(const Kinetis* device,
                                                            uint32_t register_byte_address) {
    for (size_t register_index = 0; register_index < device->manifest->register_count;
         register_index++) {
        const KinetisRegisterDescriptor* descriptor = &device->manifest->registers[register_index];
        const uint8_t register_size = (uint8_t)(descriptor->width / 8u);
        if (register_byte_address >= descriptor->address &&
            register_byte_address - descriptor->address < register_size) {
            return descriptor;
        }
    }
    return NULL;
}

static bool bit_band_read(Kinetis* device, uint32_t address, CortexM4Access access,
                          uint32_t* output_value) {
    if ((address & 3u) != 0) {
        return false;
    }
    const uint32_t alias_address = address - KINETIS_BIT_BAND_BASE;
    const uint32_t register_byte_address = KINETIS_PERIPHERAL_BASE + alias_address / 32u;
    const uint8_t bit_index = (uint8_t)((alias_address / 4u) & 7u);
    const KinetisRegisterDescriptor* descriptor =
        bit_band_descriptor(device, register_byte_address);
    if (descriptor == NULL || (descriptor->access & KINETIS_REGISTER_ACCESS_READ) == 0) {
        return false;
    }
    const uint8_t register_bit =
        (uint8_t)((register_byte_address - descriptor->address) * 8u + bit_index);
    const uint32_t bit_mask = 1u << register_bit;
    if ((descriptor->implemented_mask & descriptor->read_mask & bit_mask) == 0) {
        return false;
    }
    uint32_t register_value = 0u;
    if (!kinetis_peripheral_read(device, descriptor->address, (uint8_t)(descriptor->width / 8u),
                                 access, &register_value)) {
        return false;
    }
    *output_value = (register_value & bit_mask) != 0;
    return true;
}

static bool bit_band_write(Kinetis* device, uint32_t address, CortexM4Access access,
                           uint32_t write_value) {
    if ((address & 3u) != 0) {
        return false;
    }
    const uint32_t alias_address = address - KINETIS_BIT_BAND_BASE;
    const uint32_t register_byte_address = KINETIS_PERIPHERAL_BASE + alias_address / 32u;
    const uint8_t bit_index = (uint8_t)((alias_address / 4u) & 7u);
    const KinetisRegisterDescriptor* descriptor =
        bit_band_descriptor(device, register_byte_address);
    if (descriptor == NULL || (descriptor->access & KINETIS_REGISTER_ACCESS_WRITE) == 0) {
        return false;
    }
    const uint8_t register_bit =
        (uint8_t)((register_byte_address - descriptor->address) * 8u + bit_index);
    const uint32_t bit_mask = 1u << register_bit;
    if ((descriptor->implemented_mask & descriptor->write_mask & bit_mask) == 0) {
        return false;
    }
    uint32_t register_value = 0u;
    if (!kinetis_peripheral_read(device, descriptor->address, (uint8_t)(descriptor->width / 8u),
                                 access, &register_value)) {
        return false;
    }
    register_value &= ~descriptor->w1c_mask;
    if ((descriptor->w1c_mask & bit_mask) != 0) {
        if ((write_value & 1u) != 0) {
            register_value |= bit_mask;
        }
    } else {
        register_value =
            (write_value & 1u) != 0 ? register_value | bit_mask : register_value & ~bit_mask;
    }
    return kinetis_peripheral_write(device, descriptor->address, (uint8_t)(descriptor->width / 8u),
                                    access, register_value);
}

static bool kinetis_read_bus(void* context, uint32_t address, uint8_t access_size,
                             CortexM4Access access, uint32_t* output_value) {
    Kinetis* device = context;
    if (address >= KINETIS_BIT_BAND_BASE &&
        address < KINETIS_BIT_BAND_BASE + KINETIS_BIT_BAND_SIZE) {
        if (access_size != 1u && access_size != 2u && access_size != 4u)
            return false;
        return bit_band_read(device, address, access, output_value);
    }
    return kinetis_memory_read(device, address, access_size, access, output_value);
}

static bool kinetis_write_bus(void* context, uint32_t address, uint8_t access_size,
                              CortexM4Access access, uint32_t write_value) {
    Kinetis* device = context;
    if (address >= KINETIS_BIT_BAND_BASE &&
        address < KINETIS_BIT_BAND_BASE + KINETIS_BIT_BAND_SIZE) {
        if (access_size != 1u && access_size != 2u && access_size != 4u)
            return false;
        return bit_band_write(device, address, access, write_value);
    }
    return kinetis_memory_write(device, address, access_size, access, write_value);
}

static void kinetis_advance_bus(void* context, uint32_t cycle_count) {
    Kinetis* device = context;
    device->cycles += cycle_count;
    kinetis_peripheral_advance(device, cycle_count);
}

static void kinetis_reset_bus(void* context) { kinetis_warm_reset(context, 0, 0x04u); }

static bool kinetis_core_clock_running(void* context) {
    Kinetis* device = context;
    kinetis_timing_set_cpu_sleeping(&device->timing, device->cpu->sleeping,
                                    (device->cpu->scr & 4u) != 0u);
    return device->timing.core_clock_hz != 0u;
}

KinetisConfiguration kinetis_configuration(KinetisProfile profile_id) {
    KinetisConfiguration configuration;
    const KinetisDeviceProfile* profile = kinetis_profile_get(profile_id);
    memset(&configuration, 0, sizeof(configuration));
    configuration.profile = profile_id;
    configuration.package = KINETIS_PACKAGE_DEFAULT;
    if (profile != NULL) {
        configuration.flash_size = profile->program_flash_size;
        configuration.sram_size = profile->sram_lower_size + profile->sram_upper_size;
    }
    configuration.external_oscillator_hz = 8000000u;
    configuration.rtc_oscillator_hz = 32768u;
    return configuration;
}

bool kinetis_profile_from_name(const char* name, KinetisProfile* profile) {
    const KinetisDeviceProfile* match = kinetis_profile_find(name);
    if (match == NULL || profile == NULL) {
        return false;
    }
    *profile = match->id;
    return true;
}

const char* kinetis_profile_name(KinetisProfile profile) {
    const KinetisDeviceProfile* match = kinetis_profile_get(profile);
    return match == NULL ? NULL : match->name;
}

bool kinetis_package_from_code(const char* code, KinetisPackage* package) {
    const KinetisPackageDescription* match = kinetis_package_find(code);
    if (match == NULL || package == NULL) {
        return false;
    }
    *package = (KinetisPackage)match->id;
    return true;
}

const char* kinetis_package_code(KinetisPackage package) {
    if (package == KINETIS_PACKAGE_DEFAULT) {
        return NULL;
    }
    const KinetisPackageDescription* match = kinetis_package_get((KinetisPackage)package);
    return match == NULL ? NULL : match->code;
}

static void destroy_partial_device(Kinetis* device) {
    if (device == NULL) {
        return;
    }
    cortex_m4_destroy(device->cpu);
    kinetis_data_destroy(device->data);
    kinetis_sdhc_destroy(&device->sdhc);
    free(device->flexbus_memory);
    free(device->flash);
    free(device->sram);
    free(device->sram_initialized);
    free(device->peripheral);
    free(device);
}

Kinetis* kinetis_create(KinetisConfiguration configuration) {
    const KinetisDeviceProfile* profile = kinetis_profile_get(configuration.profile);
    const KinetisRegisterManifest* manifest = kinetis_register_manifest_get(configuration.profile);
    if (profile == NULL || manifest == NULL) {
        return NULL;
    }
    const size_t profile_sram_size = (size_t)profile->sram_lower_size + profile->sram_upper_size;
    const KinetisPackageSelection* package_selection =
        configuration.package == KINETIS_PACKAGE_DEFAULT
            ? kinetis_package_default(profile)
            : kinetis_package_select(profile, (KinetisPackage)configuration.package);
    if (package_selection == NULL || configuration.flash_size == 0 ||
        configuration.flash_size > profile->program_flash_size || configuration.sram_size == 0 ||
        configuration.sram_size > profile_sram_size) {
        return NULL;
    }
    Kinetis* device = calloc(1, sizeof(*device));
    if (device == NULL) {
        return NULL;
    }
    device->configuration = configuration;
    device->first_uninitialized_sram_read = UINT32_MAX;
    device->profile = profile;
    device->package = package_selection;
    device->manifest = manifest;
    device->sram_base = configuration.sram_size == profile_sram_size
                            ? profile->sram_lower_address
                            : KINETIS_SRAM_CENTER - (uint32_t)(configuration.sram_size / 2u);
    device->flash = malloc(configuration.flash_size);
    device->sram = calloc(1, configuration.sram_size);
    device->sram_initialized = calloc(1, configuration.sram_size);
    device->peripheral = calloc(1, KINETIS_PERIPHERAL_SIZE);
    if (device->flash == NULL || device->sram == NULL || device->sram_initialized == NULL ||
        device->peripheral == NULL) {
        destroy_partial_device(device);
        return NULL;
    }
    memset(device->flash, 0xff, configuration.flash_size);
    if (!kinetis_serial_init(&device->serial, profile) ||
        !kinetis_sdhc_init(&device->sdhc, kinetis_sdhc_bus(device)) ||
        !kinetis_timing_init(&device->timing, profile, configuration.external_oscillator_hz,
                             configuration.rtc_oscillator_hz, kinetis_timing_signals(device)) ||
        !kinetis_io_init(&device->io, kinetis_io_configuration(device))) {
        destroy_partial_device(device);
        return NULL;
    }
    device->timing.sim_sdid_pin_id = kinetis_package_pin_id(package_selection);
    device->data = kinetis_data_create(profile, kinetis_data_bus(device));
    if (device->data == NULL) {
        destroy_partial_device(device);
        return NULL;
    }
    kinetis_sync_clock_gates(device);
    CortexM4Bus bus = {device, kinetis_read_bus, kinetis_write_bus, kinetis_advance_bus,
                       kinetis_reset_bus};
    device->cpu = cortex_m4_create(bus);
    if (device->cpu == NULL) {
        destroy_partial_device(device);
        return NULL;
    }
    cortex_m4_set_clock(device->cpu, kinetis_core_clock_running, device);
    if (!cortex_m4_configure_implementation(device->cpu, profile->cpu.external_irq_count,
                                            profile->cpu.nvic_priority_bits,
                                            profile->cpu.has_mpu ? 8u : 0u)) {
        destroy_partial_device(device);
        return NULL;
    }
    return device;
}

void kinetis_destroy(Kinetis* device) { destroy_partial_device(device); }

CortexM4* kinetis_cpu(Kinetis* device) { return device == NULL ? NULL : device->cpu; }

const CortexM4* kinetis_cpu_const(const Kinetis* device) {
    return device == NULL ? NULL : device->cpu;
}

static void sync_flash_configuration(Kinetis* device) {
    KinetisPeripheralBlock block;
    if (!kinetis_profile_peripheral_block(device->profile, KINETIS_PERIPHERAL_FLASH_CONFIG,
                                          &block) ||
        block.address + block.size > device->configuration.flash_size) {
        return;
    }
    (void)kinetis_data_set_flash_configuration(device->data, device->flash + block.address,
                                               block.size);
    memcpy(device->io.configuration.flash_configuration, device->flash + block.address, block.size);
}

static void clear_sram_range(Kinetis* device, size_t offset, size_t size) {
    memset(device->sram + offset, 0, size);
    memset(device->sram_initialized + offset, 0, size);
}

static void apply_vlls_sram_retention(Kinetis* device) {
    if (device->timing.smc[3] != 0x40u)
        return;

    uint32_t retained_size;
    switch (device->timing.smc[2] & 7u) {
    case 0u:
    case 1u:
        retained_size = 0u;
        break;
    case 2u:
        retained_size = (device->timing.smc[2] & 0x10u) != 0u
                            ? device->profile->vlls2_sram_upper_size_with_ram2
                            : device->profile->vlls2_sram_upper_size;
        break;
    case 3u:
        return;
    default:
        return;
    }

    const uint64_t device_begin = device->sram_base;
    const uint64_t device_end = device_begin + device->configuration.sram_size;
    const uint64_t retained_begin = device->profile->sram_upper_address;
    const uint64_t retained_end = retained_begin + retained_size;
    const uint64_t overlap_begin = device_begin > retained_begin ? device_begin : retained_begin;
    const uint64_t overlap_end = device_end < retained_end ? device_end : retained_end;
    if (overlap_begin >= overlap_end) {
        clear_sram_range(device, 0u, device->configuration.sram_size);
        return;
    }

    const size_t prefix_size = (size_t)(overlap_begin - device_begin);
    const size_t suffix_offset = (size_t)(overlap_end - device_begin);
    clear_sram_range(device, 0u, prefix_size);
    clear_sram_range(device, suffix_offset, device->configuration.sram_size - suffix_offset);
}

static bool save_register_file(const Kinetis* device, KinetisPeripheralId id,
                               uint8_t register_file[0x20]) {
    KinetisPeripheralBlock block;
    if (!kinetis_profile_peripheral_block(device->profile, id, &block) || block.size != 0x20u)
        return false;
    memcpy(register_file, device->peripheral + block.address - KINETIS_PERIPHERAL_BASE, block.size);
    return true;
}

static void restore_register_file(Kinetis* device, KinetisPeripheralId id,
                                  const uint8_t register_file[0x20]) {
    KinetisPeripheralBlock block;
    if (kinetis_profile_peripheral_block(device->profile, id, &block) && block.size == 0x20u)
        memcpy(device->peripheral + block.address - KINETIS_PERIPHERAL_BASE, register_file,
               block.size);
}

bool kinetis_reset(Kinetis* device) {
    if (device == NULL) {
        return false;
    }
    memset(device->sram, 0, device->configuration.sram_size);
    memset(device->sram_initialized, 0, device->configuration.sram_size);
    device->uninitialized_sram_read_count = 0u;
    device->first_uninitialized_sram_read = UINT32_MAX;
    device->cycles = 0;
    device->timing.elapsed_core_cycles = 0;
    sync_flash_configuration(device);
    kinetis_peripheral_reset(device);
    kinetis_timing_reset(&device->timing, 0x82u, 0);
    kinetis_sync_clock_gates(device);
    const bool reset_succeeded =
        cortex_m4_reset(device->cpu, device->configuration.vector_table_address);
    fmc_clear_cache(device);
    return reset_succeeded;
}

void kinetis_warm_reset(Kinetis* device, uint8_t reset_cause_0, uint8_t reset_cause_1) {
    if (device == NULL) {
        return;
    }
    device->uninitialized_sram_read_count = 0u;
    device->first_uninitialized_sram_read = UINT32_MAX;
    apply_vlls_sram_retention(device);
    uint8_t retained_vbat[0x20];
    uint8_t retained_system_registers[0x20];
    const bool has_vbat = save_register_file(device, KINETIS_PERIPHERAL_RFVBAT, retained_vbat);
    const bool has_system_registers =
        save_register_file(device, KINETIS_PERIPHERAL_RFSYS, retained_system_registers);
    sync_flash_configuration(device);
    kinetis_peripheral_reset(device);
    if (has_vbat)
        restore_register_file(device, KINETIS_PERIPHERAL_RFVBAT, retained_vbat);
    if (has_system_registers)
        restore_register_file(device, KINETIS_PERIPHERAL_RFSYS, retained_system_registers);
    kinetis_timing_warm_reset(&device->timing, reset_cause_0, reset_cause_1);
    kinetis_sync_clock_gates(device);
    (void)cortex_m4_reset(device->cpu, device->configuration.vector_table_address);
    fmc_clear_cache(device);
}

bool kinetis_load(Kinetis* device, uint32_t address, const void* input_data, size_t data_size) {
    if (device == NULL || input_data == NULL) {
        return false;
    }
    if (address < device->configuration.flash_size &&
        (uint64_t)address + data_size <= device->configuration.flash_size) {
        memcpy(device->flash + address, input_data, data_size);
        fmc_clear_cache(device);
        sync_flash_configuration(device);
        return true;
    }
    if (address >= device->sram_base &&
        (uint64_t)address + data_size <=
            (uint64_t)device->sram_base + device->configuration.sram_size) {
        memcpy(device->sram + address - device->sram_base, input_data, data_size);
        memset(device->sram_initialized + address - device->sram_base, 1, data_size);
        return true;
    }
    return false;
}

bool kinetis_seed(Kinetis* device, uint32_t address, const void* input_data, size_t data_size) {
    return kinetis_write(device, address, input_data, data_size);
}

bool kinetis_read(const Kinetis* device, uint32_t address, void* output_data, size_t data_size) {
    if (device == NULL || output_data == NULL) {
        return false;
    }
    uint8_t* output_bytes = output_data;
    size_t byte_offset = 0u;
    while (byte_offset < data_size) {
        const size_t remaining_bytes = data_size - byte_offset;
        const uint8_t access_size = remaining_bytes >= 4 && ((address + byte_offset) & 3u) == 0 ? 4
                                    : remaining_bytes >= 2 && ((address + byte_offset) & 1u) == 0
                                        ? 2
                                        : 1;
        uint32_t transfer_value = 0u;
        if (!kinetis_read_bus((Kinetis*)device, address + (uint32_t)byte_offset, access_size,
                              CORTEX_M4_ACCESS_DEBUG, &transfer_value)) {
            return false;
        }
        write_little_endian(output_bytes + byte_offset, access_size, transfer_value);
        byte_offset += access_size;
    }
    return true;
}

bool kinetis_write(Kinetis* device, uint32_t address, const void* input_data, size_t data_size) {
    if (device == NULL || input_data == NULL) {
        return false;
    }
    const uint8_t* input_bytes = input_data;
    size_t byte_offset = 0u;
    while (byte_offset < data_size) {
        const size_t remaining_bytes = data_size - byte_offset;
        const uint8_t access_size = remaining_bytes >= 4 && ((address + byte_offset) & 3u) == 0 ? 4
                                    : remaining_bytes >= 2 && ((address + byte_offset) & 1u) == 0
                                        ? 2
                                        : 1;
        const uint32_t transfer_value = read_little_endian(input_bytes + byte_offset, access_size);
        if (!kinetis_write_bus(device, address + (uint32_t)byte_offset, access_size,
                               CORTEX_M4_ACCESS_DEBUG, transfer_value)) {
            return false;
        }
        byte_offset += access_size;
    }
    return true;
}

bool kinetis_copy(Kinetis* destination, const Kinetis* source) {
    if (destination == NULL || source == NULL || destination->profile->id != source->profile->id ||
        destination->package != source->package ||
        destination->configuration.flash_size != source->configuration.flash_size ||
        destination->configuration.sram_size != source->configuration.sram_size) {
        return false;
    }
    memcpy(destination->flash, source->flash, source->configuration.flash_size);
    memcpy(destination->sram, source->sram, source->configuration.sram_size);
    memcpy(destination->sram_initialized, source->sram_initialized,
           source->configuration.sram_size);
    memcpy(destination->peripheral, source->peripheral, KINETIS_PERIPHERAL_SIZE);
    destination->configuration = source->configuration;
    destination->uninitialized_sram_read_count = source->uninitialized_sram_read_count;
    destination->first_uninitialized_sram_read = source->first_uninitialized_sram_read;
    destination->cycles = source->cycles;
    destination->cmt_cycles = source->cmt_cycles;
    destination->cmt_bus_remainder = source->cmt_bus_remainder;
    destination->cmt_mark_ticks = source->cmt_mark_ticks;
    destination->cmt_period_ticks = source->cmt_period_ticks;
    destination->cmt_carrier_high_ticks = source->cmt_carrier_high_ticks;
    destination->cmt_carrier_period_ticks = source->cmt_carrier_period_ticks;
    destination->cmt_carrier_offset_ticks = source->cmt_carrier_offset_ticks;
    destination->cmt_output_delay_ticks = source->cmt_output_delay_ticks;
    destination->cmt_eoc_read = source->cmt_eoc_read;
    destination->cmt_running = source->cmt_running;
    destination->cmt_stop_pending = source->cmt_stop_pending;
    destination->cmt_fsk_secondary = source->cmt_fsk_secondary;
    destination->cmt_extended_space = source->cmt_extended_space;
    destination->cmt_dma_pending = source->cmt_dma_pending;
    memcpy(destination->fmc_bank, source->fmc_bank, sizeof(destination->fmc_bank));
    memcpy(destination->fmc_age, source->fmc_age, sizeof(destination->fmc_age));
    destination->fmc_access_count = source->fmc_access_count;
    (void)kinetis_usbdcd_copy(&destination->usbdcd, &source->usbdcd);
    memcpy(destination->comparator_output, source->comparator_output,
           sizeof(destination->comparator_output));
    memcpy(destination->events, source->events, sizeof(destination->events));
    destination->event_read_index = source->event_read_index;
    destination->event_write_index = source->event_write_index;
    destination->event_count = source->event_count;
    if (!kinetis_data_copy(destination->data, source->data) ||
        !kinetis_sdhc_copy(&destination->sdhc, &source->sdhc, kinetis_sdhc_bus(destination)) ||
        !kinetis_serial_copy(&destination->serial, &source->serial) ||
        !kinetis_io_copy(&destination->io, &source->io) ||
        !kinetis_timing_copy(&destination->timing, &source->timing,
                             kinetis_timing_signals(destination))) {
        return false;
    }
    destination->io.configuration.event_handler =
        kinetis_io_configuration(destination).event_handler;
    destination->io.configuration.event_context = destination;
    if (source->flexbus_memory != NULL) {
        if (!kinetis_flexbus_attach(destination, source->flexbus_address, source->flexbus_memory,
                                    source->flexbus_size, source->flexbus_read_only))
            return false;
    } else {
        kinetis_flexbus_detach(destination);
    }
    return cortex_m4_copy(destination->cpu, source->cpu);
}

uint64_t kinetis_get_uninitialized_sram_read_count(const Kinetis* device) {
    return device == NULL ? 0u : device->uninitialized_sram_read_count;
}

uint32_t kinetis_get_first_uninitialized_sram_read(const Kinetis* device) {
    return device == NULL ? UINT32_MAX : device->first_uninitialized_sram_read;
}

void kinetis_clear_uninitialized_sram_reads(Kinetis* device) {
    if (device == NULL) {
        return;
    }
    device->uninitialized_sram_read_count = 0u;
    device->first_uninitialized_sram_read = UINT32_MAX;
}

static bool is_live_timing_register(uint32_t address) {
    const bool pit_state =
        address >= 0x40037104u && address <= 0x4003713cu &&
        ((address - 0x40037104u) % 0x10u == 0u || (address - 0x4003710cu) % 0x10u == 0u);
    return pit_state || address == 0x40052010u || address == 0x40052012u;
}

static bool read_comparable_register(Kinetis* device, uint32_t address, uint8_t size,
                                     uint32_t* value) {
    if (address >= 0x40052000u && address < 0x40052018u) {
        return kinetis_timing_projected_watchdog_read(&device->timing, address, size, value);
    }
    return kinetis_memory_read(device, address, size, CORTEX_M4_ACCESS_DEBUG, value);
}

bool kinetis_register_state_equal(Kinetis* first, Kinetis* second, uint32_t* first_difference,
                                  uint32_t* first_value, uint32_t* second_value) {
    if (first == NULL || second == NULL || first->profile != second->profile ||
        first_difference == NULL || first_value == NULL || second_value == NULL) {
        return false;
    }
    for (size_t index = 0u; index < first->manifest->register_count; ++index) {
        const KinetisRegisterDescriptor* descriptor = &first->manifest->registers[index];
        const bool fmc_cache_state = descriptor->address >= KINETIS_FMC + 0x100u &&
                                     descriptor->address < KINETIS_FMC + 0x300u;
        const bool dma_address = descriptor->address >= 0x40009000u &&
                                 descriptor->address < 0x40009200u &&
                                 ((descriptor->address - 0x40009000u) % 32u == 0u ||
                                  (descriptor->address - 0x40009000u) % 32u == 16u);
        if (fmc_cache_state || dma_address || is_live_timing_register(descriptor->address)) {
            continue;
        }
        const uint32_t mask = descriptor->implemented_mask & descriptor->read_mask;
        if ((descriptor->access & KINETIS_REGISTER_ACCESS_READ) == 0u || mask == 0u) {
            continue;
        }
        const uint8_t size = (uint8_t)(descriptor->width / 8u);
        uint32_t first_register = 0u;
        uint32_t second_register = 0u;
        const bool first_read =
            read_comparable_register(first, descriptor->address, size, &first_register);
        const bool second_read =
            read_comparable_register(second, descriptor->address, size, &second_register);
        if (first_read != second_read ||
            (first_read && (first_register & mask) != (second_register & mask))) {
            *first_difference = descriptor->address;
            *first_value = first_register & mask;
            *second_value = second_register & mask;
            return false;
        }
    }
    return true;
}

bool kinetis_flexbus_attach(Kinetis* device, uint32_t address, const void* input_data,
                            size_t window_size, bool read_only) {
    if (device == NULL || input_data == NULL || window_size == 0u ||
        (uint64_t)address + window_size > UINT64_C(0x100000000) ||
        !kinetis_package_has_peripheral(device->package, KINETIS_PERIPHERAL_FB))
        return false;
    uint8_t* window_memory = malloc(window_size);
    if (window_memory == NULL)
        return false;
    memcpy(window_memory, input_data, window_size);
    free(device->flexbus_memory);
    device->flexbus_memory = window_memory;
    device->flexbus_address = address;
    device->flexbus_size = window_size;
    device->flexbus_read_only = read_only;
    return true;
}

void kinetis_flexbus_detach(Kinetis* device) {
    if (device == NULL)
        return;
    free(device->flexbus_memory);
    device->flexbus_memory = NULL;
    device->flexbus_address = 0u;
    device->flexbus_size = 0u;
    device->flexbus_read_only = false;
}

bool kinetis_flexbus_read(const Kinetis* device, size_t window_offset, void* output_data,
                          size_t read_size) {
    if (device == NULL || output_data == NULL || device->flexbus_memory == NULL ||
        window_offset > device->flexbus_size || read_size > device->flexbus_size - window_offset)
        return false;
    memcpy(output_data, device->flexbus_memory + window_offset, read_size);
    return true;
}

void kinetis_advance(Kinetis* device, uint32_t cycle_count) {
    if (device != NULL && cycle_count != 0u) {
        kinetis_advance_bus(device, cycle_count);
    }
}

void kinetis_watchdog_advance(Kinetis* device, uint32_t ticks) {
    if (device != NULL)
        kinetis_timing_watchdog_advance(&device->timing, ticks);
}

uint32_t kinetis_core_clock_hz(const Kinetis* device) {
    return device == NULL ? 0 : kinetis_timing_core_clock_hz(&device->timing);
}

uint32_t kinetis_bus_clock_hz(const Kinetis* device) {
    return device == NULL ? 0 : kinetis_timing_bus_clock_hz(&device->timing);
}
