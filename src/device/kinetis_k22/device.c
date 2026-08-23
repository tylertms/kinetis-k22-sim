#include "kinetis_k22.h"

#include <stdlib.h>
#include <string.h>

#include "device/kinetis_k22/internal.h"

static bool valid_range(uint32_t address, uint8_t access_size, uint32_t region_base,
                        size_t region_size) {
    if (access_size == 0u || address < region_base)
        return false;
    return (uint64_t)address + access_size <= (uint64_t)region_base + region_size;
}

static uint32_t load_little_endian(const uint8_t* input_bytes, uint8_t byte_count) {
    uint32_t output_value = 0u;
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
        output_value |= (uint32_t)input_bytes[byte_index] << (byte_index * 8u);
    }
    return output_value;
}

static void store_little_endian(uint8_t* output_bytes, uint8_t byte_count, uint32_t write_value) {
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
        output_bytes[byte_index] = (uint8_t)(write_value >> (byte_index * 8u));
    }
}

static bool flash_access_allowed(const KinetisK22* device, CortexM4Access access, uint8_t master,
                                 bool write_access) {
    if (access == CORTEX_M4_ACCESS_DEBUG) {
        return true;
    }
    const uint32_t offset = 0x4001f000u - K22_PERIPHERAL_BASE;
    const uint32_t protection = load_little_endian(device->peripheral + offset, 4u);
    const uint8_t permission = (uint8_t)((protection >> (master * 2u)) & 3u);
    return write_access ? (permission & 2u) != 0u : (permission & 1u) != 0u;
}

static uint32_t fmc_raw_load(const KinetisK22* device, uint32_t address) {
    return load_little_endian(device->peripheral + address - K22_PERIPHERAL_BASE, 4u);
}

static void fmc_raw_store(KinetisK22* device, uint32_t address, uint32_t write_value) {
    store_little_endian(device->peripheral + address - K22_PERIPHERAL_BASE, 4u, write_value);
}

static uint32_t fmc_tag_address(uint8_t cache_way, uint8_t cache_set) {
    return 0x4001f100u + ((uint32_t)cache_way * 4u + cache_set) * 4u;
}

static uint32_t fmc_data_address(uint8_t cache_way, uint8_t cache_set, uint8_t word_index) {
    return 0x4001f200u + ((uint32_t)cache_way * 4u + cache_set) * 16u + (3u - word_index) * 4u;
}

static uint32_t fmc_tag_value(const KinetisK22* device, uint32_t cache_line);

static void fmc_clear_entry(KinetisK22* device, uint8_t cache_way, uint8_t cache_set) {
    fmc_raw_store(device, fmc_tag_address(cache_way, cache_set), 0u);
    for (uint8_t word_index = 0u; word_index < 4u; word_index++) {
        fmc_raw_store(device, fmc_data_address(cache_way, cache_set, word_index), 0u);
    }
    device->fmc_bank[cache_way][cache_set] = 0u;
    device->fmc_age[cache_way][cache_set] = 0u;
}

static void fmc_clear_cache(KinetisK22* device) {
    for (uint8_t way = 0u; way < 4u; way++) {
        for (uint8_t cache_set = 0u; cache_set < 4u; cache_set++) {
            fmc_clear_entry(device, way, cache_set);
        }
    }
    device->fmc_access_count = 0u;
}

static void fmc_invalidate_line(KinetisK22* device, uint8_t flash_bank, uint32_t flash_offset) {
    const uint8_t cache_set = (uint8_t)((flash_offset >> 4u) & 3u);
    const uint32_t tag = fmc_tag_value(device, flash_offset & ~15u);
    for (uint8_t cache_way = 0u; cache_way < 4u; cache_way++) {
        if (device->fmc_bank[cache_way][cache_set] == flash_bank &&
            fmc_raw_load(device, fmc_tag_address(cache_way, cache_set)) == tag) {
            fmc_clear_entry(device, cache_way, cache_set);
        }
    }
}

static uint32_t fmc_tag_value(const KinetisK22* device, uint32_t cache_line_address) {
    const K22RegisterDescriptor* descriptor =
        k22_register_manifest_lookup(device->profile->id, 0x4001f100u, 32u);
    return (cache_line_address & descriptor->write_mask & ~1u) | 1u;
}

static bool fmc_way_allowed(uint8_t replacement_mode, bool instruction_access, uint8_t cache_way) {
    if (replacement_mode == 2u)
        return instruction_access ? cache_way < 2u : cache_way >= 2u;
    if (replacement_mode == 3u)
        return instruction_access ? cache_way < 3u : cache_way == 3u;
    return true;
}

static uint8_t fmc_replacement_way(KinetisK22* device, uint8_t cache_set, bool instruction_access) {
    const uint32_t cache_control = fmc_raw_load(device, 0x4001f004u);
    const uint8_t locked_way_mask = (uint8_t)((cache_control >> 24u) & 0x0fu);
    const uint8_t replacement_mode = (uint8_t)((cache_control >> 5u) & 7u);
    for (uint8_t cache_way = 0u; cache_way < 4u; cache_way++) {
        if ((locked_way_mask & (1u << cache_way)) == 0u &&
            fmc_way_allowed(replacement_mode, instruction_access, cache_way) &&
            (fmc_raw_load(device, fmc_tag_address(cache_way, cache_set)) & 1u) == 0u) {
            return cache_way;
        }
    }
    uint8_t selected_way = UINT8_MAX;
    uint64_t oldest_age = UINT64_MAX;
    for (uint8_t cache_way = 0u; cache_way < 4u; cache_way++) {
        if ((locked_way_mask & (1u << cache_way)) == 0u &&
            fmc_way_allowed(replacement_mode, instruction_access, cache_way) &&
            device->fmc_age[cache_way][cache_set] < oldest_age) {
            selected_way = cache_way;
            oldest_age = device->fmc_age[cache_way][cache_set];
        }
    }
    return selected_way;
}

static uint8_t fmc_source_byte(KinetisK22* device, uint8_t flash_bank, uint32_t flash_offset) {
    if (flash_bank == 0u)
        return flash_offset < device->configuration.flash_size
                   ? device->flash[k22_data_program_flash_address(device->data, flash_offset)]
                   : UINT8_MAX;
    uint32_t output_value = UINT32_MAX;
    return k22_data_read(device->data, device->profile->flexnvm_address + flash_offset, 1u,
                         &output_value)
               ? (uint8_t)output_value
               : UINT8_MAX;
}

static uint8_t fmc_cached_byte(KinetisK22* device, uint8_t flash_bank, uint32_t flash_offset,
                               CortexM4Access access) {
    const uint32_t cache_line_address = flash_offset & ~15u;
    const uint8_t cache_set = (uint8_t)((flash_offset >> 4u) & 3u);
    const uint32_t tag = fmc_tag_value(device, cache_line_address);
    uint8_t cache_way = UINT8_MAX;
    for (uint8_t candidate_way = 0u; candidate_way < 4u; candidate_way++) {
        if (device->fmc_bank[candidate_way][cache_set] == flash_bank &&
            fmc_raw_load(device, fmc_tag_address(candidate_way, cache_set)) == tag) {
            cache_way = candidate_way;
            break;
        }
    }
    if (cache_way == UINT8_MAX) {
        cache_way = fmc_replacement_way(device, cache_set, access == CORTEX_M4_ACCESS_INSTRUCTION);
        if (cache_way == UINT8_MAX)
            return fmc_source_byte(device, flash_bank, flash_offset);
        for (uint8_t word_index = 0u; word_index < 4u; word_index++) {
            uint32_t cache_word = 0u;
            for (uint8_t byte_index = 0u; byte_index < 4u; byte_index++) {
                const uint32_t source_address =
                    cache_line_address + (uint32_t)word_index * 4u + byte_index;
                cache_word |= (uint32_t)fmc_source_byte(device, flash_bank, source_address)
                              << (byte_index * 8u);
            }
            fmc_raw_store(device, fmc_data_address(cache_way, cache_set, word_index), cache_word);
        }
        fmc_raw_store(device, fmc_tag_address(cache_way, cache_set), tag);
        device->fmc_bank[cache_way][cache_set] = flash_bank;
    }
    device->fmc_age[cache_way][cache_set] = ++device->fmc_access_count;
    const uint8_t word_index = (uint8_t)((flash_offset >> 2u) & 3u);
    return (uint8_t)(fmc_raw_load(device, fmc_data_address(cache_way, cache_set, word_index)) >>
                     ((flash_offset & 3u) * 8u));
}

static bool fmc_cache_enabled(const KinetisK22* device, uint8_t bank, CortexM4Access access) {
    if (access == CORTEX_M4_ACCESS_DEBUG)
        return false;
    const uint32_t cache_control = fmc_raw_load(device, 0x4001f004u + (uint32_t)bank * 4u);
    return (cache_control & (access == CORTEX_M4_ACCESS_INSTRUCTION ? 8u : 16u)) != 0u;
}

static bool sysmpu_access_allowed(KinetisK22* device, uint32_t address, uint8_t size,
                                  uint8_t master, bool supervisor, K22SysMpuAccess access) {
    if (!k22_profile_has_peripheral(device->profile, K22_PERIPHERAL_SYSMPU) ||
        !k22_io_clock_enabled(&device->io, K22_PERIPHERAL_SYSMPU))
        return true;
    if (!k22_io_sysmpu_access(&device->io, address, master, supervisor, access))
        return false;
    return k22_io_sysmpu_access(&device->io, address + size - 1u, master, supervisor, access);
}

static bool memory_read_unprotected(KinetisK22* device, uint32_t address, uint8_t size,
                                    CortexM4Access access, uint8_t flash_master,
                                    uint32_t* output_value) {
    if (valid_range(address, size, K22_FLASH_BASE, device->configuration.flash_size)) {
        if (!flash_access_allowed(device, access, flash_master, false)) {
            return false;
        }
        if (!k22_data_flash_read(device->data, false, address, size)) {
            *output_value = size == 4u ? UINT32_MAX : (1u << (size * 8u)) - 1u;
            return true;
        }
        if (fmc_cache_enabled(device, 0u, access)) {
            *output_value = 0u;
            for (uint8_t byte_index = 0u; byte_index < size; byte_index++)
                *output_value |= (uint32_t)fmc_cached_byte(device, 0u, address + byte_index, access)
                                 << (byte_index * 8u);
        } else {
            *output_value = 0u;
            for (uint8_t byte_index = 0u; byte_index < size; byte_index++)
                *output_value |=
                    (uint32_t)device
                        ->flash[k22_data_program_flash_address(device->data, address + byte_index)]
                    << (byte_index * 8u);
        }
        return true;
    }
    if (valid_range(address, size, device->sram_base, device->configuration.sram_size)) {
        *output_value = load_little_endian(device->sram + address - device->sram_base, size);
        return true;
    }
    if (device->profile->flexnvm_size != 0u &&
        valid_range(address, size, device->profile->flexnvm_address,
                    device->profile->flexnvm_size)) {
        if (!flash_access_allowed(device, access, flash_master, false))
            return false;
        const uint32_t offset = address - device->profile->flexnvm_address;
        if (!k22_data_flash_read(device->data, true, offset, size)) {
            *output_value = size == 4u ? UINT32_MAX : (1u << (size * 8u)) - 1u;
            return true;
        }
        if (fmc_cache_enabled(device, 1u, access)) {
            *output_value = 0u;
            for (uint8_t byte_index = 0u; byte_index < size; byte_index++)
                *output_value |= (uint32_t)fmc_cached_byte(device, 1u, offset + byte_index, access)
                                 << (byte_index * 8u);
            return true;
        }
        return k22_data_read(device->data, address, size, output_value);
    }
    if (device->profile->flexram_size != 0u &&
        valid_range(address, size, device->profile->flexram_address,
                    device->profile->flexram_size)) {
        if (!flash_access_allowed(device, access, flash_master, false))
            return false;
        return k22_data_read(device->data, address, size, output_value);
    }
    if (device->flexbus_memory != NULL &&
        valid_range(address, size, device->flexbus_address, device->flexbus_size) &&
        k22_io_flexbus_transfer(&device->io, address, size, false, 0u)) {
        *output_value =
            load_little_endian(device->flexbus_memory + address - device->flexbus_address, size);
        return true;
    }
    return kinetis_k22_peripheral_read(device, address, size, access, output_value);
}

static bool memory_write_unprotected(KinetisK22* device, uint32_t address, uint8_t size,
                                     CortexM4Access access, uint8_t flash_master,
                                     uint32_t write_value) {
    if (valid_range(address, size, K22_FLASH_BASE, device->configuration.flash_size)) {
        if (access != CORTEX_M4_ACCESS_DEBUG) {
            return false;
        }
        for (uint8_t byte_index = 0u; byte_index < size; byte_index++)
            device->flash[k22_data_program_flash_address(device->data, address + byte_index)] =
                (uint8_t)(write_value >> (byte_index * 8u));
        fmc_invalidate_line(device, 0u, address);
        fmc_invalidate_line(device, 0u, address + size - 1u);
        return true;
    }
    if (valid_range(address, size, device->sram_base, device->configuration.sram_size)) {
        store_little_endian(device->sram + address - device->sram_base, size, write_value);
        return true;
    }
    if (device->profile->flexnvm_size != 0u &&
        valid_range(address, size, device->profile->flexnvm_address,
                    device->profile->flexnvm_size)) {
        if (access != CORTEX_M4_ACCESS_DEBUG ||
            !k22_data_write(device->data, address, size, write_value))
            return false;
        const uint32_t offset = address - device->profile->flexnvm_address;
        fmc_invalidate_line(device, 1u, offset);
        fmc_invalidate_line(device, 1u, offset + size - 1u);
        return true;
    }
    if (device->profile->flexram_size != 0u &&
        valid_range(address, size, device->profile->flexram_address,
                    device->profile->flexram_size)) {
        if (!flash_access_allowed(device, access, flash_master, true))
            return false;
        return k22_data_write(device->data, address, size, write_value);
    }
    if (device->flexbus_memory != NULL && !device->flexbus_read_only &&
        valid_range(address, size, device->flexbus_address, device->flexbus_size) &&
        k22_io_flexbus_transfer(&device->io, address, size, true, write_value)) {
        store_little_endian(device->flexbus_memory + address - device->flexbus_address, size,
                            write_value);
        return true;
    }
    return kinetis_k22_peripheral_write(device, address, size, access, write_value);
}

bool kinetis_k22_memory_read(KinetisK22* device, uint32_t address, uint8_t size,
                             CortexM4Access access, uint32_t* output_value) {
    if (device == NULL || output_value == NULL || (size != 1 && size != 2 && size != 4))
        return false;
    if (access != CORTEX_M4_ACCESS_DEBUG) {
        const uint8_t master = access == CORTEX_M4_ACCESS_INSTRUCTION ? 0u : 1u;
        const bool supervisor = access != CORTEX_M4_ACCESS_UNPRIVILEGED_DATA &&
                                (access != CORTEX_M4_ACCESS_INSTRUCTION || device->cpu == NULL ||
                                 (cortex_m4_get_xpsr(device->cpu) & 0x1ffu) != 0u ||
                                 (cortex_m4_get_control(device->cpu) & 1u) == 0u);
        const K22SysMpuAccess operation =
            access == CORTEX_M4_ACCESS_INSTRUCTION ? K22_SYSMPU_EXECUTE : K22_SYSMPU_READ;
        if (!sysmpu_access_allowed(device, address, size, master, supervisor, operation))
            return false;
    }
    const uint8_t master = access == CORTEX_M4_ACCESS_INSTRUCTION ? 0u : 1u;
    return memory_read_unprotected(device, address, size, access, master, output_value);
}

bool kinetis_k22_memory_write(KinetisK22* device, uint32_t address, uint8_t size,
                              CortexM4Access access, uint32_t write_value) {
    if (device == NULL || (size != 1 && size != 2 && size != 4))
        return false;
    if (access != CORTEX_M4_ACCESS_DEBUG) {
        const bool supervisor = access != CORTEX_M4_ACCESS_UNPRIVILEGED_DATA;
        if (!sysmpu_access_allowed(device, address, size, 1u, supervisor, K22_SYSMPU_WRITE))
            return false;
    }
    return memory_write_unprotected(device, address, size, access, 1u, write_value);
}

bool kinetis_k22_dma_read(KinetisK22* device, uint32_t address, uint8_t size,
                          uint32_t* output_value) {
    if (device == NULL || output_value == NULL || (size != 1u && size != 2u && size != 4u) ||
        !sysmpu_access_allowed(device, address, size, 2u, true, K22_SYSMPU_READ))
        return false;
    return memory_read_unprotected(device, address, size, CORTEX_M4_ACCESS_DATA, 2u, output_value);
}

bool kinetis_k22_dma_write(KinetisK22* device, uint32_t address, uint8_t size,
                           uint32_t write_value) {
    if (device == NULL || (size != 1u && size != 2u && size != 4u) ||
        !sysmpu_access_allowed(device, address, size, 2u, true, K22_SYSMPU_WRITE))
        return false;
    return memory_write_unprotected(device, address, size, CORTEX_M4_ACCESS_DATA, 2u, write_value);
}

bool kinetis_k22_flash_controller_write(KinetisK22* device, uint32_t address, uint8_t size,
                                        uint32_t write_value) {
    if (device == NULL || (size != 1u && size != 2u && size != 4u) ||
        !valid_range(address, size, K22_FLASH_BASE, device->configuration.flash_size))
        return false;
    store_little_endian(device->flash + address, size, write_value);
    return true;
}

static const K22RegisterDescriptor* bit_band_descriptor(const KinetisK22* device,
                                                        uint32_t byte_address) {
    for (size_t register_index = 0; register_index < device->manifest->register_count;
         register_index++) {
        const K22RegisterDescriptor* descriptor = &device->manifest->registers[register_index];
        const uint8_t size = (uint8_t)(descriptor->width / 8u);
        if (byte_address >= descriptor->address && byte_address - descriptor->address < size) {
            return descriptor;
        }
    }
    return NULL;
}

static bool bit_band_read(KinetisK22* device, uint32_t address, CortexM4Access access,
                          uint32_t* output_value) {
    if ((address & 3u) != 0) {
        return false;
    }
    const uint32_t alias = address - K22_BIT_BAND_BASE;
    const uint32_t byte_address = K22_PERIPHERAL_BASE + alias / 32u;
    const uint8_t bit_in_byte = (uint8_t)((alias / 4u) & 7u);
    const K22RegisterDescriptor* descriptor = bit_band_descriptor(device, byte_address);
    if (descriptor == NULL || (descriptor->access & K22_REGISTER_ACCESS_READ) == 0) {
        return false;
    }
    const uint8_t bit = (uint8_t)((byte_address - descriptor->address) * 8u + bit_in_byte);
    const uint32_t mask = 1u << bit;
    if ((descriptor->implemented_mask & descriptor->read_mask & mask) == 0) {
        return false;
    }
    uint32_t register_value = 0;
    if (!kinetis_k22_peripheral_read(device, descriptor->address, (uint8_t)(descriptor->width / 8u),
                                     access, &register_value)) {
        return false;
    }
    *output_value = (register_value & mask) != 0;
    return true;
}

static bool bit_band_write(KinetisK22* device, uint32_t address, CortexM4Access access,
                           uint32_t write_value) {
    if ((address & 3u) != 0) {
        return false;
    }
    const uint32_t alias = address - K22_BIT_BAND_BASE;
    const uint32_t byte_address = K22_PERIPHERAL_BASE + alias / 32u;
    const uint8_t bit_in_byte = (uint8_t)((alias / 4u) & 7u);
    const K22RegisterDescriptor* descriptor = bit_band_descriptor(device, byte_address);
    if (descriptor == NULL || (descriptor->access & K22_REGISTER_ACCESS_WRITE) == 0) {
        return false;
    }
    const uint8_t bit = (uint8_t)((byte_address - descriptor->address) * 8u + bit_in_byte);
    const uint32_t mask = 1u << bit;
    if ((descriptor->implemented_mask & descriptor->write_mask & mask) == 0) {
        return false;
    }
    uint32_t register_value = 0;
    if (!kinetis_k22_peripheral_read(device, descriptor->address, (uint8_t)(descriptor->width / 8u),
                                     access, &register_value)) {
        return false;
    }
    register_value &= ~descriptor->w1c_mask;
    if ((descriptor->w1c_mask & mask) != 0) {
        if ((write_value & 1u) != 0) {
            register_value |= mask;
        }
    } else {
        register_value = (write_value & 1u) != 0 ? register_value | mask : register_value & ~mask;
    }
    return kinetis_k22_peripheral_write(device, descriptor->address,
                                        (uint8_t)(descriptor->width / 8u), access, register_value);
}

static bool k22_read_bus(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                         uint32_t* output_value) {
    KinetisK22* device = context;
    if (address >= K22_BIT_BAND_BASE && address < K22_BIT_BAND_BASE + K22_BIT_BAND_SIZE) {
        if (size != 1u && size != 2u && size != 4u)
            return false;
        return bit_band_read(device, address, access, output_value);
    }
    return kinetis_k22_memory_read(device, address, size, access, output_value);
}

static bool k22_write_bus(void* context, uint32_t address, uint8_t size, CortexM4Access access,
                          uint32_t write_value) {
    KinetisK22* device = context;
    if (address >= K22_BIT_BAND_BASE && address < K22_BIT_BAND_BASE + K22_BIT_BAND_SIZE) {
        if (size != 1u && size != 2u && size != 4u)
            return false;
        return bit_band_write(device, address, access, write_value);
    }
    return kinetis_k22_memory_write(device, address, size, access, write_value);
}

static void k22_advance_bus(void* context, uint32_t cycles) {
    KinetisK22* device = context;
    device->cycles += cycles;
    kinetis_k22_peripheral_advance(device, cycles);
}

static void k22_reset_bus(void* context) { kinetis_k22_warm_reset(context, 0, 0x04u); }

KinetisK22Configuration kinetis_k22_configuration(KinetisK22Profile profile_id) {
    KinetisK22Configuration configuration;
    const K22Profile* profile = k22_profile_get(profile_id);
    memset(&configuration, 0, sizeof(configuration));
    configuration.profile = profile_id;
    configuration.package = KINETIS_K22_PACKAGE_DEFAULT;
    if (profile != NULL) {
        configuration.flash_size = profile->program_flash_size;
        configuration.sram_size = profile->sram_lower_size + profile->sram_upper_size;
    }
    configuration.external_oscillator_hz = 8000000u;
    configuration.rtc_oscillator_hz = 32768u;
    return configuration;
}

KinetisK22Configuration kinetis_k22_default_configuration(void) {
    return kinetis_k22_configuration(KINETIS_K22_PROFILE_MK22FN51212);
}

bool kinetis_k22_profile_from_name(const char* name, KinetisK22Profile* profile) {
    const K22Profile* match = k22_profile_find(name);
    if (match == NULL || profile == NULL) {
        return false;
    }
    *profile = match->id;
    return true;
}

const char* kinetis_k22_profile_name(KinetisK22Profile profile) {
    const K22Profile* match = k22_profile_get(profile);
    return match == NULL ? NULL : match->name;
}

bool kinetis_k22_package_from_code(const char* code, KinetisK22Package* package) {
    const K22Package* match = k22_package_find(code);
    if (match == NULL || package == NULL) {
        return false;
    }
    *package = (KinetisK22Package)match->id;
    return true;
}

const char* kinetis_k22_package_code(KinetisK22Package package) {
    if (package == KINETIS_K22_PACKAGE_DEFAULT) {
        return NULL;
    }
    const K22Package* match = k22_package_get((K22PackageId)package);
    return match == NULL ? NULL : match->code;
}

static void destroy_partial(KinetisK22* device) {
    if (device == NULL) {
        return;
    }
    cortex_m4_destroy(device->cpu);
    k22_data_destroy(device->data);
    k22_sdhc_destroy(&device->sdhc);
    free(device->flexbus_memory);
    free(device->flash);
    free(device->sram);
    free(device->peripheral);
    free(device);
}

KinetisK22* kinetis_k22_create(KinetisK22Configuration configuration) {
    const K22Profile* profile = k22_profile_get(configuration.profile);
    const K22RegisterManifest* manifest = k22_register_manifest_get(configuration.profile);
    if (profile == NULL || manifest == NULL) {
        return NULL;
    }
    const K22PackageSelection* package =
        configuration.package == KINETIS_K22_PACKAGE_DEFAULT
            ? k22_package_default(profile)
            : k22_package_select(profile, (K22PackageId)configuration.package);
    if (package == NULL || configuration.flash_size == 0 ||
        configuration.flash_size > profile->program_flash_size || configuration.sram_size == 0 ||
        configuration.sram_size > 0x40000000u) {
        return NULL;
    }
    KinetisK22* device = calloc(1, sizeof(*device));
    if (device == NULL) {
        return NULL;
    }
    device->configuration = configuration;
    device->profile = profile;
    device->package = package;
    device->manifest = manifest;
    const size_t profile_sram_size = (size_t)profile->sram_lower_size + profile->sram_upper_size;
    device->sram_base = configuration.sram_size == profile_sram_size
                            ? profile->sram_lower_address
                            : K22_SRAM_CENTER - (uint32_t)(configuration.sram_size / 2u);
    device->flash = malloc(configuration.flash_size);
    device->sram = calloc(1, configuration.sram_size);
    device->peripheral = calloc(1, K22_PERIPHERAL_SIZE);
    if (device->flash == NULL || device->sram == NULL || device->peripheral == NULL) {
        destroy_partial(device);
        return NULL;
    }
    memset(device->flash, 0xff, configuration.flash_size);
    if (!k22_serial_init(&device->serial, profile) ||
        !k22_sdhc_init(&device->sdhc, kinetis_k22_sdhc_bus(device)) ||
        !k22_timing_init(&device->timing, profile, configuration.external_oscillator_hz,
                         configuration.rtc_oscillator_hz, kinetis_k22_timing_signals(device)) ||
        !k22_io_init(&device->io, kinetis_k22_io_configuration(device))) {
        destroy_partial(device);
        return NULL;
    }
    device->data = k22_data_create(profile, kinetis_k22_data_bus(device));
    if (device->data == NULL) {
        destroy_partial(device);
        return NULL;
    }
    kinetis_k22_sync_clock_gates(device);
    CortexM4Bus bus = {device, k22_read_bus, k22_write_bus, k22_advance_bus, k22_reset_bus};
    device->cpu = cortex_m4_create(bus);
    if (device->cpu == NULL ||
        !cortex_m4_configure_implementation(device->cpu, profile->cpu.external_irq_count,
                                            profile->cpu.nvic_priority_bits,
                                            profile->cpu.has_mpu ? 8u : 0u)) {
        destroy_partial(device);
        return NULL;
    }
    return device;
}

void kinetis_k22_destroy(KinetisK22* device) { destroy_partial(device); }

CortexM4* kinetis_k22_cpu(KinetisK22* device) { return device == NULL ? NULL : device->cpu; }

const CortexM4* kinetis_k22_cpu_const(const KinetisK22* device) {
    return device == NULL ? NULL : device->cpu;
}

static void sync_flash_configuration(KinetisK22* device) {
    K22PeripheralBlock block;
    if (!k22_profile_peripheral_block(device->profile, K22_PERIPHERAL_FLASH_CONFIG, &block) ||
        block.address + block.size > device->configuration.flash_size) {
        return;
    }
    (void)k22_data_set_flash_configuration(device->data, device->flash + block.address, block.size);
    memcpy(device->io.configuration.flash_configuration, device->flash + block.address, block.size);
}

bool kinetis_k22_reset(KinetisK22* device) {
    if (device == NULL) {
        return false;
    }
    memset(device->sram, 0, device->configuration.sram_size);
    device->cycles = 0;
    device->timing.elapsed_core_cycles = 0;
    sync_flash_configuration(device);
    kinetis_k22_peripheral_reset(device);
    k22_timing_reset(&device->timing, 0x82u, 0);
    kinetis_k22_sync_clock_gates(device);
    const bool reset = cortex_m4_reset(device->cpu, device->configuration.vector_table_address);
    fmc_clear_cache(device);
    return reset;
}

void kinetis_k22_warm_reset(KinetisK22* device, uint8_t cause_0, uint8_t cause_1) {
    if (device == NULL) {
        return;
    }
    uint8_t retained_vbat[0x20];
    memcpy(retained_vbat, device->peripheral + 0x3e000u, sizeof(retained_vbat));
    sync_flash_configuration(device);
    kinetis_k22_peripheral_reset(device);
    memcpy(device->peripheral + 0x3e000u, retained_vbat, sizeof(retained_vbat));
    k22_timing_warm_reset(&device->timing, cause_0, cause_1);
    kinetis_k22_sync_clock_gates(device);
    (void)cortex_m4_reset(device->cpu, device->configuration.vector_table_address);
    fmc_clear_cache(device);
}

bool kinetis_k22_load(KinetisK22* device, uint32_t address, const void* data, size_t size) {
    if (device == NULL || data == NULL) {
        return false;
    }
    if (address < device->configuration.flash_size &&
        (uint64_t)address + size <= device->configuration.flash_size) {
        memcpy(device->flash + address, data, size);
        fmc_clear_cache(device);
        sync_flash_configuration(device);
        return true;
    }
    if (address >= device->sram_base &&
        (uint64_t)address + size <= (uint64_t)device->sram_base + device->configuration.sram_size) {
        memcpy(device->sram + address - device->sram_base, data, size);
        return true;
    }
    return false;
}

bool kinetis_k22_seed(KinetisK22* device, uint32_t address, const void* data, size_t size) {
    return kinetis_k22_write(device, address, data, size);
}

bool kinetis_k22_read(const KinetisK22* device, uint32_t address, void* data, size_t size) {
    if (device == NULL || data == NULL) {
        return false;
    }
    uint8_t* output = data;
    size_t offset = 0;
    while (offset < size) {
        const size_t remaining = size - offset;
        const uint8_t width = remaining >= 4 && ((address + offset) & 3u) == 0   ? 4
                              : remaining >= 2 && ((address + offset) & 1u) == 0 ? 2
                                                                                 : 1;
        uint32_t value = 0;
        if (!k22_read_bus((KinetisK22*)device, address + (uint32_t)offset, width,
                          CORTEX_M4_ACCESS_DEBUG, &value)) {
            return false;
        }
        store_little_endian(output + offset, width, value);
        offset += width;
    }
    return true;
}

bool kinetis_k22_write(KinetisK22* device, uint32_t address, const void* data, size_t size) {
    if (device == NULL || data == NULL) {
        return false;
    }
    const uint8_t* input = data;
    size_t offset = 0;
    while (offset < size) {
        const size_t remaining = size - offset;
        const uint8_t width = remaining >= 4 && ((address + offset) & 3u) == 0   ? 4
                              : remaining >= 2 && ((address + offset) & 1u) == 0 ? 2
                                                                                 : 1;
        const uint32_t value = load_little_endian(input + offset, width);
        if (!k22_write_bus(device, address + (uint32_t)offset, width, CORTEX_M4_ACCESS_DEBUG,
                           value)) {
            return false;
        }
        offset += width;
    }
    return true;
}

bool kinetis_k22_copy(KinetisK22* destination, const KinetisK22* source) {
    if (destination == NULL || source == NULL || destination->profile->id != source->profile->id ||
        destination->package != source->package ||
        destination->configuration.flash_size != source->configuration.flash_size ||
        destination->configuration.sram_size != source->configuration.sram_size) {
        return false;
    }
    memcpy(destination->flash, source->flash, source->configuration.flash_size);
    memcpy(destination->sram, source->sram, source->configuration.sram_size);
    memcpy(destination->peripheral, source->peripheral, K22_PERIPHERAL_SIZE);
    destination->configuration = source->configuration;
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
    (void)k22_usbdcd_copy(&destination->usbdcd, &source->usbdcd);
    memcpy(destination->comparator_output, source->comparator_output,
           sizeof(destination->comparator_output));
    memcpy(destination->events, source->events, sizeof(destination->events));
    destination->event_read_index = source->event_read_index;
    destination->event_write_index = source->event_write_index;
    destination->event_count = source->event_count;
    if (!k22_data_copy(destination->data, source->data) ||
        !k22_sdhc_copy(&destination->sdhc, &source->sdhc, kinetis_k22_sdhc_bus(destination)) ||
        !k22_serial_copy(&destination->serial, &source->serial) ||
        !k22_io_copy(&destination->io, &source->io) ||
        !k22_timing_copy(&destination->timing, &source->timing,
                         kinetis_k22_timing_signals(destination))) {
        return false;
    }
    destination->io.configuration.event_handler =
        kinetis_k22_io_configuration(destination).event_handler;
    destination->io.configuration.event_context = destination;
    if (source->flexbus_memory != NULL) {
        if (!kinetis_k22_flexbus_attach(destination, source->flexbus_address,
                                        source->flexbus_memory, source->flexbus_size,
                                        source->flexbus_read_only))
            return false;
    } else {
        kinetis_k22_flexbus_detach(destination);
    }
    return cortex_m4_copy(destination->cpu, source->cpu);
}

bool kinetis_k22_flexbus_attach(KinetisK22* device, uint32_t address, const void* data, size_t size,
                                bool read_only) {
    if (device == NULL || data == NULL || size == 0u ||
        (uint64_t)address + size > UINT64_C(0x100000000) ||
        !k22_package_has_peripheral(device->package, K22_PERIPHERAL_FB))
        return false;
    uint8_t* memory = malloc(size);
    if (memory == NULL)
        return false;
    memcpy(memory, data, size);
    free(device->flexbus_memory);
    device->flexbus_memory = memory;
    device->flexbus_address = address;
    device->flexbus_size = size;
    device->flexbus_read_only = read_only;
    return true;
}

void kinetis_k22_flexbus_detach(KinetisK22* device) {
    if (device == NULL)
        return;
    free(device->flexbus_memory);
    device->flexbus_memory = NULL;
    device->flexbus_address = 0u;
    device->flexbus_size = 0u;
    device->flexbus_read_only = false;
}

bool kinetis_k22_flexbus_read(const KinetisK22* device, size_t offset, void* data, size_t size) {
    if (device == NULL || data == NULL || device->flexbus_memory == NULL ||
        offset > device->flexbus_size || size > device->flexbus_size - offset)
        return false;
    memcpy(data, device->flexbus_memory + offset, size);
    return true;
}

void kinetis_k22_advance(KinetisK22* device, uint32_t cycles) {
    if (device != NULL && cycles != 0) {
        k22_advance_bus(device, cycles);
    }
}

void kinetis_k22_watchdog_advance(KinetisK22* device, uint32_t ticks) {
    if (device != NULL)
        k22_timing_watchdog_advance(&device->timing, ticks);
}

uint32_t kinetis_k22_core_clock_hz(const KinetisK22* device) {
    return device == NULL ? 0 : k22_timing_core_clock_hz(&device->timing);
}

uint32_t kinetis_k22_bus_clock_hz(const KinetisK22* device) {
    return device == NULL ? 0 : k22_timing_bus_clock_hz(&device->timing);
}
