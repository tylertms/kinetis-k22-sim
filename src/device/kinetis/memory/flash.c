#include "internal.h"

#include <string.h>

static uint8_t flash_fccob_offset(uint8_t fccob_index) {
    static const uint8_t offsets[12] = {7u, 6u, 5u, 4u, 11u, 10u, 9u, 8u, 15u, 14u, 13u, 12u};
    return offsets[fccob_index];
}

static uint8_t flash_fccob(const KinetisData* data, uint8_t fccob_index) {
    return data->flash[flash_fccob_offset(fccob_index)];
}

static void flash_set_fccob(KinetisData* data, uint8_t fccob_index, uint8_t byte_value) {
    data->flash[flash_fccob_offset(fccob_index)] = byte_value;
}

static uint32_t flash_address(const KinetisData* data) {
    return ((uint32_t)flash_fccob(data, 1u) << 16u) | ((uint32_t)flash_fccob(data, 2u) << 8u) |
           flash_fccob(data, 3u);
}

uint32_t kinetis_data_program_flash_address(const KinetisData* data, uint32_t address) {
    if (data == NULL || data->flash_swap_current_block == 0u ||
        address >= data->profile->program_flash_size)
        return address;
    return address ^ (data->profile->program_flash_size / 2u);
}

static bool flash_store(KinetisData* data, uint32_t address, uint8_t byte_count,
                        uint32_t write_value) {
    bool write_succeeded = false;
    const uint32_t physical_address = kinetis_data_program_flash_address(data, address);
    if (data->bus.program != NULL)
        write_succeeded =
            data->bus.program(data->bus.context, physical_address, byte_count, write_value);
    else if (data->bus.write != NULL)
        write_succeeded =
            data->bus.write(data->bus.context, physical_address, byte_count, write_value);
    if (!write_succeeded)
        return false;
    if (address < 0x410u && address + byte_count > 0x400u) {
        for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
            const uint32_t byte_address = address + byte_index;
            if (byte_address >= 0x400u && byte_address < 0x410u)
                data->flash_config[byte_address - 0x400u] =
                    (uint8_t)(write_value >> (byte_index * 8u));
        }
    }
    return true;
}

static bool flash_load(KinetisData* data, uint32_t address, uint8_t byte_count,
                       uint32_t* output_value) {
    if (address >= 0x400u && address <= 0x410u - byte_count) {
        *output_value =
            kinetis_data_internal_load_bytes(data->flash_config, address - 0x400u, byte_count);
        return true;
    }
    if (data->bus.read == NULL)
        return false;
    return data->bus.read(data->bus.context, address, byte_count, output_value);
}

static uint32_t flash_data_size(const KinetisData* data) {
    if (!data->flash_partitioned)
        return data->profile->flexnvm_size;
    switch (data->flash_data_ifr[0x3fcu]) {
    case 0x00u:
    case 0x0du:
    case 0x0fu:
        return 0x20000u;
    case 0x03u:
        return 0x18000u;
    case 0x04u:
    case 0x0cu:
        return 0x10000u;
    case 0x0bu:
        return 0x8000u;
    case 0x05u:
    case 0x08u:
        return 0u;
    default:
        return 0u;
    }
}

static bool flash_memory_range(const KinetisData* data, uint32_t address, uint32_t range_size,
                               bool* data_flash_access, uint32_t* flash_offset) {
    *data_flash_access = (address & 0x800000u) != 0u;
    *flash_offset = address & 0x7fffffu;
    const uint32_t available_size =
        *data_flash_access ? flash_data_size(data) : data->profile->program_flash_size;
    return range_size <= available_size && *flash_offset <= available_size - range_size;
}

static bool flash_memory_load(KinetisData* data, uint32_t address, uint8_t byte_count,
                              uint32_t* output_value) {
    const bool data_flash_access = (address & 0x800000u) != 0u;
    const uint32_t flash_offset = address & 0x7fffffu;
    if (!data_flash_access)
        return flash_load(data, flash_offset, byte_count, output_value);
    *output_value = kinetis_data_internal_load_bytes(data->flexnvm, flash_offset, byte_count);
    return true;
}

static bool flash_memory_store(KinetisData* data, uint32_t address, uint8_t byte_count,
                               uint32_t write_value) {
    const bool data_flash_access = (address & 0x800000u) != 0u;
    const uint32_t flash_offset = address & 0x7fffffu;
    if (!data_flash_access)
        return flash_store(data, flash_offset, byte_count, write_value);
    kinetis_data_internal_store_bytes(data->flexnvm, flash_offset, byte_count, write_value);
    return true;
}

static bool flash_memory_range_protected(const KinetisData* data, uint32_t address,
                                         uint32_t length) {
    const bool data_flash = (address & 0x800000u) != 0u;
    const uint32_t offset = address & 0x7fffffu;
    const uint32_t region_count = data_flash ? 8u : 32u;
    const uint32_t memory_size =
        data_flash ? flash_data_size(data) : data->profile->program_flash_size;
    const uint32_t first = ((uint64_t)offset * region_count) / memory_size;
    const uint32_t last = ((uint64_t)(offset + length - 1u) * region_count) / memory_size;
    for (uint32_t region = first; region <= last; region++) {
        const uint8_t protection = data->flash[data_flash ? 0x17u : 0x10u + region / 8u];
        if ((protection & (1u << (region & 7u))) == 0u)
            return true;
    }
    return false;
}

static bool flash_program_words(KinetisData* data, uint32_t address, uint8_t words,
                                bool* verify_failure) {
    for (uint8_t word_index = 0; word_index < words; word_index++) {
        const uint8_t offset = (uint8_t)(4u + word_index * 4u);
        const uint32_t program_word = (uint32_t)flash_fccob(data, offset) |
                                      ((uint32_t)flash_fccob(data, offset + 1u) << 8u) |
                                      ((uint32_t)flash_fccob(data, offset + 2u) << 16u) |
                                      ((uint32_t)flash_fccob(data, offset + 3u) << 24u);
        uint32_t existing_word = 0;
        if (!flash_memory_load(data, address + (uint32_t)word_index * 4u, 4u, &existing_word))
            return false;
        if (existing_word != UINT32_MAX) {
            *verify_failure = true;
            return true;
        }
        if (!flash_memory_store(data, address + (uint32_t)word_index * 4u, 4u, program_word))
            return false;
    }
    return true;
}

static bool flash_program_buffer(KinetisData* data, uint32_t address, uint32_t length,
                                 bool* verify_failure) {
    for (uint32_t offset = 0; offset < length; offset += 4u) {
        uint32_t existing_word = 0;
        if (!flash_memory_load(data, address + offset, 4u, &existing_word))
            return false;
        if (existing_word != UINT32_MAX) {
            *verify_failure = true;
            return true;
        }
        const uint32_t program_word = kinetis_data_internal_load_bytes(data->flexram, offset, 4u);
        if (!flash_memory_store(data, address + offset, 4u, program_word))
            return false;
    }
    return true;
}

static bool flash_erase(KinetisData* data, uint32_t start, uint32_t length) {
    for (uint32_t offset = 0; offset < length; offset += 4u) {
        if (!flash_memory_store(data, start + offset, 4u, UINT32_MAX))
            return false;
    }
    return true;
}

static bool flash_range_erased(KinetisData* data, uint32_t start, uint32_t length) {
    for (uint32_t offset = 0; offset < length; offset += 4u) {
        uint32_t stored_word = 0;
        if (!flash_memory_load(data, start + offset, 4u, &stored_word) || stored_word != UINT32_MAX)
            return false;
    }
    return true;
}

static uint32_t flash_block_size(const KinetisData* data) {
    if (data->profile->program_flash_size >= 0x80000u)
        return data->profile->program_flash_size == 0x100000u ? 0x80000u : 0x40000u;
    return data->profile->program_flash_size;
}

static bool flash_block_range(const KinetisData* data, uint32_t address, uint32_t* start,
                              uint32_t* length, bool* data_flash) {
    uint32_t offset = 0u;
    if (!flash_memory_range(data, address, 1u, data_flash, &offset))
        return false;
    *length = *data_flash ? flash_data_size(data) : flash_block_size(data);
    *start = *data_flash ? 0x800000u : address & ~(*length - 1u);
    return true;
}

static bool flash_read_resource(KinetisData* data) {
    const bool ftfe = kinetis_profile_has_peripheral(data->profile, KINETIS_PERIPHERAL_FTFE);
    const uint32_t address = flash_address(data);
    const uint8_t option = flash_fccob(data, ftfe ? 4u : 8u);
    const uint8_t resource_size = ftfe ? 8u : 4u;
    const uint8_t* resource_bytes = NULL;
    uint32_t resource_offset = address;
    if (option == 0u && address < (ftfe ? 1024u : 256u))
        resource_bytes = data->flash_program_ifr;
    else if (option == 0u && ftfe && data->profile->flexnvm_size != 0u && address >= 0x800000u &&
             address - 0x800000u <= 1024u - resource_size) {
        resource_bytes = data->flash_data_ifr;
        resource_offset = address - 0x800000u;
    } else if (option == 1u && address == (ftfe ? 8u : 0u)) {
        static const uint8_t ftfa_version[8] = {0x01u, 0x00u, 0x46u, 0x54u,
                                                0x46u, 0x41u, 0x00u, 0x00u};
        static const uint8_t ftfe_version[8] = {0x01u, 0x00u, 0x46u, 0x54u,
                                                0x46u, 0x45u, 0x00u, 0x00u};
        resource_bytes = ftfe ? ftfe_version : ftfa_version;
        resource_offset = 0u;
    }
    if (resource_bytes == NULL || resource_offset > 1024u - resource_size ||
        (address & (resource_size - 1u)) != 0u)
        return false;
    for (uint8_t byte_index = 0u; byte_index < resource_size; byte_index++)
        flash_set_fccob(data, (uint8_t)(4u + byte_index),
                        resource_bytes[resource_offset + byte_index]);
    return true;
}

static bool flash_once_record(const KinetisData* data, uint8_t record_index,
                              uint32_t* record_offset, uint8_t* record_size) {
    if (kinetis_profile_has_peripheral(data->profile, KINETIS_PERIPHERAL_FTFE)) {
        if (record_index > 7u)
            return false;
        *record_offset = 0x3c0u + (uint32_t)record_index * 8u;
        *record_size = 8u;
        return true;
    }
    if (record_index <= 7u) {
        *record_offset = 0xc0u + (uint32_t)record_index * 4u;
        *record_size = 4u;
        return true;
    }
    if (record_index >= 0x10u && record_index <= 0x13u) {
        *record_offset = 0xe0u + (uint32_t)(record_index - 0x10u) * 8u;
        *record_size = 8u;
        return true;
    }
    return false;
}

static bool flash_read_once(KinetisData* data) {
    uint32_t record_offset = 0u;
    uint8_t record_size = 0u;
    if (!flash_once_record(data, flash_fccob(data, 1u), &record_offset, &record_size))
        return false;
    for (uint8_t byte_index = 0u; byte_index < record_size; byte_index++)
        flash_set_fccob(data, (uint8_t)(4u + byte_index),
                        data->flash_program_ifr[record_offset + byte_index]);
    return true;
}

static bool flash_program_once(KinetisData* data, bool* verify_failure) {
    uint32_t record_offset = 0u;
    uint8_t record_size = 0u;
    if (!flash_once_record(data, flash_fccob(data, 1u), &record_offset, &record_size))
        return false;
    for (uint8_t byte_index = 0u; byte_index < record_size; byte_index++) {
        if (data->flash_program_ifr[record_offset + byte_index] != 0xffu)
            return false;
    }
    for (uint8_t byte_index = 0u; byte_index < record_size; byte_index++)
        data->flash_program_ifr[record_offset + byte_index] =
            flash_fccob(data, (uint8_t)(4u + byte_index));
    for (uint8_t byte_index = 0u; byte_index < record_size; byte_index++)
        *verify_failure |= data->flash_program_ifr[record_offset + byte_index] !=
                           flash_fccob(data, (uint8_t)(4u + byte_index));
    return true;
}

static bool flash_verify_key(KinetisData* data) {
    if (data->flash_key_blocked || (data->flash[2] & 0xc0u) != 0x80u)
        return false;
    uint8_t aggregate_and = 0xffu;
    uint8_t aggregate_or = 0u;
    bool matches = true;
    for (uint8_t key_index = 0u; key_index < 8u; key_index++) {
        const uint8_t key_byte = flash_fccob(data, (uint8_t)(4u + key_index));
        aggregate_and &= key_byte;
        aggregate_or |= key_byte;
        matches &= key_byte == data->flash_config[key_index];
    }
    if (aggregate_or == 0u || aggregate_and == 0xffu)
        matches = false;
    if (matches)
        data->flash[2] = (uint8_t)((data->flash[2] & 0xfcu) | 0x02u);
    else
        data->flash_key_blocked = true;
    return matches;
}

static bool flash_program_partition(KinetisData* data) {
    static const uint8_t valid_partition_codes[] = {0x00u, 0x03u, 0x04u, 0x05u, 0x08u,
                                                    0x0bu, 0x0cu, 0x0du, 0x0fu};
    if (data->profile->flexnvm_size == 0u || data->flash_partitioned)
        return false;
    const uint8_t load_code = flash_fccob(data, 3u);
    const uint8_t eeprom_size_code = flash_fccob(data, 4u);
    const uint8_t partition_code = flash_fccob(data, 5u);
    bool valid_partition_code = false;
    for (size_t code_index = 0u; code_index < sizeof(valid_partition_codes); code_index++)
        valid_partition_code |= partition_code == valid_partition_codes[code_index];
    const bool no_eeprom =
        partition_code == 0x00u || partition_code == 0x0du || partition_code == 0x0fu;
    const bool eeprom_disabled = eeprom_size_code == 0x0fu;
    if (load_code > 1u || (eeprom_size_code & 0xc0u) != 0u || (partition_code & 0xf0u) != 0u ||
        !valid_partition_code || (eeprom_size_code < 2u && !eeprom_disabled) ||
        (eeprom_size_code > 9u && !eeprom_disabled) || no_eeprom != eeprom_disabled)
        return false;
    memset(data->flexnvm, 0xff, data->profile->flexnvm_size);
    data->flash_data_ifr[0x3fcu] = partition_code;
    data->flash_data_ifr[0x3fdu] = eeprom_size_code;
    data->flash_partitioned = true;
    data->flexram_eeprom = !no_eeprom;
    memset(data->eeprom, 0xff, data->profile->flexram_size);
    memset(data->flexram, 0xff, data->profile->flexram_size);
    data->flash[1] = (uint8_t)((data->flash[1] & 0xfcu) | (data->flexram_eeprom ? 0x01u : 0x02u));
    return true;
}

static bool flash_set_flexram(KinetisData* data) {
    if (data->flexram == NULL)
        return false;
    const uint8_t control = flash_fccob(data, 1u);
    if (control != 0u && control != 0xffu)
        return false;
    if (control == 0u && !data->flash_partitioned)
        return false;
    data->flexram_eeprom = control == 0u;
    if (data->flexram_eeprom)
        memcpy(data->flexram, data->eeprom, data->profile->flexram_size);
    else
        memset(data->flexram, 0xff, data->profile->flexram_size);
    data->flash[1] = (uint8_t)((data->flash[1] & 0xfcu) | (data->flexram_eeprom ? 0x01u : 0x02u));
    return true;
}

static bool flash_swap_address_valid(const KinetisData* data, uint32_t address) {
    const uint32_t swap_block_size = data->profile->program_flash_size / 2u;
    return (address & 0x0fu) == 0u && address < swap_block_size &&
           (address < 0x400u || address >= 0x410u);
}

static bool flash_swap_program_indicator(KinetisData* data, uint8_t block_index,
                                         uint16_t indicator_value) {
    const uint32_t logical_flash_address =
        data->flash_swap_address + ((uint32_t)(block_index ^ data->flash_swap_current_block) *
                                    (data->profile->program_flash_size / 2u));
    return flash_memory_store(data, logical_flash_address, 2u, indicator_value);
}

static bool flash_swap_control(KinetisData* data, uint32_t address, bool* verify_failure) {
    const uint8_t control = flash_fccob(data, 4u);
    if (!flash_swap_address_valid(data, address))
        return false;
    if (control == 0x08u) {
        flash_set_fccob(data, 5u, data->flash_swap_mode);
        flash_set_fccob(data, 6u, data->flash_swap_current_block);
        flash_set_fccob(data, 7u, data->flash_swap_next_block);
        return true;
    }
    if (control == 0x01u) {
        if (data->flash_swap_mode != 0u)
            return false;
        data->flash_swap_address = address;
        if (!flash_swap_program_indicator(data, 0u, 0xff00u)) {
            *verify_failure = true;
            return true;
        }
        data->flash_swap_mode = 3u;
        data->flash_swap_current_block = 0u;
        data->flash_swap_next_block = 0u;
        return true;
    }
    if (address != data->flash_swap_address)
        return false;
    if (control == 0x02u) {
        if (data->flash_swap_mode != 1u)
            return false;
        if (!flash_swap_program_indicator(data, data->flash_swap_current_block, 0xff00u)) {
            *verify_failure = true;
            return true;
        }
        data->flash_swap_mode = 2u;
        return true;
    }
    if (control == 0x04u) {
        if (data->flash_swap_mode != 3u)
            return false;
        if (!flash_swap_program_indicator(data, data->flash_swap_current_block, 0u)) {
            *verify_failure = true;
            return true;
        }
        data->flash_swap_mode = 4u;
        data->flash_swap_next_block = data->flash_swap_current_block ^ 1u;
        return true;
    }
    return false;
}

static void flash_swap_erased(KinetisData* data, uint32_t range_start, uint32_t range_size) {
    if (data->flash_swap_mode != 2u)
        return;
    const uint32_t swap_block_size = data->profile->program_flash_size / 2u;
    const uint32_t nonactive_address = data->flash_swap_address + swap_block_size;
    if (range_start <= nonactive_address && nonactive_address - range_start < range_size)
        data->flash_swap_mode = 3u;
}

static bool flash_swap_range_protected(const KinetisData* data, uint32_t range_start,
                                       uint32_t range_size, bool erase_operation) {
    if (data->flash_swap_mode == 0u || range_size == 0u)
        return false;
    const uint32_t swap_block_size = data->profile->program_flash_size / 2u;
    const uint32_t active_address = data->flash_swap_address;
    const uint32_t nonactive_address = active_address + swap_block_size;
    const bool active_overlap =
        range_start <= active_address && active_address - range_start < range_size;
    const bool nonactive_overlap =
        range_start <= nonactive_address && nonactive_address - range_start < range_size;
    if (active_overlap)
        return true;
    return nonactive_overlap &&
           (!erase_operation || (data->flash_swap_mode != 2u && data->flash_swap_mode != 3u));
}

static uint8_t flash_busy_banks(uint8_t command, uint32_t address) {
    switch (command) {
    case 0x00u:
    case 0x01u:
    case 0x02u:
    case 0x03u:
    case 0x06u:
    case 0x07u:
    case 0x08u:
    case 0x09u:
    case 0x0bu:
        return (address & 0x800000u) != 0u ? 2u : 1u;
    case 0x40u:
    case 0x44u:
        return 3u;
    case 0x41u:
    case 0x43u:
    case 0x45u:
    case 0x46u:
        return 1u;
    default:
        return 2u;
    }
}

void kinetis_data_internal_flash_update_interrupts(KinetisData* data) {
    kinetis_data_internal_interrupt(data, KINETIS_DATA_INTERRUPT_FTFA,
                                    (data->flash[1] & 0x80u) != 0u &&
                                        (data->flash[0] & 0x80u) != 0u);
    kinetis_data_internal_interrupt(data, KINETIS_DATA_INTERRUPT_FLASH_COLLISION,
                                    (data->flash[1] & 0x40u) != 0u &&
                                        (data->flash[0] & 0x40u) != 0u);
}

static void flash_execute(KinetisData* data) {
    data->flash[0] &= 0x70u;
    const uint8_t command = flash_fccob(data, 0u);
    const uint32_t address = flash_address(data);
    const bool ftfe = kinetis_profile_has_peripheral(data->profile, KINETIS_PERIPHERAL_FTFE);
    const uint32_t sector_size = ftfe ? 4096u : 2048u;
    const uint8_t program_command = ftfe ? 0x07u : 0x06u;
    const uint8_t program_words = ftfe ? 2u : 1u;
    bool valid = true;
    bool protection_failure = false;
    bool verify_failure = false;
    if (command == program_command) {
        bool data_flash = false;
        uint32_t offset = 0;
        valid = address % (program_words * 4u) == 0u &&
                flash_memory_range(data, address, program_words * 4u, &data_flash, &offset);
        protection_failure =
            valid && (flash_memory_range_protected(data, address, program_words * 4u) ||
                      flash_swap_range_protected(data, address, program_words * 4u, false));
        if (valid && !protection_failure)
            valid = flash_program_words(data, address, program_words, &verify_failure);
    } else if (command == 0x09u) {
        bool data_flash = false;
        uint32_t offset = 0;
        const uint32_t start = address & ~(sector_size - 1u);
        valid = (address & 0x0fu) == 0u &&
                flash_memory_range(data, start, sector_size, &data_flash, &offset);
        protection_failure = valid && (flash_memory_range_protected(data, start, sector_size) ||
                                       flash_swap_range_protected(data, start, sector_size, true));
        if (valid && !protection_failure)
            valid = flash_erase(data, start, sector_size);
        if (valid && !protection_failure)
            flash_swap_erased(data, start, sector_size);
    } else if (command == 0x08u) {
        bool data_flash = false;
        uint32_t block_size = 0u;
        uint32_t start = 0u;
        valid = (address & 0x0fu) == 0u &&
                flash_block_range(data, address, &start, &block_size, &data_flash) &&
                (ftfe || data->profile->program_flash_size == 0x80000u) &&
                !(data_flash && data->flexram_eeprom);
        protection_failure = valid && (flash_memory_range_protected(data, start, block_size) ||
                                       flash_swap_range_protected(data, start, block_size, true));
        if (valid && !protection_failure)
            valid = flash_erase(data, start, block_size);
        if (valid && !protection_failure)
            flash_swap_erased(data, start, block_size);
    } else if (command == 0x44u) {
        protection_failure =
            flash_memory_range_protected(data, 0, data->profile->program_flash_size);
        if (!protection_failure && data->profile->flexnvm_size != 0u)
            protection_failure = data->flash[0x17u] != 0xffu;
        if (!protection_failure) {
            valid = flash_erase(data, 0, data->profile->program_flash_size);
            if (valid && data->profile->flexnvm_size != 0u)
                memset(data->flexnvm, 0xff, data->profile->flexnvm_size);
            if (valid && data->flexram != NULL)
                memset(data->flexram, 0xff, data->profile->flexram_size);
            if (valid && data->eeprom != NULL)
                memset(data->eeprom, 0xff, data->profile->flexram_size);
            if (valid) {
                memset(data->flash_config, 0xff, sizeof(data->flash_config));
                memset(data->flash_data_ifr, 0xff, sizeof(data->flash_data_ifr));
                data->flash_partitioned = false;
                data->flexram_eeprom = false;
                data->flash[1] =
                    (uint8_t)((data->flash[1] & 0xfcu) | (data->flexram != NULL ? 0x02u : 0u));
                data->flash[2] = 0xfeu;
                data->flash_swap_address = 0u;
                data->flash_swap_mode = 0u;
                data->flash_swap_current_block = 0u;
                data->flash_swap_next_block = 0u;
            }
        }
    } else if (command == 0x00u) {
        bool data_flash = false;
        const uint8_t margin = flash_fccob(data, 4u);
        uint32_t block_size = 0u;
        uint32_t start = 0u;
        valid = margin <= 2u && (address & 0x0fu) == 0u &&
                flash_block_range(data, address, &start, &block_size, &data_flash) &&
                (ftfe || data->profile->program_flash_size == 0x80000u) &&
                !(data_flash && data->flexram_eeprom);
        verify_failure = valid && !flash_range_erased(data, start, block_size);
    } else if (command == 0x01u) {
        const uint32_t count = ((uint32_t)flash_fccob(data, 4u) << 8u) | flash_fccob(data, 5u);
        const uint8_t margin = flash_fccob(data, 6u);
        const uint32_t length = count * 16u;
        bool data_flash = false;
        uint32_t offset = 0;
        valid = margin <= 2u && count != 0u && (address & 0x0fu) == 0u &&
                length <= sector_size - (address & (sector_size - 1u)) &&
                flash_memory_range(data, address, length, &data_flash, &offset);
        verify_failure = valid && !flash_range_erased(data, address, length);
    } else if (command == 0x40u) {
        const uint8_t margin = flash_fccob(data, 1u);
        valid = margin <= 2u;
        verify_failure = valid && !flash_range_erased(data, 0, data->profile->program_flash_size);
        if (valid && !verify_failure && data->profile->flexnvm_size != 0u)
            verify_failure = !flash_range_erased(data, 0x800000u, flash_data_size(data));
        if (valid && !verify_failure)
            data->flash[2] = (uint8_t)((data->flash[2] & 0xfcu) | 0x02u);
    } else if (command == 0x02u) {
        uint32_t actual = 0;
        const uint8_t margin = flash_fccob(data, 4u);
        const uint32_t expected =
            (uint32_t)flash_fccob(data, 8u) | ((uint32_t)flash_fccob(data, 9u) << 8u) |
            ((uint32_t)flash_fccob(data, 10u) << 16u) | ((uint32_t)flash_fccob(data, 11u) << 24u);
        bool data_flash = false;
        uint32_t offset = 0;
        valid = margin >= 1u && margin <= 2u && (address & 3u) == 0u &&
                flash_memory_range(data, address, 4u, &data_flash, &offset);
        verify_failure =
            valid && (!flash_memory_load(data, address, 4u, &actual) || actual != expected);
    } else if (command == 0x03u) {
        valid = flash_read_resource(data);
    } else if (command == 0x41u) {
        valid = flash_read_once(data);
    } else if (command == 0x43u) {
        valid = flash_program_once(data, &verify_failure);
    } else if (command == 0x0bu) {
        const uint32_t count = ((uint32_t)flash_fccob(data, 4u) << 8u) | flash_fccob(data, 5u);
        const uint32_t length = count * 16u;
        bool data_flash = false;
        uint32_t offset = 0;
        valid = ftfe && data->flexram != NULL && (data->flash[1] & 0x02u) != 0u && count != 0u &&
                length <= 1024u && (address & 0x0fu) == 0u &&
                length <= sector_size - (address & (sector_size - 1u)) &&
                flash_memory_range(data, address, length, &data_flash, &offset);
        protection_failure = valid && (flash_memory_range_protected(data, address, length) ||
                                       flash_swap_range_protected(data, address, length, false));
        if (valid && !protection_failure)
            valid = flash_program_buffer(data, address, length, &verify_failure);
    } else if (command == 0x45u) {
        valid = flash_verify_key(data);
    } else if (command == 0x80u) {
        valid = ftfe && flash_program_partition(data);
    } else if (command == 0x81u) {
        valid = ftfe && flash_set_flexram(data);
    } else if (command == 0x46u) {
        valid = ftfe && data->profile->flexnvm_size == 0u;
        if (valid)
            valid = flash_swap_control(data, address, &verify_failure);
    } else {
        valid = false;
    }
    if (protection_failure)
        data->flash[0] |= 0x10u;
    else if (!valid)
        data->flash[0] |= 0x20u;
    else if (verify_failure)
        data->flash[0] |= 1u;
    data->flash_cycles =
        command == 0x08u || command == 0x09u || command == 0x44u || command == 0x80u ? 2000u : 40u;
    data->flash_busy_banks = valid && !protection_failure ? flash_busy_banks(command, address) : 0u;
    data->flash_busy_start = 0u;
    data->flash_busy_length = 0u;
    if (data->flash_busy_banks == 1u) {
        data->flash_busy_length = flash_block_size(data);
        data->flash_busy_start = address & ~(data->flash_busy_length - 1u);
    } else if (data->flash_busy_banks == 2u) {
        data->flash_busy_length = flash_data_size(data);
    } else if (data->flash_busy_banks == 3u) {
        data->flash_busy_length = UINT32_MAX;
    }
    kinetis_data_internal_flash_update_interrupts(data);
}

bool kinetis_data_internal_flash_read(KinetisData* data, uint32_t address, uint8_t byte_count,
                                      uint32_t* output_value) {
    const uint32_t register_offset = address - FLASH_BASE;
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, sizeof(data->flash)))
        return false;
    const bool ftfe = kinetis_profile_has_peripheral(data->profile, KINETIS_PERIPHERAL_FTFE);
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
        const uint32_t register_address = register_offset + byte_index;
        const bool common_register = register_address <= 0x13u;
        const bool extension_register =
            ftfe ? register_address >= 0x16u && register_address <= 0x17u
                 : (register_address >= 0x18u && register_address <= 0x28u) ||
                       register_address == 0x2bu;
        if (!common_register && !extension_register)
            return false;
    }
    *output_value = kinetis_data_internal_load_bytes(data->flash, register_offset, byte_count);
    return true;
}

bool kinetis_data_internal_flash_write(KinetisData* data, uint32_t address, uint8_t byte_count,
                                       uint32_t write_value) {
    const uint32_t register_offset = address - FLASH_BASE;
    if (!kinetis_data_internal_valid_access(register_offset, byte_count, sizeof(data->flash)))
        return false;
    if (register_offset == 0u && byte_count == 1u) {
        data->flash[0] &= (uint8_t)~(write_value & 0x70u);
        if ((write_value & 0x80u) != 0u && (data->flash[0] & 0xb0u) == 0x80u)
            flash_execute(data);
        else
            kinetis_data_internal_flash_update_interrupts(data);
        return true;
    }
    if (register_offset == 1u && byte_count == 1u) {
        data->flash[1] = (uint8_t)((data->flash[1] & 0x2fu) | (write_value & 0xd0u));
        kinetis_data_internal_flash_update_interrupts(data);
        return true;
    }
    if (register_offset >= 4u && register_offset <= 0x10u - byte_count) {
        if ((data->flash[0] & 0x80u) != 0u)
            kinetis_data_internal_store_bytes(data->flash, register_offset, byte_count,
                                              write_value);
        return true;
    }
    if (register_offset >= 0x10u && register_offset <= 0x14u - byte_count) {
        if ((data->flash[0] & 0x80u) == 0u)
            return true;
        for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++)
            data->flash[register_offset + byte_index] &=
                (uint8_t)(write_value >> (byte_index * 8u));
        return true;
    }
    if (kinetis_profile_has_peripheral(data->profile, KINETIS_PERIPHERAL_FTFE) &&
        (register_offset == 0x16u || register_offset == 0x17u) && byte_count == 1u) {
        if ((data->flash[0] & 0x80u) != 0u)
            data->flash[register_offset] &= (uint8_t)write_value;
        return true;
    }
    return false;
}
