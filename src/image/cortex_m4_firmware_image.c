#include "cortex_m4_firmware_image.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    ELF_HEADER_SIZE = 52,
    ELF_PROGRAM_HEADER_SIZE = 32,
    ELF_LOAD_SEGMENT = 1,
};

static uint16_t read_u16(const uint8_t* bytes) {
    return (uint16_t)(bytes[0] | (uint16_t)bytes[1] << 8);
}

static uint32_t read_u32(const uint8_t* bytes) {
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 |
           (uint32_t)bytes[3] << 24;
}

static bool load_file(const char* path, uint8_t** file_data, size_t* file_size) {
    FILE* file = fopen(path, "rb");
    if (file == NULL) {
        return false;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return false;
    }
    const long file_length = ftell(file);
    if (file_length <= 0 || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return false;
    }
    uint8_t* file_bytes = malloc((size_t)file_length);
    if (file_bytes == NULL ||
        fread(file_bytes, 1u, (size_t)file_length, file) != (size_t)file_length) {
        free(file_bytes);
        fclose(file);
        return false;
    }
    fclose(file);
    *file_data = file_bytes;
    *file_size = (size_t)file_length;
    return true;
}

bool cortex_m4_load_elf_data(KinetisK22* device, const void* image_data, size_t image_size,
                             uint32_t* entry_address) {
    if (device == NULL || image_data == NULL) {
        return false;
    }
    const uint8_t* bytes = image_data;
    bool loaded = image_size >= ELF_HEADER_SIZE && bytes[0] == 0x7f && bytes[1] == 'E' &&
                  bytes[2] == 'L' && bytes[3] == 'F' && bytes[4] == 1 && bytes[5] == 1 &&
                  read_u16(bytes + 18) == 40;
    if (!loaded) {
        return false;
    }
    const uint32_t program_header_offset = read_u32(bytes + 28);
    const uint16_t program_header_size = read_u16(bytes + 42);
    const uint16_t program_header_count = read_u16(bytes + 44);
    loaded =
        program_header_size >= ELF_PROGRAM_HEADER_SIZE &&
        (uint64_t)program_header_offset + (uint64_t)program_header_size * program_header_count <=
            image_size;
    for (uint16_t header_index = 0; loaded && header_index < program_header_count; header_index++) {
        const uint8_t* program_header =
            bytes + program_header_offset + (size_t)header_index * program_header_size;
        if (read_u32(program_header) != ELF_LOAD_SEGMENT) {
            continue;
        }
        const uint32_t file_offset = read_u32(program_header + 4);
        const uint32_t virtual_address = read_u32(program_header + 8);
        const uint32_t physical_address = read_u32(program_header + 12);
        const uint32_t file_size = read_u32(program_header + 16);
        const uint32_t memory_size = read_u32(program_header + 20);
        const uint32_t load_address = physical_address != 0 ? physical_address : virtual_address;
        if ((uint64_t)file_offset + file_size > image_size || file_size > memory_size ||
            !kinetis_k22_load(device, load_address, bytes + file_offset, file_size)) {
            loaded = false;
            break;
        }
        if (memory_size > file_size) {
            uint8_t zeros[256] = {0};
            uint32_t remaining = memory_size - file_size;
            uint32_t zero_address = load_address + file_size;
            while (loaded && remaining != 0) {
                const size_t chunk = remaining < sizeof(zeros) ? remaining : sizeof(zeros);
                loaded = kinetis_k22_load(device, zero_address, zeros, chunk);
                zero_address += (uint32_t)chunk;
                remaining -= (uint32_t)chunk;
            }
        }
    }
    if (loaded && entry_address != NULL) {
        *entry_address = read_u32(bytes + 24);
    }
    return loaded;
}

bool cortex_m4_load_binary_data(KinetisK22* device, const void* image_data, size_t image_size,
                                uint32_t load_address, uint32_t* entry_address) {
    if (device == NULL || image_data == NULL || image_size == 0u ||
        !kinetis_k22_load(device, load_address, image_data, image_size)) {
        return false;
    }
    if (entry_address != NULL) {
        *entry_address = load_address;
    }
    return true;
}

static bool range_valid(size_t image_size, uint32_t file_offset, uint32_t byte_count) {
    return file_offset <= image_size && byte_count <= image_size - file_offset;
}

bool cortex_m4_elf_symbol_data(const void* image_data, size_t image_size,
                               const char* requested_name, uint32_t* symbol_address) {
    if (image_data == NULL || requested_name == NULL || symbol_address == NULL ||
        image_size < ELF_HEADER_SIZE) {
        return false;
    }
    const uint8_t* bytes = image_data;
    if (bytes[0] != 0x7fu || bytes[1] != 'E' || bytes[2] != 'L' || bytes[3] != 'F' ||
        bytes[4] != 1u || bytes[5] != 1u) {
        return false;
    }
    const uint32_t section_table_offset = read_u32(bytes + 32u);
    const uint16_t section_entry_size = read_u16(bytes + 46u);
    const uint16_t section_count = read_u16(bytes + 48u);
    if (section_entry_size < 40u ||
        (uint64_t)section_table_offset + (uint64_t)section_entry_size * section_count >
            image_size) {
        return false;
    }
    for (uint16_t section_index = 0u; section_index < section_count; ++section_index) {
        const uint8_t* section_header =
            bytes + section_table_offset + (uint32_t)section_index * section_entry_size;
        const uint32_t section_type = read_u32(section_header + 4u);
        if (section_type != 2u && section_type != 11u) {
            continue;
        }
        const uint32_t symbol_table_offset = read_u32(section_header + 16u);
        const uint32_t symbol_table_size = read_u32(section_header + 20u);
        const uint32_t string_table_index = read_u32(section_header + 24u);
        const uint32_t symbol_entry_size = read_u32(section_header + 36u);
        if (symbol_entry_size < 16u || symbol_entry_size > symbol_table_size ||
            string_table_index >= section_count ||
            !range_valid(image_size, symbol_table_offset, symbol_table_size)) {
            continue;
        }
        const uint8_t* string_table_header =
            bytes + section_table_offset + string_table_index * section_entry_size;
        const uint32_t string_table_offset = read_u32(string_table_header + 16u);
        const uint32_t string_table_size = read_u32(string_table_header + 20u);
        if (!range_valid(image_size, string_table_offset, string_table_size)) {
            continue;
        }
        for (uint32_t symbol_offset = 0u; symbol_offset <= symbol_table_size - symbol_entry_size;
             symbol_offset += symbol_entry_size) {
            const uint8_t* symbol_entry = bytes + symbol_table_offset + symbol_offset;
            const uint32_t string_offset = read_u32(symbol_entry);
            if (string_offset >= string_table_size) {
                continue;
            }
            const char* symbol_name = (const char*)bytes + string_table_offset + string_offset;
            const size_t string_bytes_remaining = string_table_size - string_offset;
            if (memchr(symbol_name, '\0', string_bytes_remaining) != NULL &&
                strcmp(symbol_name, requested_name) == 0) {
                *symbol_address = read_u32(symbol_entry + 4u);
                return true;
            }
        }
    }
    return false;
}

bool cortex_m4_load_elf(KinetisK22* device, const char* path, uint32_t* entry_address) {
    uint8_t* image_data = NULL;
    size_t image_size = 0;
    if (!load_file(path, &image_data, &image_size)) {
        return false;
    }
    const bool loaded = cortex_m4_load_elf_data(device, image_data, image_size, entry_address);
    free(image_data);
    return loaded;
}

bool cortex_m4_load_binary(KinetisK22* device, const char* path, uint32_t load_address) {
    uint8_t* image_data = NULL;
    size_t image_size = 0;
    if (!load_file(path, &image_data, &image_size)) {
        return false;
    }
    const bool loaded =
        cortex_m4_load_binary_data(device, image_data, image_size, load_address, NULL);
    free(image_data);
    return loaded;
}
