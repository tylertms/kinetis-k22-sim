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
    const long length = ftell(file);
    if (length <= 0 || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return false;
    }
    uint8_t* bytes = malloc((size_t)length);
    if (bytes == NULL || fread(bytes, 1, (size_t)length, file) != (size_t)length) {
        free(bytes);
        fclose(file);
        return false;
    }
    fclose(file);
    *file_data = bytes;
    *file_size = (size_t)length;
    return true;
}

bool cortex_m4_load_elf_data(KinetisK22* device, const void* image, size_t image_size,
                             uint32_t* entry_address) {
    if (device == NULL || image == NULL) {
        return false;
    }
    const uint8_t* image_data = image;
    bool loaded = image_size >= ELF_HEADER_SIZE && image_data[0] == 0x7f && image_data[1] == 'E' &&
                  image_data[2] == 'L' && image_data[3] == 'F' && image_data[4] == 1 &&
                  image_data[5] == 1 && read_u16(image_data + 18) == 40;
    if (!loaded) {
        return false;
    }
    const uint32_t header_offset = read_u32(image_data + 28);
    const uint16_t header_size = read_u16(image_data + 42);
    const uint16_t header_count = read_u16(image_data + 44);
    loaded = header_size >= ELF_PROGRAM_HEADER_SIZE &&
             (uint64_t)header_offset + (uint64_t)header_size * header_count <= image_size;
    for (uint16_t header_index = 0; loaded && header_index < header_count; header_index++) {
        const uint8_t* header = image_data + header_offset + (size_t)header_index * header_size;
        if (read_u32(header) != ELF_LOAD_SEGMENT) {
            continue;
        }
        const uint32_t file_offset = read_u32(header + 4);
        const uint32_t virtual_address = read_u32(header + 8);
        const uint32_t physical_address = read_u32(header + 12);
        const uint32_t file_size = read_u32(header + 16);
        const uint32_t memory_size = read_u32(header + 20);
        const uint32_t address = physical_address != 0 ? physical_address : virtual_address;
        if ((uint64_t)file_offset + file_size > image_size || file_size > memory_size ||
            !kinetis_k22_load(device, address, image_data + file_offset, file_size)) {
            loaded = false;
            break;
        }
        if (memory_size > file_size) {
            uint8_t zeros[256] = {0};
            uint32_t remaining = memory_size - file_size;
            uint32_t zero_address = address + file_size;
            while (loaded && remaining != 0) {
                const size_t chunk = remaining < sizeof(zeros) ? remaining : sizeof(zeros);
                loaded = kinetis_k22_load(device, zero_address, zeros, chunk);
                zero_address += (uint32_t)chunk;
                remaining -= (uint32_t)chunk;
            }
        }
    }
    if (loaded && entry_address != NULL) {
        *entry_address = read_u32(image_data + 24);
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

static bool range_valid(size_t image_size, uint32_t offset, uint32_t length) {
    return offset <= image_size && length <= image_size - offset;
}

bool cortex_m4_elf_symbol_data(const void* image, size_t image_size, const char* name,
                               uint32_t* address) {
    if (image == NULL || name == NULL || address == NULL || image_size < ELF_HEADER_SIZE) {
        return false;
    }
    const uint8_t* image_data = image;
    if (image_data[0] != 0x7fu || image_data[1] != 'E' || image_data[2] != 'L' ||
        image_data[3] != 'F' || image_data[4] != 1u || image_data[5] != 1u) {
        return false;
    }
    const uint32_t section_offset = read_u32(image_data + 32u);
    const uint16_t section_size = read_u16(image_data + 46u);
    const uint16_t section_count = read_u16(image_data + 48u);
    if (section_size < 40u ||
        (uint64_t)section_offset + (uint64_t)section_size * section_count > image_size) {
        return false;
    }
    for (uint16_t section_index = 0u; section_index < section_count; ++section_index) {
        const uint8_t* section =
            image_data + section_offset + (uint32_t)section_index * section_size;
        const uint32_t type = read_u32(section + 4u);
        if (type != 2u && type != 11u) {
            continue;
        }
        const uint32_t symbols_offset = read_u32(section + 16u);
        const uint32_t symbols_size = read_u32(section + 20u);
        const uint32_t strings_index = read_u32(section + 24u);
        const uint32_t symbol_size = read_u32(section + 36u);
        if (symbol_size < 16u || symbol_size > symbols_size || strings_index >= section_count ||
            !range_valid(image_size, symbols_offset, symbols_size)) {
            continue;
        }
        const uint8_t* strings_section = image_data + section_offset + strings_index * section_size;
        const uint32_t strings_offset = read_u32(strings_section + 16u);
        const uint32_t strings_size = read_u32(strings_section + 20u);
        if (!range_valid(image_size, strings_offset, strings_size)) {
            continue;
        }
        for (uint32_t offset = 0u; offset <= symbols_size - symbol_size; offset += symbol_size) {
            const uint8_t* symbol = image_data + symbols_offset + offset;
            const uint32_t name_offset = read_u32(symbol);
            if (name_offset >= strings_size) {
                continue;
            }
            const char* symbol_name = (const char*)image_data + strings_offset + name_offset;
            const size_t available = strings_size - name_offset;
            if (memchr(symbol_name, '\0', available) != NULL && strcmp(symbol_name, name) == 0) {
                *address = read_u32(symbol + 4u);
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
