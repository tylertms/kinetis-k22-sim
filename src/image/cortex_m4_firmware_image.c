#include "cortex_m4_firmware_image.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    ELF_HEADER_SIZE = 52,
    ELF_PROGRAM_HEADER_SIZE = 32,
    ELF_SECTION_HEADER_SIZE = 40,
    ELF_SYMBOL_SIZE = 16,
    ELF_LOAD_SEGMENT = 1,
    ELF_SECTION_PROGBITS = 1,
    ELF_SECTION_SYMTAB = 2,
    ELF_SECTION_STRTAB = 3,
    ELF_SECTION_DYNSYM = 11,
    ELF_SECTION_EXECUTABLE = 4,
};

typedef struct {
    uint32_t type;
    uint32_t flags;
    uint32_t address;
    uint32_t file_offset;
    uint32_t size;
    uint32_t linked_section;
    uint32_t entry_size;
} ElfSection;

static uint16_t read_u16(const uint8_t* bytes) {
    return (uint16_t)(bytes[0] | (uint16_t)bytes[1] << 8);
}

static uint32_t read_u32(const uint8_t* bytes) {
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 |
           (uint32_t)bytes[3] << 24;
}

static bool range_valid(size_t image_size, uint32_t file_offset, uint32_t byte_count) {
    return file_offset <= image_size && byte_count <= image_size - file_offset;
}

static bool elf32_arm(const uint8_t* bytes, size_t image_size) {
    return image_size >= ELF_HEADER_SIZE && bytes[0] == 0x7fu && bytes[1] == 'E' &&
           bytes[2] == 'L' && bytes[3] == 'F' && bytes[4] == 1u && bytes[5] == 1u &&
           read_u16(bytes + 18u) == 40u;
}

static bool section_table(const uint8_t* bytes, size_t image_size, uint32_t* table_offset,
                          uint16_t* section_count) {
    if (!elf32_arm(bytes, image_size)) {
        return false;
    }
    *table_offset = read_u32(bytes + 32u);
    const uint16_t entry_size = read_u16(bytes + 46u);
    *section_count = read_u16(bytes + 48u);
    return entry_size >= ELF_SECTION_HEADER_SIZE &&
           (uint64_t)*table_offset + (uint64_t)entry_size * *section_count <= image_size;
}

static ElfSection read_section(const uint8_t* bytes, uint32_t table_offset, uint16_t entry_size,
                               uint16_t section_index) {
    const uint8_t* header = bytes + table_offset + (uint32_t)section_index * entry_size;
    return (ElfSection){
        .type = read_u32(header + 4u),
        .flags = read_u32(header + 8u),
        .address = read_u32(header + 12u),
        .file_offset = read_u32(header + 16u),
        .size = read_u32(header + 20u),
        .linked_section = read_u32(header + 24u),
        .entry_size = read_u32(header + 36u),
    };
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

bool cortex_m4_load_elf_data(Kinetis* device, const void* image_data, size_t image_size,
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
        if (file_size > memory_size ||
            (file_size != 0u &&
             ((uint64_t)file_offset + file_size > image_size ||
              !kinetis_load(device, load_address, bytes + file_offset, file_size)))) {
            loaded = false;
            break;
        }
        if (memory_size > file_size) {
            uint8_t zeros[256] = {0};
            uint32_t remaining = memory_size - file_size;
            uint32_t zero_address = virtual_address + file_size;
            while (loaded && remaining != 0) {
                const size_t chunk = remaining < sizeof(zeros) ? remaining : sizeof(zeros);
                loaded = kinetis_load(device, zero_address, zeros, chunk);
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

bool cortex_m4_load_binary_data(Kinetis* device, const void* image_data, size_t image_size,
                                uint32_t load_address, uint32_t* entry_address) {
    if (device == NULL || image_data == NULL || image_size == 0u ||
        !kinetis_load(device, load_address, image_data, image_size)) {
        return false;
    }
    if (entry_address != NULL) {
        *entry_address = load_address;
    }
    return true;
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

static bool executable_section(const ElfSection* section, size_t image_size) {
    return section->type == ELF_SECTION_PROGBITS &&
           (section->flags & ELF_SECTION_EXECUTABLE) != 0u && section->size != 0u &&
           (section->address & 1u) == 0u && (section->size & 1u) == 0u &&
           (uint64_t)section->address + section->size <= UINT64_C(0x100000000) &&
           range_valid(image_size, section->file_offset, section->size);
}

static bool mapping_symbol(const char* name, size_t available, uint8_t* mapping) {
    const char* terminator = memchr(name, '\0', available);
    if (available < 3u || terminator == NULL || name[0] != '$' ||
        (name[1] != 't' && name[1] != 'd') || (name[2] != '\0' && name[2] != '.')) {
        return false;
    }
    *mapping = name[1] == 't' ? 1u : 2u;
    return true;
}

static bool section_mappings(const uint8_t* bytes, size_t image_size, uint32_t table_offset,
                             uint16_t entry_size, uint16_t section_count,
                             uint16_t executable_section_index, const ElfSection* executable,
                             uint8_t* mappings) {
    for (uint16_t section_index = 0u; section_index < section_count; section_index++) {
        const ElfSection symbols = read_section(bytes, table_offset, entry_size, section_index);
        if ((symbols.type != ELF_SECTION_SYMTAB && symbols.type != ELF_SECTION_DYNSYM) ||
            symbols.entry_size < ELF_SYMBOL_SIZE || symbols.entry_size > symbols.size ||
            symbols.linked_section >= section_count ||
            !range_valid(image_size, symbols.file_offset, symbols.size)) {
            continue;
        }
        const ElfSection strings =
            read_section(bytes, table_offset, entry_size, (uint16_t)symbols.linked_section);
        if (strings.type != ELF_SECTION_STRTAB ||
            !range_valid(image_size, strings.file_offset, strings.size)) {
            continue;
        }
        for (uint32_t offset = 0u; offset <= symbols.size - symbols.entry_size;
             offset += symbols.entry_size) {
            const uint8_t* symbol = bytes + symbols.file_offset + offset;
            if (read_u16(symbol + 14u) != executable_section_index) {
                continue;
            }
            const uint32_t string_offset = read_u32(symbol);
            const uint32_t address = read_u32(symbol + 4u) & ~1u;
            if (string_offset >= strings.size || address < executable->address ||
                address >= executable->address + executable->size) {
                continue;
            }
            uint8_t mapping;
            if (mapping_symbol((const char*)bytes + strings.file_offset + string_offset,
                               strings.size - string_offset, &mapping)) {
                mappings[(address - executable->address) / 2u] = mapping;
            }
        }
    }
    return true;
}

static bool conditional_thumb16(uint16_t opcode) {
    return (opcode & 0xf500u) == 0xb100u ||
           ((opcode & 0xf000u) == 0xd000u && ((opcode >> 8u) & 15u) < 14u);
}

static bool conditional_thumb32(uint16_t first, uint16_t second) {
    return (first & 0xf800u) == 0xf000u && (second & 0xd000u) == 0x8000u &&
           ((first >> 6u) & 15u) < 14u;
}

static bool define_executable_section(CortexM4Coverage* coverage, const uint8_t* bytes,
                                      size_t image_size, uint32_t table_offset, uint16_t entry_size,
                                      uint16_t section_count, uint16_t section_index,
                                      const ElfSection* section) {
    const size_t slot_count = section->size / 2u;
    uint8_t* mappings = calloc(slot_count, 1u);
    if (mappings == NULL || !section_mappings(bytes, image_size, table_offset, entry_size,
                                              section_count, section_index, section, mappings)) {
        free(mappings);
        return false;
    }
    bool code = true;
    for (uint32_t offset = 0u; offset < section->size;) {
        const uint8_t mapping = mappings[offset / 2u];
        if (mapping != 0u) {
            code = mapping == 1u;
        }
        if (!code) {
            offset += 2u;
            continue;
        }
        const uint16_t first = read_u16(bytes + section->file_offset + offset);
        const bool wide = (first & 0xf800u) >= 0xe800u;
        if (wide && section->size - offset < 4u) {
            free(mappings);
            return false;
        }
        const uint16_t second = wide ? read_u16(bytes + section->file_offset + offset + 2u) : 0u;
        if (!cortex_m4_coverage_define_instruction(coverage, section->address + offset,
                                                   wide ? conditional_thumb32(first, second)
                                                        : conditional_thumb16(first))) {
            free(mappings);
            return false;
        }
        offset += wide ? 4u : 2u;
    }
    free(mappings);
    return true;
}

CortexM4Coverage* cortex_m4_coverage_create_elf_data(const void* image_data, size_t image_size) {
    if (image_data == NULL) {
        return NULL;
    }
    const uint8_t* bytes = image_data;
    uint32_t table_offset;
    uint16_t section_count;
    if (!section_table(bytes, image_size, &table_offset, &section_count)) {
        return NULL;
    }
    const uint16_t entry_size = read_u16(bytes + 46u);
    uint32_t first_address = UINT32_MAX;
    uint64_t end_address = 0u;
    for (uint16_t section_index = 0u; section_index < section_count; section_index++) {
        const ElfSection section = read_section(bytes, table_offset, entry_size, section_index);
        if (!executable_section(&section, image_size)) {
            continue;
        }
        if (section.address < first_address) {
            first_address = section.address;
        }
        if ((uint64_t)section.address + section.size > end_address) {
            end_address = (uint64_t)section.address + section.size;
        }
    }
    if (first_address == UINT32_MAX || end_address - first_address > SIZE_MAX) {
        return NULL;
    }
    CortexM4Coverage* coverage =
        cortex_m4_coverage_create(first_address, (size_t)(end_address - first_address));
    if (coverage == NULL) {
        return NULL;
    }
    for (uint16_t section_index = 0u; section_index < section_count; section_index++) {
        const ElfSection section = read_section(bytes, table_offset, entry_size, section_index);
        if (executable_section(&section, image_size) &&
            !define_executable_section(coverage, bytes, image_size, table_offset, entry_size,
                                       section_count, section_index, &section)) {
            cortex_m4_coverage_destroy(coverage);
            return NULL;
        }
    }
    return coverage;
}

bool cortex_m4_load_elf(Kinetis* device, const char* path, uint32_t* entry_address) {
    uint8_t* image_data = NULL;
    size_t image_size = 0;
    if (!load_file(path, &image_data, &image_size)) {
        return false;
    }
    const bool loaded = cortex_m4_load_elf_data(device, image_data, image_size, entry_address);
    free(image_data);
    return loaded;
}

bool cortex_m4_load_binary(Kinetis* device, const char* path, uint32_t load_address) {
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

bool cortex_m4_elf_symbol(const char* path, const char* name, uint32_t* address) {
    uint8_t* image_data = NULL;
    size_t image_size = 0u;
    if (!load_file(path, &image_data, &image_size)) {
        return false;
    }
    const bool found = cortex_m4_elf_symbol_data(image_data, image_size, name, address);
    free(image_data);
    return found;
}

CortexM4Coverage* cortex_m4_coverage_create_elf(const char* path) {
    uint8_t* image_data = NULL;
    size_t image_size = 0u;
    if (!load_file(path, &image_data, &image_size)) {
        return NULL;
    }
    CortexM4Coverage* coverage = cortex_m4_coverage_create_elf_data(image_data, image_size);
    free(image_data);
    return coverage;
}
