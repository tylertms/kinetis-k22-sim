#ifndef KINETIS_SIM_CORTEX_M4_FIRMWARE_IMAGE_H
#define KINETIS_SIM_CORTEX_M4_FIRMWARE_IMAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kinetis.h"

bool cortex_m4_load_elf_data(Kinetis* device, const void* image_data, size_t image_size,
                             uint32_t* entry_address);
bool cortex_m4_load_binary_data(Kinetis* device, const void* image_data, size_t image_size,
                                uint32_t load_address, uint32_t* entry_address);
bool cortex_m4_elf_symbol_data(const void* image_data, size_t image_size, const char* name,
                               uint32_t* address);
CortexM4Coverage* cortex_m4_coverage_create_elf_data(const void* image_data, size_t image_size);
size_t cortex_m4_coverage_select_elf_functions_data(CortexM4Coverage* coverage,
                                                    const void* image_data, size_t image_size,
                                                    const char* name_prefix);
bool cortex_m4_load_elf(Kinetis* device, const char* path, uint32_t* entry_address);
bool cortex_m4_load_binary(Kinetis* device, const char* path, uint32_t load_address);
bool cortex_m4_elf_symbol(const char* path, const char* name, uint32_t* address);
CortexM4Coverage* cortex_m4_coverage_create_elf(const char* path);
size_t cortex_m4_coverage_select_elf_functions(CortexM4Coverage* coverage, const char* path,
                                               const char* name_prefix);

#endif
