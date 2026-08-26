#ifndef KINETIS_SIM_REGISTER_MANIFEST_H
#define KINETIS_SIM_REGISTER_MANIFEST_H

#include "device/kinetis/variants/profile.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    KINETIS_REGISTER_ACCESS_NONE = 0,
    KINETIS_REGISTER_ACCESS_READ = 1,
    KINETIS_REGISTER_ACCESS_WRITE = 2,
    KINETIS_REGISTER_ACCESS_READ_WRITE = 3,
} KinetisRegisterAccess;

typedef struct {
    uint32_t address;
    uint32_t reset_value;
    uint32_t reset_mask;
    uint32_t implemented_mask;
    uint32_t read_mask;
    uint32_t write_mask;
    uint32_t w1c_mask;
    uint16_t peripheral_index;
    uint8_t width;
    KinetisRegisterAccess access;
} KinetisRegisterDescriptor;

typedef struct {
    KinetisProfile profile;
    const KinetisRegisterDescriptor* registers;
    size_t register_count;
    const char* const* peripheral_names;
    size_t peripheral_count;
    uint64_t register_digest;
    uint64_t peripheral_digest;
} KinetisRegisterManifest;

const KinetisRegisterManifest* kinetis_register_manifest_get(KinetisProfile profile);
const KinetisRegisterDescriptor* kinetis_register_manifest_lookup(KinetisProfile profile,
                                                                  uint32_t address, uint8_t width);
bool kinetis_register_manifest_reset(KinetisProfile profile, uint32_t address, uint8_t width,
                                     uint32_t* reset_value, uint32_t* reset_mask);
bool kinetis_register_manifest_has_peripheral(KinetisProfile profile, const char* name);
const char* kinetis_register_manifest_peripheral_name(const KinetisRegisterManifest* manifest,
                                                      uint16_t index);

#endif
