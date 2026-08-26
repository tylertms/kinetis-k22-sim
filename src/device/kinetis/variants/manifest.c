#include "device/kinetis/variants/manifest.h"

#include <string.h>

#include "device/kinetis/variants/register_data.h"

const KinetisRegisterManifest* kinetis_register_manifest_get(KinetisProfile profile) {
    switch (profile) {
    case KINETIS_PROFILE_MK22FN12810:
        return kinetis_mk22fn12810_register_manifest();
    case KINETIS_PROFILE_MKV30F12810:
        return kinetis_mkv30f12810_register_manifest();
    case KINETIS_PROFILE_MK22FN12812:
        return kinetis_mk22fn12812_register_manifest();
    case KINETIS_PROFILE_MK22FN25612:
        return kinetis_mk22fn25612_register_manifest();
    case KINETIS_PROFILE_MK22FN256CAP12:
        return kinetis_mk22fn256cap12_register_manifest();
    case KINETIS_PROFILE_MK22FN51212:
        return kinetis_mk22fn51212_register_manifest();
    case KINETIS_PROFILE_MK22FN1M012:
        return kinetis_mk22fn1m012_register_manifest();
    case KINETIS_PROFILE_MK22FX51212:
        return kinetis_mk22fx51212_register_manifest();
    default:
        return NULL;
    }
}

const KinetisRegisterDescriptor* kinetis_register_manifest_lookup(KinetisProfile profile,
                                                                  uint32_t address, uint8_t width) {
    const KinetisRegisterManifest* manifest = kinetis_register_manifest_get(profile);
    if (manifest == NULL || (width != 8u && width != 16u && width != 32u))
        return NULL;
    size_t lower_bound = 0;
    size_t upper_bound = manifest->register_count;
    while (lower_bound < upper_bound) {
        size_t middle = lower_bound + (upper_bound - lower_bound) / 2u;
        const KinetisRegisterDescriptor* descriptor = &manifest->registers[middle];
        if (descriptor->address < address ||
            (descriptor->address == address && descriptor->width < width))
            lower_bound = middle + 1u;
        else
            upper_bound = middle;
    }
    if (lower_bound == manifest->register_count)
        return NULL;
    const KinetisRegisterDescriptor* descriptor = &manifest->registers[lower_bound];
    if (descriptor->address != address || descriptor->width != width)
        return NULL;
    return descriptor;
}

bool kinetis_register_manifest_reset(KinetisProfile profile, uint32_t address, uint8_t width,
                                     uint32_t* reset_value, uint32_t* reset_mask) {
    const KinetisRegisterDescriptor* descriptor =
        kinetis_register_manifest_lookup(profile, address, width);
    if (descriptor == NULL || reset_value == NULL || reset_mask == NULL)
        return false;
    *reset_value = descriptor->reset_value;
    *reset_mask = descriptor->reset_mask;
    return true;
}

bool kinetis_register_manifest_has_peripheral(KinetisProfile profile, const char* name) {
    const KinetisRegisterManifest* manifest = kinetis_register_manifest_get(profile);
    if (manifest == NULL || name == NULL)
        return false;
    for (size_t index = 0; index < manifest->peripheral_count; index++) {
        if (strcmp(manifest->peripheral_names[index], name) == 0)
            return true;
    }
    return false;
}

const char* kinetis_register_manifest_peripheral_name(const KinetisRegisterManifest* manifest,
                                                      uint16_t index) {
    if (manifest == NULL || index >= manifest->peripheral_count)
        return NULL;
    return manifest->peripheral_names[index];
}
