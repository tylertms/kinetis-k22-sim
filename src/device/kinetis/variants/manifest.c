#include "device/kinetis/variants/manifest.h"

#include <string.h>

#include "device/kinetis/variants/register_data.h"

const K22RegisterManifest* k22_register_manifest_get(K22ProfileId profile) {
    switch (profile) {
    case K22_PROFILE_MK22F12810:
        return k22_mk22f12810_register_manifest();
    case K22_PROFILE_MK22FN12812:
        return k22_mk22fn12812_register_manifest();
    case K22_PROFILE_MK22FN25612:
        return k22_mk22fn25612_register_manifest();
    case K22_PROFILE_MK22FN51212:
        return k22_mk22fn51212_register_manifest();
    case K22_PROFILE_MK22FN1M012:
        return k22_mk22fn1m012_register_manifest();
    case K22_PROFILE_MK22FX51212:
        return k22_mk22fx51212_register_manifest();
    default:
        return NULL;
    }
}

const K22RegisterDescriptor* k22_register_manifest_lookup(K22ProfileId profile, uint32_t address,
                                                          uint8_t width) {
    const K22RegisterManifest* manifest = k22_register_manifest_get(profile);
    if (manifest == NULL || (width != 8u && width != 16u && width != 32u))
        return NULL;
    size_t lower_bound = 0;
    size_t upper_bound = manifest->register_count;
    while (lower_bound < upper_bound) {
        size_t middle = lower_bound + (upper_bound - lower_bound) / 2u;
        const K22RegisterDescriptor* descriptor = &manifest->registers[middle];
        if (descriptor->address < address ||
            (descriptor->address == address && descriptor->width < width))
            lower_bound = middle + 1u;
        else
            upper_bound = middle;
    }
    if (lower_bound == manifest->register_count)
        return NULL;
    const K22RegisterDescriptor* descriptor = &manifest->registers[lower_bound];
    if (descriptor->address != address || descriptor->width != width)
        return NULL;
    return descriptor;
}

bool k22_register_manifest_reset(K22ProfileId profile, uint32_t address, uint8_t width,
                                 uint32_t* reset_value, uint32_t* reset_mask) {
    const K22RegisterDescriptor* descriptor = k22_register_manifest_lookup(profile, address, width);
    if (descriptor == NULL || reset_value == NULL || reset_mask == NULL)
        return false;
    *reset_value = descriptor->reset_value;
    *reset_mask = descriptor->reset_mask;
    return true;
}

bool k22_register_manifest_has_peripheral(K22ProfileId profile, const char* name) {
    const K22RegisterManifest* manifest = k22_register_manifest_get(profile);
    if (manifest == NULL || name == NULL)
        return false;
    for (size_t index = 0; index < manifest->peripheral_count; index++) {
        if (strcmp(manifest->peripheral_names[index], name) == 0)
            return true;
    }
    return false;
}

const char* k22_register_manifest_peripheral_name(const K22RegisterManifest* manifest,
                                                  uint16_t index) {
    if (manifest == NULL || index >= manifest->peripheral_count)
        return NULL;
    return manifest->peripheral_names[index];
}
