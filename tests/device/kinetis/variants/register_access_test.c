#include "kinetis.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "device/kinetis/internal.h"
#include "device/kinetis/variants/manifest.h"
#include "device/kinetis/variants/package.h"
#include "device/kinetis/variants/profile.h"
#include "test.h"

typedef struct {
    KinetisProfile profile;
    KinetisPackage package;
} ProfileFixture;

static const ProfileFixture fixtures[] = {
    {KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_DC_121_XFBGA},
    {KINETIS_PROFILE_MKV30F12810, KINETIS_PACKAGE_LH_64_LQFP},
    {KINETIS_PROFILE_MK22FN12812, KINETIS_PACKAGE_AH_64_WLCSP},
    {KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_DC_121_XFBGA},
    {KINETIS_PROFILE_MK22FN256CAP12, KINETIS_PACKAGE_AP_80_WLCSP},
    {KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_DC_121_XFBGA},
    {KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LQ_144_LQFP},
    {KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LQ_144_LQFP},
};

static const uint8_t flash_configuration[16] = {
    0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
    0x81u, 0x82u, 0x84u, 0x88u, 0xfeu, 0x5au, 0xc3u, 0x3cu,
};

static const uint32_t sim_fcfg1_reset_values[KINETIS_PROFILE_COUNT] = {
    0x0f0f0f00u, 0x0f0f0f00u, 0x0f0f0f00u, 0x0f0f0f00u,
    0x090f0f00u, 0x0f0f0f00u, 0xff0f0f00u, 0xff0f0f00u,
};

static const uint32_t sim_fcfg2_reset_values[KINETIS_PROFILE_COUNT] = {
    0x10000000u, 0x10800000u, 0x10000000u, 0x20000000u,
    0x20000000u, 0x20200000u, 0x40c00000u, 0x40100000u,
};

static uint32_t width_mask(uint8_t width) {
    return width == 32u ? UINT32_MAX : (UINT32_C(1) << width) - 1u;
}

static bool descriptor_available(const Kinetis* device,
                                 const KinetisRegisterDescriptor* descriptor) {
    KinetisPeripheralLocation location;
    return kinetis_profile_resolve_peripheral(device->profile, descriptor->address,
                                              (uint8_t)(descriptor->width / 8u), &location) &&
           kinetis_package_has_peripheral(device->package, location.id);
}

static Kinetis* create_device(TestState* state, const ProfileFixture* fixture) {
    const KinetisDeviceProfile* profile = kinetis_profile_get(fixture->profile);
    expect(state, profile != NULL, "profile != NULL");
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    configuration.profile = fixture->profile;
    configuration.package = (KinetisPackage)fixture->package;
    configuration.flash_size = profile->program_flash_size;
    configuration.sram_size = (size_t)profile->sram_lower_size + profile->sram_upper_size;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "device != NULL");
    if (device == NULL) {
        return NULL;
    }
    const uint32_t vectors[] = {
        profile->sram_upper_address + profile->sram_upper_size,
        0x00000101u,
    };
    const uint16_t program = 0xbf00u;
    expect(state, kinetis_load(device, 0u, vectors, sizeof(vectors)),
           "kinetis_load(device, 0u, vectors, sizeof(vectors))");
    expect(state, kinetis_load(device, 0x100u, &program, sizeof(program)),
           "kinetis_load(device, 0x100u, &program, sizeof(program))");
    expect(state, kinetis_load(device, 0x400u, flash_configuration, sizeof(flash_configuration)),
           "kinetis_load(device, 0x400u, flash_configuration, "
           "sizeof(flash_configuration))");
    expect(state, kinetis_reset(device), "kinetis_reset(device)");
    return device;
}

static void report_mismatch(const KinetisRegisterDescriptor* descriptor, const char* field,
                            uint32_t actual, uint32_t expected) {
    fprintf(stderr, "register 0x%08lx width %u has %s 0x%08lx, expected 0x%08lx\n",
            (unsigned long)descriptor->address, descriptor->width, field, (unsigned long)actual,
            (unsigned long)expected);
}

static const KinetisRegisterDescriptor*
widest_covering_descriptor(const KinetisRegisterManifest* manifest,
                           const KinetisRegisterDescriptor* descriptor) {
    const KinetisRegisterDescriptor* widest = descriptor;
    const uint32_t end = descriptor->address + descriptor->width / 8u;
    for (size_t index = 0u; index < manifest->register_count; index++) {
        const KinetisRegisterDescriptor* candidate = &manifest->registers[index];
        const uint32_t candidate_end = candidate->address + candidate->width / 8u;
        if (candidate->peripheral_index == descriptor->peripheral_index &&
            candidate->address <= descriptor->address && candidate_end >= end &&
            candidate->width > widest->width)
            widest = candidate;
    }
    return widest;
}

static uint32_t expected_reset_value(const ProfileFixture* fixture,
                                     const KinetisRegisterManifest* manifest,
                                     const KinetisRegisterDescriptor* descriptor) {
    if (descriptor->address == 0x40048024u) {
        const KinetisDeviceProfile* profile = kinetis_profile_get(fixture->profile);
        const KinetisPackageSelection* package = kinetis_package_select(profile, fixture->package);
        return (profile->sim_sdid_reset & ~15u) | kinetis_package_pin_id(package);
    }
    if (descriptor->address == 0x4004804cu) {
        return sim_fcfg1_reset_values[fixture->profile];
    }
    if (descriptor->address == 0x40048050u) {
        return sim_fcfg2_reset_values[fixture->profile];
    }
    const uint32_t offset = descriptor->address - 0x40020000u;
    if (offset == 1u && (manifest->profile == KINETIS_PROFILE_MK22FN1M012 ||
                         manifest->profile == KINETIS_PROFILE_MK22FX51212))
        return 2u;
    if (offset == 2u)
        return flash_configuration[12];
    if (offset == 3u)
        return flash_configuration[13];
    if (offset >= 0x10u && offset <= 0x13u)
        return flash_configuration[8u + offset - 0x10u];
    if (offset == 0x16u)
        return flash_configuration[14];
    if (offset == 0x17u)
        return flash_configuration[15];
    const KinetisRegisterDescriptor* widest = widest_covering_descriptor(manifest, descriptor);
    return widest->reset_value >> ((descriptor->address - widest->address) * 8u);
}

static bool uses_flash_configuration(const KinetisRegisterDescriptor* descriptor) {
    const uint32_t offset = descriptor->address - 0x40020000u;
    return offset == 2u || offset == 3u || (offset >= 0x10u && offset <= 0x13u) ||
           offset == 0x16u || offset == 0x17u;
}

static void expect_reset_read(TestState* state, Kinetis* device, const ProfileFixture* fixture,
                              const KinetisRegisterManifest* manifest,
                              const KinetisRegisterDescriptor* descriptor) {
    if (descriptor->address < KINETIS_PERIPHERAL_BASE ||
        !descriptor_available(device, descriptor)) {
        return;
    }
    expect(state, kinetis_reset(device), "kinetis_reset(device)");
    for (uint8_t port = 0u; port < 5u; port++) {
        for (uint8_t pin = 0u; pin < 32u; pin++) {
            kinetis_gpio_drive(device, port, pin, false);
        }
    }
    const uint8_t size = (uint8_t)(descriptor->width / 8u);
    uint32_t actual = UINT32_MAX;
    const bool read = kinetis_read(device, descriptor->address, &actual, size);
    const bool readable = (descriptor->access & KINETIS_REGISTER_ACCESS_READ) != 0u;
    if (read != readable) {
        report_mismatch(descriptor, "read access", read, readable);
    }
    expect(state, read == readable, "read == readable");
    if (!readable) {
        return;
    }
    actual &= width_mask(descriptor->width);
    const KinetisRegisterDescriptor* widest = widest_covering_descriptor(manifest, descriptor);
    const bool semantic_reset = descriptor->address == 0x40048024u ||
                                descriptor->address == 0x4004804cu ||
                                descriptor->address == 0x40048050u;
    const uint32_t reset_mask =
        uses_flash_configuration(descriptor) || semantic_reset
            ? width_mask(descriptor->width)
            : widest->reset_mask >> ((descriptor->address - widest->address) * 8u);
    const uint32_t expected = expected_reset_value(fixture, manifest, descriptor) & reset_mask &
                              width_mask(descriptor->width);
    if (actual != expected) {
        report_mismatch(descriptor, "reset value", actual, expected);
    }
    expect(state, actual == expected, "actual == expected");
}

static void expect_declared_writes(TestState* state, Kinetis* device,
                                   const KinetisRegisterDescriptor* descriptor) {
    if (descriptor->address < KINETIS_PERIPHERAL_BASE ||
        !descriptor_available(device, descriptor)) {
        return;
    }
    const uint8_t size = (uint8_t)(descriptor->width / 8u);
    static const uint32_t values[] = {
        0u, 1u, 0x55555555u, 0xaaaaaaaau, UINT32_MAX,
    };
    for (size_t index = 0u; index < sizeof(values) / sizeof(values[0]); index++) {
        expect(state, kinetis_reset(device), "kinetis_reset(device)");
        const bool wrote = kinetis_write(device, descriptor->address, &values[index], size);
        expect(state, wrote, "wrote");
    }
}

static bool descriptor_covers(const KinetisRegisterDescriptor* descriptor, uint32_t address) {
    const uint32_t size = descriptor->width / 8u;
    return address >= descriptor->address && address < descriptor->address + size;
}

static void expect_reserved_gaps(TestState* state, Kinetis* device,
                                 const KinetisRegisterManifest* manifest) {
    for (size_t index = 0u; index + 1u < manifest->register_count; index++) {
        const KinetisRegisterDescriptor* current = &manifest->registers[index];
        const KinetisRegisterDescriptor* next = &manifest->registers[index + 1u];
        const uint32_t address = current->address + current->width / 8u;
        if (address >= next->address || address < KINETIS_PERIPHERAL_BASE ||
            descriptor_covers(current, address)) {
            continue;
        }
        KinetisPeripheralLocation location;
        if (!kinetis_profile_resolve_peripheral(device->profile, address, 1u, &location)) {
            continue;
        }
        uint8_t value = 0u;
        expect(state, !kinetis_read(device, address, &value, sizeof(value)),
               "!kinetis_read(device, address, &value, sizeof(value))");
        expect(state, !kinetis_write(device, address, &value, sizeof(value)),
               "!kinetis_write(device, address, &value, sizeof(value))");
    }
}

static void expect_profile(TestState* state, const ProfileFixture* fixture) {
    Kinetis* device = create_device(state, fixture);
    if (device == NULL) {
        return;
    }
    const KinetisRegisterManifest* manifest = kinetis_register_manifest_get(fixture->profile);
    expect(state, manifest != NULL, "manifest != NULL");
    if (manifest != NULL) {
        for (size_t index = 0u; index < manifest->register_count; index++) {
            expect_reset_read(state, device, fixture, manifest, &manifest->registers[index]);
            expect_declared_writes(state, device, &manifest->registers[index]);
        }
        expect_reserved_gaps(state, device, manifest);
    }
    kinetis_destroy(device);
}

int main(void) {
    TestState state = {0};
    for (size_t index = 0u; index < sizeof(fixtures) / sizeof(fixtures[0]); index++) {
        expect_profile(&state, &fixtures[index]);
    }
    return test_finish(&state);
}
