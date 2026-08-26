#include "device/kinetis/variants/profile.h"

#include <stdint.h>
#include <string.h>

#include "profile_expectations.h"
#include "test.h"

static void expect_cpu(TestState* state, const KinetisDeviceProfile* profile,
                       const KinetisExpectedProfile* expected) {
    expect(state, profile->cpu.architecture == KINETIS_CPU_ARCHITECTURE_ARMV7E_M,
           "profile->cpu.architecture == KINETIS_CPU_ARCHITECTURE_ARMV7E_M");
    expect(state, profile->cpu.core_revision_major == 0, "profile->cpu.core_revision_major == 0");
    expect(state, profile->cpu.core_revision_minor == 1, "profile->cpu.core_revision_minor == 1");
    expect(state, profile->cpu.nvic_priority_bits == 4, "profile->cpu.nvic_priority_bits == 4");
    expect(state, profile->cpu.external_irq_count == expected->external_irq_count,
           "profile->cpu.external_irq_count == expected->external_irq_count");
    expect(state, profile->cpu.little_endian, "profile->cpu.little_endian");
    expect(state, profile->cpu.has_fpu, "profile->cpu.has_fpu");
    expect(state, !profile->cpu.has_mpu, "!profile->cpu.has_mpu");
    expect(state, profile->cpu.has_vtor, "profile->cpu.has_vtor");
    expect(state, profile->cpu.has_systick, "profile->cpu.has_systick");
    expect(state, profile->cpu.maximum_core_clock_hz == expected->maximum_core_clock_hz,
           "profile->cpu.maximum_core_clock_hz == expected->maximum_core_clock_hz");
}

static void expect_memory(TestState* state, const KinetisDeviceProfile* profile,
                          const KinetisExpectedProfile* expected) {
    expect(state, profile->program_flash_size == expected->program_flash_size,
           "profile->program_flash_size == expected->program_flash_size");
    expect(state, profile->sram_lower_address == expected->sram_lower_address,
           "profile->sram_lower_address == expected->sram_lower_address");
    expect(state, profile->sram_lower_size == expected->sram_lower_size,
           "profile->sram_lower_size == expected->sram_lower_size");
    expect(state, profile->sram_upper_address == expected->sram_upper_address,
           "profile->sram_upper_address == expected->sram_upper_address");
    expect(state, profile->sram_upper_size == expected->sram_upper_size,
           "profile->sram_upper_size == expected->sram_upper_size");
    expect(state, profile->flexnvm_address == expected->flexnvm_address,
           "profile->flexnvm_address == expected->flexnvm_address");
    expect(state, profile->flexnvm_size == expected->flexnvm_size,
           "profile->flexnvm_size == expected->flexnvm_size");
    expect(state, profile->flexram_address == expected->flexram_address,
           "profile->flexram_address == expected->flexram_address");
    expect(state, profile->flexram_size == expected->flexram_size,
           "profile->flexram_size == expected->flexram_size");
}

static void expect_block(TestState* state, const KinetisDeviceProfile* profile,
                         const KinetisExpectedBlock* expected) {
    KinetisPeripheralBlock block = {0};
    expect(state, kinetis_profile_has_peripheral(profile, expected->id),
           "kinetis_profile_has_peripheral(profile, expected->id)");
    expect(state, kinetis_profile_peripheral_block(profile, expected->id, &block),
           "kinetis_profile_peripheral_block(profile, expected->id, &block)");
    expect(state, block.id == expected->id, "block.id == expected->id");
    expect(state, block.address == expected->address, "block.address == expected->address");
    expect(state, block.size == expected->size, "block.size == expected->size");

    KinetisPeripheralLocation location = {0};
    expect(state, kinetis_profile_resolve_peripheral(profile, expected->address, 1, &location),
           "kinetis_profile_resolve_peripheral(profile, expected->address, 1, &location)");
    expect(state, location.id == expected->id, "location.id == expected->id");
    expect(state, location.block_address == expected->address,
           "location.block_address == expected->address");
    expect(state, location.block_size == expected->size, "location.block_size == expected->size");
    expect(state, location.offset == 0, "location.offset == 0");

    uint32_t last_address = expected->address + expected->size - 1;
    expect(state, kinetis_profile_resolve_peripheral(profile, last_address, 1, &location),
           "kinetis_profile_resolve_peripheral(profile, last_address, 1, &location)");
    expect(state, location.offset == expected->size - 1, "location.offset == expected->size - 1");
    expect(state, !kinetis_profile_resolve_peripheral(profile, last_address, 2, NULL),
           "!kinetis_profile_resolve_peripheral(profile, last_address, 2, NULL)");
    if (expected->size >= 4) {
        uint32_t final_word_address = expected->address + expected->size - 4;
        expect(state, kinetis_profile_resolve_peripheral(profile, final_word_address, 4, NULL),
               "kinetis_profile_resolve_peripheral(profile, final_word_address, 4, NULL)");
    }
}

static void expect_profile(TestState* state, const KinetisExpectedProfile* expected) {
    KinetisProfile public_profile = KINETIS_PROFILE_COUNT;
    const KinetisDeviceProfile* profile = kinetis_profile_get(expected->id);
    expect(state, profile != NULL, "profile != NULL");
    expect(state, profile->id == expected->id, "profile->id == expected->id");
    expect(state, strcmp(profile->name, expected->name) == 0,
           "strcmp(profile->name, expected->name) == 0");
    expect(state, kinetis_profile_find(expected->name) == profile,
           "kinetis_profile_find(expected->name) == profile");
    expect(state, kinetis_profile_from_name(expected->name, &public_profile),
           "kinetis_profile_from_name(expected->name, &public_profile)");
    expect(state, public_profile == expected->id, "public_profile == expected->id");
    expect(state, strcmp(kinetis_profile_name(public_profile), expected->name) == 0,
           "strcmp(kinetis_profile_name(public_profile), expected->name) == 0");
    KinetisConfiguration configuration = kinetis_configuration(public_profile);
    expect(state, configuration.profile == public_profile,
           "configuration.profile == public_profile");
    expect(state, configuration.flash_size == expected->program_flash_size,
           "configuration.flash_size == expected->program_flash_size");
    expect(state, configuration.sram_size == expected->sram_lower_size + expected->sram_upper_size,
           "configuration.sram_size == expected SRAM size");
    expect(state, profile->sim_sdid_reset == expected->sim_sdid_reset,
           "profile->sim_sdid_reset == expected->sim_sdid_reset");
    expect(state, profile->sim_sdid_mask == expected->sim_sdid_mask,
           "profile->sim_sdid_mask == expected->sim_sdid_mask");
    expect(state, profile->peripheral_block_count == expected->block_count,
           "profile->peripheral_block_count == expected->block_count");
    expect_cpu(state, profile, expected);
    expect_memory(state, profile, expected);

    bool expected_ids[KINETIS_PERIPHERAL_COUNT] = {false};
    for (size_t index = 0; index < expected->block_count; index++) {
        expect(state, !expected_ids[expected->blocks[index].id],
               "!expected_ids[expected->blocks[index].id]");
        expected_ids[expected->blocks[index].id] = true;
        expect_block(state, profile, &expected->blocks[index]);
    }
    for (int peripheral_id = 0; peripheral_id < KINETIS_PERIPHERAL_COUNT; peripheral_id++) {
        expect(state,
               kinetis_profile_has_peripheral(profile, (KinetisPeripheralId)peripheral_id) ==
                   expected_ids[peripheral_id],
               "kinetis_profile_has_peripheral(profile, (KinetisPeripheralId)peripheral_id) == "
               "expected_ids[peripheral_id]");
    }
}

static void expect_fail_closed(TestState* state) {
    const KinetisDeviceProfile* small_profile = kinetis_profile_get(KINETIS_PROFILE_MK22F12810);
    const KinetisDeviceProfile* large_profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN1M012);
    expect(state, !kinetis_profile_resolve_peripheral(NULL, 0x40000000u, 4, NULL),
           "!kinetis_profile_resolve_peripheral(NULL, 0x40000000u, 4, NULL)");
    expect(state, !kinetis_profile_resolve_peripheral(small_profile, 0x40000000u, 4, NULL),
           "!kinetis_profile_resolve_peripheral(small_profile, 0x40000000u, 4, NULL)");
    expect(state, kinetis_profile_resolve_peripheral(large_profile, 0x40000000u, 4, NULL),
           "kinetis_profile_resolve_peripheral(large_profile, 0x40000000u, 4, NULL)");
    expect(state, !kinetis_profile_resolve_peripheral(small_profile, 0x40003000u, 4, NULL),
           "!kinetis_profile_resolve_peripheral(small_profile, 0x40003000u, 4, NULL)");
    expect(state, !kinetis_profile_resolve_peripheral(small_profile, 0xe000e000u, 4, NULL),
           "!kinetis_profile_resolve_peripheral(small_profile, 0xe000e000u, 4, NULL)");
    expect(state, !kinetis_profile_resolve_peripheral(small_profile, UINT32_MAX, 4, NULL),
           "!kinetis_profile_resolve_peripheral(small_profile, UINT32_MAX, 4, NULL)");
    expect(state, !kinetis_profile_resolve_peripheral(small_profile, 0x40008000u, 0, NULL),
           "!kinetis_profile_resolve_peripheral(small_profile, 0x40008000u, 0, NULL)");
    expect(state, !kinetis_profile_resolve_peripheral(small_profile, 0x40008000u, 3, NULL),
           "!kinetis_profile_resolve_peripheral(small_profile, 0x40008000u, 3, NULL)");
    expect(state, !kinetis_profile_resolve_peripheral(small_profile, 0x40008000u, 8, NULL),
           "!kinetis_profile_resolve_peripheral(small_profile, 0x40008000u, 8, NULL)");
    expect(state, !kinetis_profile_peripheral_block(NULL, KINETIS_PERIPHERAL_DMA, NULL),
           "!kinetis_profile_peripheral_block(NULL, KINETIS_PERIPHERAL_DMA, NULL)");
    expect(state, !kinetis_profile_peripheral_block(small_profile, (KinetisPeripheralId)-1, NULL),
           "!kinetis_profile_peripheral_block(small_profile, (KinetisPeripheralId)-1, NULL)");
    expect(state, !kinetis_profile_peripheral_block(small_profile, KINETIS_PERIPHERAL_COUNT, NULL),
           "!kinetis_profile_peripheral_block(small_profile, KINETIS_PERIPHERAL_COUNT, NULL)");
}

static void expect_invalid_configuration(TestState* state) {
    KinetisConfiguration configuration = kinetis_default_configuration();
    configuration.profile = KINETIS_PROFILE_COUNT;
    expect(state, kinetis_create(configuration) == NULL, "invalid profile is rejected");
    configuration = kinetis_default_configuration();
    configuration.package = KINETIS_PACKAGE_COUNT;
    expect(state, kinetis_create(configuration) == NULL, "invalid package is rejected");
    configuration = kinetis_default_configuration();
    configuration.flash_size = 0u;
    expect(state, kinetis_create(configuration) == NULL, "empty flash is rejected");
    configuration = kinetis_default_configuration();
    configuration.flash_size++;
    expect(state, kinetis_create(configuration) == NULL, "oversized flash is rejected");
    configuration = kinetis_default_configuration();
    configuration.sram_size = 0u;
    expect(state, kinetis_create(configuration) == NULL, "empty SRAM is rejected");
    for (KinetisProfile profile = 0; profile < KINETIS_PROFILE_COUNT; profile++) {
        configuration = kinetis_configuration(profile);
        configuration.sram_size++;
        expect(state, kinetis_create(configuration) == NULL,
               "SRAM larger than the selected device is rejected");
    }
    kinetis_destroy(NULL);
}

int main(void) {
    TestState state = {0};
    for (size_t index = 0; index < EXPECTED_COUNT(expected_profiles); index++)
        expect_profile(&state, &expected_profiles[index]);
    expect(&state, kinetis_profile_get((KinetisProfile)-1) == NULL,
           "kinetis_profile_get((KinetisProfile)-1) == NULL");
    expect(&state, kinetis_profile_get(KINETIS_PROFILE_COUNT) == NULL,
           "kinetis_profile_get(KINETIS_PROFILE_COUNT) == NULL");
    expect(&state, kinetis_profile_find(NULL) == NULL, "kinetis_profile_find(NULL) == NULL");
    expect(&state, kinetis_profile_find("") == NULL, "kinetis_profile_find(\"\") == NULL");
    expect(&state, kinetis_profile_find("MK22FN512VLL12") == NULL,
           "kinetis_profile_find(\"MK22FN512VLL12\") == NULL");
    KinetisProfile public_profile = KINETIS_PROFILE_MK22F12810;
    expect(&state, !kinetis_profile_from_name(NULL, &public_profile),
           "!kinetis_profile_from_name(NULL, &public_profile)");
    expect(&state, !kinetis_profile_from_name("unknown", &public_profile),
           "!kinetis_profile_from_name(\"unknown\", &public_profile)");
    expect(&state, kinetis_profile_name(KINETIS_PROFILE_COUNT) == NULL,
           "kinetis_profile_name(KINETIS_PROFILE_COUNT) == NULL");
    expect_fail_closed(&state);
    expect_invalid_configuration(&state);
    return test_finish(&state);
}
