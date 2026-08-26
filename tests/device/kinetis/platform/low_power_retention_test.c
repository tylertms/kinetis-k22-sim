#include "device/kinetis/internal.h"

#include <string.h>

#include "test.h"

enum {
    LLWU_PE1 = 0x4007c000u,
    SMC_PMPROT = 0x4007e000u,
    SMC_PMCTRL = 0x4007e001u,
    SMC_STOPCTRL = 0x4007e002u,
    SMC_PMSTAT = 0x4007e003u,
};

static const uint32_t scb_scr_address = 0xe000ed10u;

typedef struct {
    KinetisProfile profile;
    uint32_t retained_size;
    uint32_t retained_size_with_ram2;
} RetentionCase;

static void expect_write8(TestState* state, Kinetis* device, uint32_t address, uint8_t value) {
    expect(state, kinetis_write(device, address, &value, sizeof(value)),
           "8-bit register write succeeds");
}

static uint8_t read8(TestState* state, Kinetis* device, uint32_t address) {
    uint8_t value = UINT8_MAX;
    expect(state, kinetis_read(device, address, &value, sizeof(value)),
           "8-bit register read succeeds");
    return value;
}

static Kinetis* create_sleeping_device(TestState* state, KinetisProfile profile, uint8_t submode,
                                       bool retain_ram2) {
    KinetisConfiguration configuration = kinetis_configuration(profile);
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "low-power device is created");
    if (device == NULL)
        return NULL;

    const uint32_t vectors[2] = {0x20000800u, 0x00000101u};
    const uint16_t wait_for_interrupt = 0xbf30u;
    expect(state, kinetis_load(device, 0u, vectors, sizeof(vectors)), "vectors are loaded");
    expect(state, kinetis_load(device, 0x100u, &wait_for_interrupt, sizeof(wait_for_interrupt)),
           "low-power program is loaded");
    expect(state, kinetis_reset(device), "low-power device resets");

    memset(device->sram, 0x5au, device->configuration.sram_size);
    memset(device->sram_initialized, 1, device->configuration.sram_size);
    if (device->profile->flexram_size != 0u) {
        const uint32_t flexram_value = 0xa5a55a5au;
        expect(state,
               kinetis_write(device, device->profile->flexram_address, &flexram_value,
                             sizeof(flexram_value)),
               "FlexRAM sentinel is written");
    }
    expect_write8(state, device, SMC_PMPROT, 0x02u);
    expect_write8(state, device, SMC_STOPCTRL, (uint8_t)(submode | (retain_ram2 ? 0x10u : 0u)));
    expect_write8(state, device, SMC_PMCTRL, 0x04u);
    expect_write8(state, device, LLWU_PE1, 0x01u);
    expect(state, cortex_m4_write_memory(kinetis_cpu(device), scb_scr_address, 4u, 4u),
           "firmware enables deep sleep");
    expect(state, cortex_m4_step(kinetis_cpu(device)).instructions == 1u, "firmware executes WFI");
    kinetis_advance(device, 1u);
    expect(state, read8(state, device, SMC_PMSTAT) == 0x40u, "device enters VLLS");
    return device;
}

static void expect_sram_retention(TestState* state, const RetentionCase* test_case, uint8_t submode,
                                  bool retain_ram2) {
    Kinetis* device = create_sleeping_device(state, test_case->profile, submode, retain_ram2);
    if (device == NULL)
        return;

    expect(state, kinetis_set_llwu_pin(device, 0u, true), "LLWU wakes the device");
    uint32_t retained_size = 0u;
    if (submode == 3u)
        retained_size = device->profile->sram_upper_size;
    else if (submode == 2u)
        retained_size = retain_ram2 ? test_case->retained_size_with_ram2 : test_case->retained_size;
    bool contents_match = true;
    bool initialization_matches = true;
    for (size_t offset = 0u; offset < device->configuration.sram_size; offset++) {
        const uint32_t address = device->sram_base + (uint32_t)offset;
        const bool retained =
            submode == 3u || (address >= device->profile->sram_upper_address &&
                              address - device->profile->sram_upper_address < retained_size);
        contents_match &= device->sram[offset] == (retained ? 0x5au : 0u);
        initialization_matches &= device->sram_initialized[offset] == (retained ? 1u : 0u);
    }
    expect(state, contents_match, "VLLS retains only the powered SRAM range");
    expect(state, initialization_matches, "VLLS invalidates only the unpowered SRAM range");
    if (device->profile->flexram_size != 0u) {
        uint32_t flexram_value = UINT32_MAX;
        expect(state,
               kinetis_read(device, device->profile->flexram_address, &flexram_value,
                            sizeof(flexram_value)),
               "FlexRAM sentinel is read");
        expect(state, flexram_value == 0u, "VLLS powers off FlexRAM");
    }
    kinetis_destroy(device);
}

int main(void) {
    static const RetentionCase cases[] = {
        {KINETIS_PROFILE_MK22FN12810, 0x2000u, 0x2000u},
        {KINETIS_PROFILE_MKV30F12810, 0x2000u, 0x2000u},
        {KINETIS_PROFILE_MK22FN12812, 0x4000u, 0x4000u},
        {KINETIS_PROFILE_MK22FN25612, 0x4000u, 0x4000u},
        {KINETIS_PROFILE_MK22FN51212, 0x8000u, 0x8000u},
        {KINETIS_PROFILE_MK22FN1M012, 0x1000u, 0x4000u},
        {KINETIS_PROFILE_MK22FX51212, 0x1000u, 0x4000u},
    };
    TestState state = {0};
    for (size_t case_index = 0u; case_index < sizeof(cases) / sizeof(cases[0]); case_index++) {
        for (uint8_t submode = 0u; submode <= 3u; submode++)
            expect_sram_retention(&state, &cases[case_index], submode, false);
    }
    expect_sram_retention(&state, &cases[5], 2u, true);
    expect_sram_retention(&state, &cases[6], 2u, true);
    return test_finish(&state);
}
