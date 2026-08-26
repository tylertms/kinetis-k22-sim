#include "device/kinetis/variants/package.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "test.h"

#define MASKS(a, b, c, d, e) {a, b, c, d, e}
#define EXPECTED(profile_id, package_id, a, b, c, d, e, dac1_value, flexbus_value)                 \
    {profile_id, package_id, MASKS(a, b, c, d, e), dac1_value, flexbus_value}

typedef struct {
    KinetisPackage id;
    const char* code;
    const char* name;
    uint16_t terminal_count;
    uint8_t pin_id;
} ExpectedPackage;

typedef struct {
    KinetisProfile profile;
    KinetisPackage package;
    uint32_t port_pin_mask[KINETIS_PACKAGE_PORT_COUNT];
    bool dac1;
    bool flexbus;
} ExpectedSelection;

static const ExpectedPackage expected_packages[] = {
    {KINETIS_PACKAGE_LH_64_LQFP, "LH", "64 LQFP", 64, 5},
    {KINETIS_PACKAGE_MP_64_MAPBGA, "MP", "64 MAPBGA", 64, 5},
    {KINETIS_PACKAGE_AH_64_WLCSP, "AH", "64 WLCSP", 64, 11},
    {KINETIS_PACKAGE_LK_80_LQFP, "LK", "80 LQFP", 80, 6},
    {KINETIS_PACKAGE_AP_80_WLCSP, "AP", "80 WLCSP 0.564 mm", 80, 11},
    {KINETIS_PACKAGE_BP_80_WLCSP, "BP", "80 WLCSP 0.321 mm", 80, 11},
    {KINETIS_PACKAGE_FX_88_HVQFN, "FX", "88 HVQFN", 88, 7},
    {KINETIS_PACKAGE_LL_100_LQFP, "LL", "100 LQFP", 100, 8},
    {KINETIS_PACKAGE_DC_121_XFBGA, "DC", "121 XFBGA", 121, 9},
    {KINETIS_PACKAGE_MC_121_MAPBGA, "MC", "121 MAPBGA", 121, 9},
    {KINETIS_PACKAGE_LQ_144_LQFP, "LQ", "144 LQFP", 144, 10},
    {KINETIS_PACKAGE_MD_144_MAPBGA, "MD", "144 MAPBGA", 144, 10},
    {KINETIS_PACKAGE_AK_49_WLCSP, "AK", "49 WLCSP", 49, 11},
    {KINETIS_PACKAGE_FM_32_QFN, "FM", "32 QFN", 32, 2},
    {KINETIS_PACKAGE_LF_48_LQFP, "LF", "48 LQFP", 48, 4},
};

static const ExpectedSelection expected_selections[] = {
    EXPECTED(KINETIS_PROFILE_MKV30F12810, KINETIS_PACKAGE_FM_32_QFN, 0x000c001fu, 0x00000003u,
             0x000000feu, 0x000000f0u, 0x030f0000u, false, false),
    EXPECTED(KINETIS_PROFILE_MKV30F12810, KINETIS_PACKAGE_LF_48_LQFP, 0x000c001fu, 0x0003000fu,
             0x000000ffu, 0x000000ffu, 0x030f0000u, false, false),
    EXPECTED(KINETIS_PROFILE_MKV30F12810, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x030f0003u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_AK_49_WLCSP, 0x000c001fu, 0x0003000fu,
             0x000000ffu, 0x000000ffu, 0x0000003fu, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_MP_64_MAPBGA, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
             0x0007ffffu, 0x000000ffu, 0x07000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_DC_121_XFBGA, 0x000ff03fu, 0x00ff0f0fu,
             0x0007ffffu, 0x000000ffu, 0x07000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN12812, KINETIS_PACKAGE_AH_64_WLCSP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_MP_64_MAPBGA, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_AH_64_WLCSP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN256CAP12, KINETIS_PACKAGE_AP_80_WLCSP, 0x000ff03fu, 0x000f0c0fu,
             0x00030fffu, 0x000000ffu, 0x00000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
             0x0007ffffu, 0x000000ffu, 0x07000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_DC_121_XFBGA, 0x000ff03fu, 0x00ff0fcfu,
             0x000fffffu, 0x000000ffu, 0x07000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_MP_64_MAPBGA, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_AP_80_WLCSP, 0x000ff03fu, 0x000f0c0fu,
             0x00030fffu, 0x000000ffu, 0x00000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_BP_80_WLCSP, 0x000ff03fu, 0x000f0c0fu,
             0x00030fffu, 0x000000ffu, 0x00000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_FX_88_HVQFN, 0x000ff03fu, 0x000f0fcfu,
             0x000ff1ffu, 0x000000ffu, 0x00000000u, false, false),
    EXPECTED(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
             0x0007ffffu, 0x000000ffu, 0x07000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_DC_121_XFBGA, 0x200ffc3fu, 0x00ff0fcfu,
             0x000fffffu, 0x0000ffffu, 0x07000000u, true, true),
    EXPECTED(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LK_80_LQFP, 0x000ff03fu, 0x000f0c0fu,
             0x00030fffu, 0x000000ffu, 0x00000038u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
             0x0007ffffu, 0x000000ffu, 0x07000078u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_MC_121_MAPBGA, 0x200ffc3fu, 0x00ff3fcfu,
             0x000fffffu, 0x0000ffffu, 0x07000078u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LQ_144_LQFP, 0x3f0fffffu, 0x00ff0fffu,
             0x000fffffu, 0x0000ffffu, 0x1f001ff8u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_MD_144_MAPBGA, 0x3f0fffffu, 0x00ff0fffu,
             0x000fffffu, 0x0000ffffu, 0x1f001ff8u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
             0x00000fffu, 0x000000ffu, 0x00000000u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LK_80_LQFP, 0x000ff03fu, 0x000f0c0fu,
             0x00030fffu, 0x000000ffu, 0x00000038u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
             0x0007ffffu, 0x000000ffu, 0x07000078u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_MC_121_MAPBGA, 0x200ffc3fu, 0x00ff3fcfu,
             0x000fffffu, 0x0000ffffu, 0x07000078u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LQ_144_LQFP, 0x3f0fffffu, 0x00ff0fffu,
             0x000fffffu, 0x0000ffffu, 0x1f001ff8u, false, true),
    EXPECTED(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_MD_144_MAPBGA, 0x3f0fffffu, 0x00ff0fffu,
             0x000fffffu, 0x0000ffffu, 0x1f001ff8u, false, true),
};

static size_t expected_package_count(void) {
    return sizeof(expected_packages) / sizeof(expected_packages[0]);
}

static size_t expected_selection_count(void) {
    return sizeof(expected_selections) / sizeof(expected_selections[0]);
}

static const ExpectedSelection* expected_selection(KinetisProfile profile, KinetisPackage package) {
    for (size_t index = 0; index < expected_selection_count(); index++) {
        if (expected_selections[index].profile == profile &&
            expected_selections[index].package == package)
            return &expected_selections[index];
    }
    return NULL;
}

static void expect_package_metadata(TestState* state) {
    expect(state, expected_package_count() == KINETIS_PACKAGE_COUNT,
           "expected_package_count() == KINETIS_PACKAGE_COUNT");
    for (size_t index = 0; index < expected_package_count(); index++) {
        const ExpectedPackage* expected = &expected_packages[index];
        const KinetisPackageDescription* package = kinetis_package_get(expected->id);
        expect(state, package != NULL, "package != NULL");
        expect(state, package->id == expected->id, "package->id == expected->id");
        expect(state, strcmp(package->code, expected->code) == 0,
               "strcmp(package->code, expected->code) == 0");
        expect(state, strcmp(package->name, expected->name) == 0,
               "strcmp(package->name, expected->name) == 0");
        expect(state, package->terminal_count == expected->terminal_count,
               "package->terminal_count == expected->terminal_count");
        expect(state, package->pin_id == expected->pin_id, "package->pin_id == expected->pin_id");
        expect(state, kinetis_package_find(expected->code) == package,
               "kinetis_package_find(expected->code) == package");
        KinetisPackage public_package = KINETIS_PACKAGE_DEFAULT;
        expect(state, kinetis_package_from_code(expected->code, &public_package),
               "kinetis_package_from_code(expected->code, &public_package)");
        expect(state, public_package == (KinetisPackage)expected->id,
               "public_package == expected->id");
        expect(state, strcmp(kinetis_package_code(public_package), expected->code) == 0,
               "strcmp(kinetis_package_code(public_package), expected->code) == 0");
    }
}

static void expect_selection_data(TestState* state, const ExpectedSelection* expected,
                                  const KinetisPackageSelection* selected) {
    expect(state, selected != NULL, "selected != NULL");
    expect(state, kinetis_package_selection_profile(selected) == expected->profile,
           "kinetis_package_selection_profile(selected) == expected->profile");
    expect(state, kinetis_package_selection_package(selected)->id == expected->package,
           "kinetis_package_selection_package(selected)->id == expected->package");
    expect(state,
           kinetis_package_pin_id(selected) == kinetis_package_selection_package(selected)->pin_id,
           "kinetis_package_pin_id(selected) == selected package pin ID");
    for (uint8_t port = 0; port < KINETIS_PACKAGE_PORT_COUNT; port++) {
        expect(state,
               kinetis_package_port_pin_mask(selected, port) == expected->port_pin_mask[port],
               "kinetis_package_port_pin_mask(selected, port) == expected->port_pin_mask[port]");
        for (uint8_t pin = 0; pin < KINETIS_PACKAGE_PIN_COUNT; pin++) {
            const bool exists = (expected->port_pin_mask[port] & (UINT32_C(1) << pin)) != 0;
            expect(state, kinetis_package_pin_exists(selected, port, pin) == exists,
                   "kinetis_package_pin_exists(selected, port, pin) == exists");
        }
    }
    for (int peripheral = 0; peripheral < KINETIS_PERIPHERAL_COUNT; peripheral++) {
        bool exists = kinetis_profile_has_peripheral(kinetis_profile_get(expected->profile),
                                                     (KinetisPeripheralId)peripheral);
        if (peripheral == KINETIS_PERIPHERAL_DAC1)
            exists = expected->dac1;
        if (peripheral == KINETIS_PERIPHERAL_FB)
            exists = expected->flexbus;
        if (expected->profile == KINETIS_PROFILE_MKV30F12810 &&
            expected->package == KINETIS_PACKAGE_FM_32_QFN && peripheral == KINETIS_PERIPHERAL_VREF)
            exists = false;
        if (expected->profile == KINETIS_PROFILE_MK22FN1M012 ||
            expected->profile == KINETIS_PROFILE_MK22FX51212) {
            const bool at_least_80 = expected->package != KINETIS_PACKAGE_LH_64_LQFP;
            const bool at_least_100 = expected->package == KINETIS_PACKAGE_LL_100_LQFP ||
                                      expected->package == KINETIS_PACKAGE_MC_121_MAPBGA ||
                                      expected->package == KINETIS_PACKAGE_LQ_144_LQFP ||
                                      expected->package == KINETIS_PACKAGE_MD_144_MAPBGA;
            const bool at_least_121 = expected->package == KINETIS_PACKAGE_MC_121_MAPBGA ||
                                      expected->package == KINETIS_PACKAGE_LQ_144_LQFP ||
                                      expected->package == KINETIS_PACKAGE_MD_144_MAPBGA;
            if (peripheral == KINETIS_PERIPHERAL_SPI1 || peripheral == KINETIS_PERIPHERAL_UART3 ||
                peripheral == KINETIS_PERIPHERAL_SDHC)
                exists = at_least_80;
            if (peripheral == KINETIS_PERIPHERAL_SPI2 || peripheral == KINETIS_PERIPHERAL_UART4)
                exists = at_least_100;
            if (peripheral == KINETIS_PERIPHERAL_UART5 || peripheral == KINETIS_PERIPHERAL_DAC1)
                exists = at_least_121;
        }
        expect(
            state,
            kinetis_package_has_peripheral(selected, (KinetisPeripheralId)peripheral) == exists,
            "kinetis_package_has_peripheral( selected, (KinetisPeripheralId)peripheral) == exists");
    }
}

static void expect_all_combinations(TestState* state) {
    for (int profile_id = 0; profile_id < KINETIS_PROFILE_COUNT; profile_id++) {
        const KinetisDeviceProfile* profile = kinetis_profile_get((KinetisProfile)profile_id);
        expect(state, profile != NULL, "profile != NULL");
        for (int package_id = 0; package_id < KINETIS_PACKAGE_COUNT; package_id++) {
            const ExpectedSelection* expected =
                expected_selection((KinetisProfile)profile_id, (KinetisPackage)package_id);
            const KinetisPackageSelection* selected =
                kinetis_package_select(profile, (KinetisPackage)package_id);
            expect(state, (selected != NULL) == (expected != NULL),
                   "(selected != NULL) == (expected != NULL)");
            if (expected != NULL)
                expect_selection_data(state, expected, selected);
        }
    }
}

static void expect_defaults(TestState* state) {
    for (int profile_id = 0; profile_id < KINETIS_PROFILE_COUNT; profile_id++) {
        const KinetisDeviceProfile* profile = kinetis_profile_get((KinetisProfile)profile_id);
        const KinetisPackageSelection* selected = kinetis_package_default(profile);
        expect(state, selected != NULL, "selected != NULL");
        KinetisPackage expected = KINETIS_PACKAGE_LH_64_LQFP;
        if (profile_id == KINETIS_PROFILE_MK22FN12812)
            expected = KINETIS_PACKAGE_AH_64_WLCSP;
        if (profile_id == KINETIS_PROFILE_MK22FN256CAP12)
            expected = KINETIS_PACKAGE_AP_80_WLCSP;
        expect(state, kinetis_package_selection_package(selected)->id == expected,
               "kinetis_package_selection_package(selected)->id == expected");
    }
}

static void expect_fail_closed(TestState* state) {
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN51212);
    const KinetisPackageSelection* selected =
        kinetis_package_select(profile, KINETIS_PACKAGE_LH_64_LQFP);
    KinetisDeviceProfile invalid_profile = *profile;
    invalid_profile.id = KINETIS_PROFILE_COUNT;
    expect(state, kinetis_package_get((KinetisPackage)-1) == NULL,
           "kinetis_package_get((KinetisPackage)-1) == NULL");
    expect(state, kinetis_package_get(KINETIS_PACKAGE_COUNT) == NULL,
           "kinetis_package_get(KINETIS_PACKAGE_COUNT) == NULL");
    expect(state, kinetis_package_find(NULL) == NULL, "kinetis_package_find(NULL) == NULL");
    expect(state, kinetis_package_find("") == NULL, "kinetis_package_find(\"\") == NULL");
    expect(state, kinetis_package_find("lh") == NULL, "kinetis_package_find(\"lh\") == NULL");
    KinetisPackage public_package = KINETIS_PACKAGE_LH_64_LQFP;
    expect(state, !kinetis_package_from_code(NULL, &public_package),
           "!kinetis_package_from_code(NULL, &public_package)");
    expect(state, !kinetis_package_from_code("lh", &public_package),
           "!kinetis_package_from_code(\"lh\", &public_package)");
    expect(state, kinetis_package_code(KINETIS_PACKAGE_DEFAULT) == NULL,
           "kinetis_package_code(KINETIS_PACKAGE_DEFAULT) == NULL");
    expect(state, kinetis_package_select(NULL, KINETIS_PACKAGE_LH_64_LQFP) == NULL,
           "kinetis_package_select(NULL, KINETIS_PACKAGE_LH_64_LQFP) == NULL");
    expect(state, kinetis_package_select(&invalid_profile, KINETIS_PACKAGE_LH_64_LQFP) == NULL,
           "kinetis_package_select(&invalid_profile, KINETIS_PACKAGE_LH_64_LQFP) == NULL");
    expect(state, kinetis_package_select(profile, (KinetisPackage)-1) == NULL,
           "kinetis_package_select(profile, (KinetisPackage)-1) == NULL");
    expect(state, kinetis_package_select(profile, KINETIS_PACKAGE_COUNT) == NULL,
           "kinetis_package_select(profile, KINETIS_PACKAGE_COUNT) == NULL");
    expect(state, kinetis_package_default(NULL) == NULL, "kinetis_package_default(NULL) == NULL");
    expect(state, kinetis_package_default(&invalid_profile) == NULL,
           "kinetis_package_default(&invalid_profile) == NULL");
    expect(state, kinetis_package_selection_profile(NULL) == KINETIS_PROFILE_COUNT,
           "kinetis_package_selection_profile(NULL) == KINETIS_PROFILE_COUNT");
    expect(state, kinetis_package_selection_package(NULL) == NULL,
           "kinetis_package_selection_package(NULL) == NULL");
    expect(state, kinetis_package_pin_id(NULL) == 0, "kinetis_package_pin_id(NULL) == 0");
    expect(state, kinetis_package_port_pin_mask(NULL, 0) == 0,
           "kinetis_package_port_pin_mask(NULL, 0) == 0");
    expect(state, kinetis_package_port_pin_mask(selected, KINETIS_PACKAGE_PORT_COUNT) == 0,
           "kinetis_package_port_pin_mask(selected, KINETIS_PACKAGE_PORT_COUNT) == 0");
    expect(state, !kinetis_package_pin_exists(NULL, 0, 0),
           "!kinetis_package_pin_exists(NULL, 0, 0)");
    expect(state, !kinetis_package_pin_exists(selected, KINETIS_PACKAGE_PORT_COUNT, 0),
           "!kinetis_package_pin_exists(selected, KINETIS_PACKAGE_PORT_COUNT, 0)");
    expect(state, !kinetis_package_pin_exists(selected, 0, KINETIS_PACKAGE_PIN_COUNT),
           "!kinetis_package_pin_exists(selected, 0, KINETIS_PACKAGE_PIN_COUNT)");
    expect(state, !kinetis_package_adc_input_exists(NULL, 0u, KINETIS_ADC_MUX_A, 0u),
           "!kinetis_package_adc_input_exists(NULL, 0u, KINETIS_ADC_MUX_A, 0u)");
    expect(state,
           !kinetis_package_adc_input_exists(selected, 2u, KINETIS_ADC_MUX_A, 0u) &&
               !kinetis_package_adc_input_exists(selected, 0u, KINETIS_ADC_MUX_COUNT, 0u) &&
               !kinetis_package_adc_input_exists(selected, 0u, KINETIS_ADC_MUX_A, 31u) &&
               !kinetis_package_adc_input_exists(selected, 0u, KINETIS_ADC_MUX_B, 8u),
           "invalid ADC inputs are rejected");
    expect(state, !kinetis_package_has_peripheral(NULL, KINETIS_PERIPHERAL_DMA),
           "!kinetis_package_has_peripheral(NULL, KINETIS_PERIPHERAL_DMA)");
    expect(state, !kinetis_package_has_peripheral(selected, (KinetisPeripheralId)-1),
           "!kinetis_package_has_peripheral(selected, (KinetisPeripheralId)-1)");
    expect(state, !kinetis_package_has_peripheral(selected, KINETIS_PERIPHERAL_COUNT),
           "!kinetis_package_has_peripheral(selected, KINETIS_PERIPHERAL_COUNT)");
}

static void expect_kv30_adc_inputs(TestState* state) {
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MKV30F12810);
    const KinetisPackageSelection* fm = kinetis_package_select(profile, KINETIS_PACKAGE_FM_32_QFN);
    const KinetisPackageSelection* lf = kinetis_package_select(profile, KINETIS_PACKAGE_LF_48_LQFP);
    const KinetisPackageSelection* lh = kinetis_package_select(profile, KINETIS_PACKAGE_LH_64_LQFP);
    expect(state, kinetis_package_adc_input_exists(fm, 0u, KINETIS_ADC_MUX_A, 4u),
           "FM ADC0 channel 4A exists");
    expect(state, kinetis_package_adc_input_exists(fm, 0u, KINETIS_ADC_MUX_B, 4u),
           "FM ADC0 channel 4B exists");
    expect(state, !kinetis_package_adc_input_exists(fm, 0u, KINETIS_ADC_MUX_B, 5u),
           "FM ADC0 channel 5B is absent");
    expect(state, !kinetis_package_adc_input_exists(fm, 0u, KINETIS_ADC_MUX_A, 14u),
           "FM ADC0 channel 14 is absent");
    expect(state, kinetis_package_adc_input_exists(lf, 0u, KINETIS_ADC_MUX_A, 14u),
           "LF ADC0 channel 14 exists");
    expect(state, !kinetis_package_adc_input_exists(fm, 1u, KINETIS_ADC_MUX_A, 18u),
           "FM ADC1 channel 18 is absent");
    expect(state, kinetis_package_adc_input_exists(lf, 1u, KINETIS_ADC_MUX_A, 18u),
           "LF ADC1 channel 18 exists");
    expect(state, kinetis_package_adc_input_exists(lh, 1u, KINETIS_ADC_MUX_B, 7u),
           "LH ADC1 channel 7B exists");
    const uint8_t internal_channels[] = {26u, 27u, 29u, 30u};
    for (size_t index = 0u; index < sizeof(internal_channels); index++) {
        expect(
            state,
            kinetis_package_adc_input_exists(fm, 0u, KINETIS_ADC_MUX_A, internal_channels[index]) &&
                kinetis_package_adc_input_exists(fm, 1u, KINETIS_ADC_MUX_A,
                                                 internal_channels[index]),
            "FM internal ADC input exists");
    }
}

static void expect_k22_adc_inputs(TestState* state) {
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN51212);
    const KinetisPackageSelection* lh = kinetis_package_select(profile, KINETIS_PACKAGE_LH_64_LQFP);
    const KinetisPackageSelection* ll =
        kinetis_package_select(profile, KINETIS_PACKAGE_LL_100_LQFP);
    const KinetisPackageSelection* dc =
        kinetis_package_select(profile, KINETIS_PACKAGE_DC_121_XFBGA);
    expect(state, !kinetis_package_adc_input_exists(lh, 0u, KINETIS_ADC_MUX_A, 17u),
           "LH ADC0 channel 17 is absent");
    expect(state, !kinetis_package_adc_input_exists(lh, 1u, KINETIS_ADC_MUX_A, 23u),
           "LH ADC1 channel 23 is absent");
    expect(state, kinetis_package_adc_input_exists(ll, 0u, KINETIS_ADC_MUX_A, 17u),
           "LL ADC0 channel 17 exists");
    expect(state, kinetis_package_adc_input_exists(dc, 1u, KINETIS_ADC_MUX_A, 23u),
           "DC ADC1 channel 23 exists");

    profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN12810);
    const KinetisPackageSelection* ak =
        kinetis_package_select(profile, KINETIS_PACKAGE_AK_49_WLCSP);
    expect(state, !kinetis_package_adc_input_exists(ak, 0u, KINETIS_ADC_MUX_A, 0u),
           "AK ADC0 channel 0 is absent");
    expect(state, !kinetis_package_adc_input_exists(ak, 1u, KINETIS_ADC_MUX_A, 3u),
           "AK ADC1 channel 3 is absent");

    profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN12812);
    const KinetisPackageSelection* ah =
        kinetis_package_select(profile, KINETIS_PACKAGE_AH_64_WLCSP);
    expect(state, kinetis_package_adc_input_exists(ah, 0u, KINETIS_ADC_MUX_A, 1u),
           "AH ADC0 channel 1 exists");
    expect(state, !kinetis_package_adc_input_exists(ah, 0u, KINETIS_ADC_MUX_A, 3u),
           "AH ADC0 channel 3 is absent");
    expect(state, !kinetis_package_adc_input_exists(ah, 1u, KINETIS_ADC_MUX_A, 0u),
           "AH ADC1 channel 0 is absent");

    profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN51212);
    const KinetisPackageSelection* ap =
        kinetis_package_select(profile, KINETIS_PACKAGE_AP_80_WLCSP);
    const KinetisPackageSelection* bp =
        kinetis_package_select(profile, KINETIS_PACKAGE_BP_80_WLCSP);
    const KinetisPackageSelection* packages[] = {ap, bp};
    for (size_t index = 0u; index < sizeof(packages) / sizeof(packages[0]); index++) {
        expect(state, kinetis_package_adc_input_exists(packages[index], 0u, KINETIS_ADC_MUX_A, 2u),
               "AP/BP ADC0 channel 2 exists");
        expect(state, kinetis_package_adc_input_exists(packages[index], 1u, KINETIS_ADC_MUX_A, 1u),
               "AP/BP ADC1 channel 1 exists");
        expect(state, !kinetis_package_adc_input_exists(packages[index], 0u, KINETIS_ADC_MUX_A, 3u),
               "AP/BP ADC0 channel 3 is absent");
        expect(state, !kinetis_package_adc_input_exists(packages[index], 1u, KINETIS_ADC_MUX_A, 0u),
               "AP/BP ADC1 channel 0 is absent");
    }
}

int main(void) {
    TestState state = {0};
    expect_package_metadata(&state);
    expect_all_combinations(&state);
    expect_defaults(&state);
    expect_fail_closed(&state);
    expect_kv30_adc_inputs(&state);
    expect_k22_adc_inputs(&state);
    return test_finish(&state);
}
