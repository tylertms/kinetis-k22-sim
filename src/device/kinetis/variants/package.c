#include "device/kinetis/variants/package.h"

#include <stddef.h>
#include <string.h>

#define MASKS(a, b, c, d, e) {a, b, c, d, e}
#define SELECTION(profile_id, package_id, a, b, c, d, e)                                           \
    {profile_id, package_id, MASKS(a, b, c, d, e)}

struct KinetisPackageSelection {
    KinetisProfile profile;
    KinetisPackage package;
    uint32_t port_pin_mask[KINETIS_PACKAGE_PORT_COUNT];
};

static const KinetisPackageDescription packages[KINETIS_PACKAGE_COUNT] = {
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

static const KinetisPackageSelection selections[] = {
    SELECTION(KINETIS_PROFILE_MKV30F12810, KINETIS_PACKAGE_FM_32_QFN, 0x000c001fu, 0x00000003u,
              0x000000feu, 0x000000f0u, 0x030f0000u),
    SELECTION(KINETIS_PROFILE_MKV30F12810, KINETIS_PACKAGE_LF_48_LQFP, 0x000c001fu, 0x0003000fu,
              0x000000ffu, 0x000000ffu, 0x030f0000u),
    SELECTION(KINETIS_PROFILE_MKV30F12810, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x030f0003u),
    SELECTION(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_AK_49_WLCSP, 0x000c001fu, 0x0003000fu,
              0x000000ffu, 0x000000ffu, 0x0000003fu),
    SELECTION(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_MP_64_MAPBGA, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
              0x0007ffffu, 0x000000ffu, 0x07000000u),
    SELECTION(KINETIS_PROFILE_MK22FN12810, KINETIS_PACKAGE_DC_121_XFBGA, 0x000ff03fu, 0x00ff0f0fu,
              0x0007ffffu, 0x000000ffu, 0x07000000u),
    SELECTION(KINETIS_PROFILE_MK22FN12812, KINETIS_PACKAGE_AH_64_WLCSP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_MP_64_MAPBGA, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_AH_64_WLCSP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_AP_80_WLCSP, 0x000ff03fu, 0x000f0c0fu,
              0x00030fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
              0x0007ffffu, 0x000000ffu, 0x07000000u),
    SELECTION(KINETIS_PROFILE_MK22FN25612, KINETIS_PACKAGE_DC_121_XFBGA, 0x000ff03fu, 0x00ff0fcfu,
              0x000fffffu, 0x000000ffu, 0x07000000u),
    SELECTION(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_MP_64_MAPBGA, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_AP_80_WLCSP, 0x000ff03fu, 0x000f0c0fu,
              0x00030fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_BP_80_WLCSP, 0x000ff03fu, 0x000f0c0fu,
              0x00030fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_FX_88_HVQFN, 0x000ff03fu, 0x000f0fcfu,
              0x000ff1ffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
              0x0007ffffu, 0x000000ffu, 0x07000000u),
    SELECTION(KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_DC_121_XFBGA, 0x200ffc3fu, 0x00ff0fcfu,
              0x000fffffu, 0x0000ffffu, 0x07000000u),
    SELECTION(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LK_80_LQFP, 0x000ff03fu, 0x000f0c0fu,
              0x00030fffu, 0x000000ffu, 0x00000038u),
    SELECTION(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
              0x0007ffffu, 0x000000ffu, 0x07000078u),
    SELECTION(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_MC_121_MAPBGA, 0x200ffc3fu, 0x00ff3fcfu,
              0x000fffffu, 0x0000ffffu, 0x07000078u),
    SELECTION(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LQ_144_LQFP, 0x3f0fffffu, 0x00ff0fffu,
              0x000fffffu, 0x0000ffffu, 0x1f001ff8u),
    SELECTION(KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_MD_144_MAPBGA, 0x3f0fffffu, 0x00ff0fffu,
              0x000fffffu, 0x0000ffffu, 0x1f001ff8u),
    SELECTION(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LH_64_LQFP, 0x000c303fu, 0x000f000fu,
              0x00000fffu, 0x000000ffu, 0x00000000u),
    SELECTION(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LK_80_LQFP, 0x000ff03fu, 0x000f0c0fu,
              0x00030fffu, 0x000000ffu, 0x00000038u),
    SELECTION(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LL_100_LQFP, 0x000ff03fu, 0x00ff0e0fu,
              0x0007ffffu, 0x000000ffu, 0x07000078u),
    SELECTION(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_MC_121_MAPBGA, 0x200ffc3fu, 0x00ff3fcfu,
              0x000fffffu, 0x0000ffffu, 0x07000078u),
    SELECTION(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_LQ_144_LQFP, 0x3f0fffffu, 0x00ff0fffu,
              0x000fffffu, 0x0000ffffu, 0x1f001ff8u),
    SELECTION(KINETIS_PROFILE_MK22FX51212, KINETIS_PACKAGE_MD_144_MAPBGA, 0x3f0fffffu, 0x00ff0fffu,
              0x000fffffu, 0x0000ffffu, 0x1f001ff8u),
};

static size_t package_selection_count(void) { return sizeof(selections) / sizeof(selections[0]); }

const KinetisPackageDescription* kinetis_package_get(KinetisPackage id) {
    if ((unsigned)id >= KINETIS_PACKAGE_COUNT)
        return NULL;
    return &packages[id];
}

const KinetisPackageDescription* kinetis_package_find(const char* code) {
    if (code == NULL)
        return NULL;
    for (size_t index = 0; index < KINETIS_PACKAGE_COUNT; index++) {
        if (strcmp(packages[index].code, code) == 0)
            return &packages[index];
    }
    return NULL;
}

const KinetisPackageSelection* kinetis_package_select(const KinetisDeviceProfile* profile,
                                                      KinetisPackage id) {
    if (profile == NULL || (unsigned)profile->id >= KINETIS_PROFILE_COUNT ||
        (unsigned)id >= KINETIS_PACKAGE_COUNT)
        return NULL;
    for (size_t index = 0; index < package_selection_count(); index++) {
        if (selections[index].profile == profile->id && selections[index].package == id)
            return &selections[index];
    }
    return NULL;
}

const KinetisPackageSelection* kinetis_package_default(const KinetisDeviceProfile* profile) {
    if (profile == NULL)
        return NULL;
    if (profile->id == KINETIS_PROFILE_MK22FN12812)
        return kinetis_package_select(profile, KINETIS_PACKAGE_AH_64_WLCSP);
    return kinetis_package_select(profile, KINETIS_PACKAGE_LH_64_LQFP);
}

KinetisProfile kinetis_package_selection_profile(const KinetisPackageSelection* selection) {
    return selection == NULL ? KINETIS_PROFILE_COUNT : selection->profile;
}

const KinetisPackageDescription*
kinetis_package_selection_package(const KinetisPackageSelection* selection) {
    return selection == NULL ? NULL : kinetis_package_get(selection->package);
}

uint8_t kinetis_package_pin_id(const KinetisPackageSelection* selection) {
    const KinetisPackageDescription* package = kinetis_package_selection_package(selection);
    return package == NULL ? 0 : package->pin_id;
}

uint32_t kinetis_package_port_pin_mask(const KinetisPackageSelection* selection, uint8_t port) {
    if (selection == NULL || port >= KINETIS_PACKAGE_PORT_COUNT)
        return 0;
    return selection->port_pin_mask[port];
}

bool kinetis_package_pin_exists(const KinetisPackageSelection* selection, uint8_t port,
                                uint8_t pin) {
    return selection != NULL && port < KINETIS_PACKAGE_PORT_COUNT &&
           pin < KINETIS_PACKAGE_PIN_COUNT &&
           (selection->port_pin_mask[port] & (UINT32_C(1) << pin)) != 0;
}

static uint8_t kv30_package_index(KinetisPackage package) {
    switch (package) {
    case KINETIS_PACKAGE_LH_64_LQFP:
        return 0u;
    case KINETIS_PACKAGE_LF_48_LQFP:
        return 1u;
    case KINETIS_PACKAGE_FM_32_QFN:
        return 2u;
    default:
        return 3u;
    }
}

bool kinetis_package_adc_input_exists(const KinetisPackageSelection* selection, uint8_t instance,
                                      KinetisAdcMux mux, uint8_t channel) {
    if (selection == NULL || instance >= 2u || (unsigned)mux >= KINETIS_ADC_MUX_COUNT ||
        channel >= 31u || (mux == KINETIS_ADC_MUX_B && (channel < 4u || channel > 7u)))
        return false;
    if (selection->profile != KINETIS_PROFILE_MKV30F12810)
        return true;

    static const uint32_t inputs[3][2][KINETIS_ADC_MUX_COUNT] = {
        {{0x0086f3ffu, 0x000000f0u}, {0x0084033fu, 0x000000f0u}},
        {{0x0086f3f7u, 0x000000f0u}, {0x0004030eu, 0x00000000u}},
        {{0x008683f6u, 0x000000d0u}, {0x00040306u, 0x00000000u}},
    };
    const uint8_t package_index = kv30_package_index(selection->package);
    return package_index < 3u &&
           (inputs[package_index][instance][mux] & (UINT32_C(1) << channel)) != 0u;
}

bool kinetis_package_has_peripheral(const KinetisPackageSelection* selection,
                                    KinetisPeripheralId peripheral) {
    if (selection == NULL || (unsigned)peripheral >= KINETIS_PERIPHERAL_COUNT)
        return false;
    const KinetisDeviceProfile* profile = kinetis_profile_get(selection->profile);
    if (!kinetis_profile_has_peripheral(profile, peripheral))
        return false;
    if (selection->profile == KINETIS_PROFILE_MKV30F12810 &&
        selection->package == KINETIS_PACKAGE_FM_32_QFN && peripheral == KINETIS_PERIPHERAL_VREF)
        return false;
    if (selection->profile == KINETIS_PROFILE_MK22FN51212 && peripheral == KINETIS_PERIPHERAL_DAC1)
        return selection->package == KINETIS_PACKAGE_DC_121_XFBGA;
    if (selection->profile == KINETIS_PROFILE_MK22FN51212 && peripheral == KINETIS_PERIPHERAL_FB)
        return selection->package != KINETIS_PACKAGE_FX_88_HVQFN;
    if (selection->profile == KINETIS_PROFILE_MK22FN1M012 ||
        selection->profile == KINETIS_PROFILE_MK22FX51212) {
        const bool supports_80_pins = selection->package != KINETIS_PACKAGE_LH_64_LQFP;
        const bool supports_100_pins = selection->package == KINETIS_PACKAGE_LL_100_LQFP ||
                                       selection->package == KINETIS_PACKAGE_MC_121_MAPBGA ||
                                       selection->package == KINETIS_PACKAGE_LQ_144_LQFP ||
                                       selection->package == KINETIS_PACKAGE_MD_144_MAPBGA;
        const bool supports_121_pins = selection->package == KINETIS_PACKAGE_MC_121_MAPBGA ||
                                       selection->package == KINETIS_PACKAGE_LQ_144_LQFP ||
                                       selection->package == KINETIS_PACKAGE_MD_144_MAPBGA;
        if (peripheral == KINETIS_PERIPHERAL_SPI1 || peripheral == KINETIS_PERIPHERAL_UART3 ||
            peripheral == KINETIS_PERIPHERAL_SDHC)
            return supports_80_pins;
        if (peripheral == KINETIS_PERIPHERAL_SPI2 || peripheral == KINETIS_PERIPHERAL_UART4)
            return supports_100_pins;
        if (peripheral == KINETIS_PERIPHERAL_UART5 || peripheral == KINETIS_PERIPHERAL_DAC1)
            return supports_121_pins;
    }
    return true;
}
