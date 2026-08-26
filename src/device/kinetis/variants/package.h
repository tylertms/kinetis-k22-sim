#ifndef KINETIS_SIM_PACKAGE_H
#define KINETIS_SIM_PACKAGE_H

#include <stdbool.h>
#include <stdint.h>

#include "device/kinetis/variants/profile.h"

enum {
    KINETIS_PACKAGE_PORT_COUNT = 5,
    KINETIS_PACKAGE_PIN_COUNT = 32,
};

typedef struct {
    KinetisPackage id;
    const char* code;
    const char* name;
    uint16_t terminal_count;
    uint8_t pin_id;
} KinetisPackageDescription;

typedef struct KinetisPackageSelection KinetisPackageSelection;

const KinetisPackageDescription* kinetis_package_get(KinetisPackage id);
const KinetisPackageDescription* kinetis_package_find(const char* code);
const KinetisPackageSelection* kinetis_package_select(const KinetisDeviceProfile* profile,
                                                      KinetisPackage id);
const KinetisPackageSelection* kinetis_package_default(const KinetisDeviceProfile* profile);
KinetisProfile kinetis_package_selection_profile(const KinetisPackageSelection* selection);
const KinetisPackageDescription*
kinetis_package_selection_package(const KinetisPackageSelection* selection);
uint8_t kinetis_package_pin_id(const KinetisPackageSelection* selection);
uint32_t kinetis_package_port_pin_mask(const KinetisPackageSelection* selection, uint8_t port);
bool kinetis_package_pin_exists(const KinetisPackageSelection* selection, uint8_t port,
                                uint8_t pin);
bool kinetis_package_has_peripheral(const KinetisPackageSelection* selection,
                                    KinetisPeripheralId peripheral);

#endif
