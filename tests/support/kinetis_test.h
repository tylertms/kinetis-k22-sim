#ifndef KINETIS_SIM_TEST_H
#define KINETIS_SIM_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "kinetis.h"

static inline uint32_t kinetis_test_core_cycles_for_bus_cycles(const Kinetis* device,
                                                               uint32_t bus_cycles) {
    const uint64_t core = kinetis_core_clock_hz(device);
    const uint64_t bus = kinetis_bus_clock_hz(device);
    return bus == 0u ? 0u : (uint32_t)((bus_cycles * core + bus - 1u) / bus);
}

static inline bool kinetis_test_write16(Kinetis* device, uint32_t address, uint16_t value) {
    return kinetis_write(device, address, &value, sizeof(value));
}

static inline bool kinetis_test_disable_watchdog(Kinetis* device) {
    const uint32_t control = 0x40052000u;
    const uint32_t unlock = 0x4005200eu;
    if (!kinetis_test_write16(device, unlock, 0xc520u) ||
        !kinetis_test_write16(device, unlock, 0xd928u) ||
        !kinetis_test_write16(device, control, 0u))
        return false;
    kinetis_advance(device, kinetis_test_core_cycles_for_bus_cycles(device, 260u));
    uint16_t value = UINT16_MAX;
    return kinetis_read(device, control, &value, sizeof(value)) && (value & 1u) == 0u;
}

#endif
