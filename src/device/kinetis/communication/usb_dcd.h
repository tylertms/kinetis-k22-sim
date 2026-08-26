#ifndef KINETIS_SIM_USBDCD_H
#define KINETIS_SIM_USBDCD_H

#include <stdbool.h>
#include <stdint.h>

#include "kinetis.h"

typedef enum {
    KINETIS_USBDCD_IDLE,
    KINETIS_USBDCD_DATA_CONTACT,
    KINETIS_USBDCD_PRIMARY_DELAY,
    KINETIS_USBDCD_PRIMARY_DETECTION,
    KINETIS_USBDCD_WAIT_PULLUP,
    KINETIS_USBDCD_SECONDARY_DELAY,
    KINETIS_USBDCD_SECONDARY_DETECTION,
} KinetisUsbDcdPhase;

typedef struct {
    uint32_t control;
    uint32_t clock;
    uint32_t status;
    uint32_t timer0;
    uint32_t timer1;
    uint32_t timer2;
    uint64_t clock_cycles;
    uint16_t phase_elapsed;
    KinetisUsbDcdPhase phase;
    KinetisUsbCharger charger;
    bool pullup;
} KinetisUsbDcd;

void kinetis_usbdcd_reset(KinetisUsbDcd* usbdcd);
bool kinetis_usbdcd_copy(KinetisUsbDcd* destination, const KinetisUsbDcd* source);
bool kinetis_usbdcd_read(KinetisUsbDcd* usbdcd, uint32_t address, uint8_t byte_count,
                         uint32_t* output_value);
bool kinetis_usbdcd_write(KinetisUsbDcd* usbdcd, uint32_t address, uint8_t byte_count,
                          uint32_t write_value);
void kinetis_usbdcd_advance(KinetisUsbDcd* usbdcd, uint64_t cycles);
bool kinetis_usbdcd_set_charger(KinetisUsbDcd* usbdcd, KinetisUsbCharger charger);
bool kinetis_usbdcd_set_pullup(KinetisUsbDcd* usbdcd, bool enabled);
bool kinetis_usbdcd_irq(const KinetisUsbDcd* usbdcd);

#endif
