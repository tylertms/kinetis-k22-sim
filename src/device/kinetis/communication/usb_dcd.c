#include "device/kinetis/communication/usb_dcd.h"

#include <string.h>

enum {
    USBDCD_BASE = 0x40035000u,
    USBDCD_CONTROL = 0x00u,
    USBDCD_CLOCK = 0x04u,
    USBDCD_STATUS = 0x08u,
    USBDCD_TIMER0 = 0x10u,
    USBDCD_TIMER1 = 0x14u,
    USBDCD_TIMER2 = 0x18u,
    CONTROL_IACK = 1u << 0u,
    CONTROL_IF = 1u << 8u,
    CONTROL_IE = 1u << 16u,
    CONTROL_BC12 = 1u << 17u,
    CONTROL_START = 1u << 24u,
    CONTROL_SR = 1u << 25u,
    STATUS_RESULT_SHIFT = 16u,
    STATUS_PHASE_SHIFT = 18u,
    STATUS_ERROR = 1u << 20u,
    STATUS_TIMEOUT = 1u << 21u,
    STATUS_ACTIVE = 1u << 22u,
};

static uint32_t* selected_register(KinetisUsbDcd* usbdcd, uint32_t register_offset) {
    switch (register_offset) {
    case USBDCD_CONTROL:
        return &usbdcd->control;
    case USBDCD_CLOCK:
        return &usbdcd->clock;
    case USBDCD_STATUS:
        return &usbdcd->status;
    case USBDCD_TIMER0:
        return &usbdcd->timer0;
    case USBDCD_TIMER1:
        return &usbdcd->timer1;
    case USBDCD_TIMER2:
        return &usbdcd->timer2;
    default:
        return NULL;
    }
}

static void software_reset(KinetisUsbDcd* usbdcd) {
    usbdcd->control &= CONTROL_IE | CONTROL_BC12 | CONTROL_START;
    usbdcd->status = 0u;
    usbdcd->timer0 &= 0x03ff0000u;
    usbdcd->clock_cycles = 0u;
    usbdcd->phase_elapsed = 0u;
    usbdcd->phase = KINETIS_USBDCD_IDLE;
}

void kinetis_usbdcd_reset(KinetisUsbDcd* usbdcd) {
    if (usbdcd == NULL)
        return;
    memset(usbdcd, 0, sizeof(*usbdcd));
    usbdcd->control = CONTROL_IE;
    usbdcd->clock = 0x000000c1u;
    usbdcd->timer0 = 0x00100000u;
    usbdcd->timer1 = 0x000a0028u;
    usbdcd->timer2 = 0x00280001u;
    usbdcd->charger = KINETIS_USB_CHARGER_NONE;
}

bool kinetis_usbdcd_copy(KinetisUsbDcd* destination, const KinetisUsbDcd* source) {
    if (destination == NULL || source == NULL)
        return false;
    *destination = *source;
    return true;
}

bool kinetis_usbdcd_read(KinetisUsbDcd* usbdcd, uint32_t address, uint8_t byte_count,
                         uint32_t* output_value) {
    if (usbdcd == NULL || output_value == NULL || byte_count != 4u || address < USBDCD_BASE ||
        (address & 3u) != 0u)
        return false;
    uint32_t* register_pointer = selected_register(usbdcd, address - USBDCD_BASE);
    if (register_pointer == NULL)
        return false;
    *output_value = *register_pointer;
    return true;
}

static void start(KinetisUsbDcd* usbdcd) {
    usbdcd->control |= CONTROL_START;
    usbdcd->status = STATUS_ACTIVE;
    usbdcd->timer0 = (usbdcd->timer0 & 0x03ff0000u) | (usbdcd->timer0 >> 16u);
    usbdcd->clock_cycles = 0u;
    usbdcd->phase_elapsed = 0u;
    usbdcd->phase = KINETIS_USBDCD_DATA_CONTACT;
}

bool kinetis_usbdcd_write(KinetisUsbDcd* usbdcd, uint32_t address, uint8_t byte_count,
                          uint32_t write_value) {
    if (usbdcd == NULL || byte_count != 4u || address < USBDCD_BASE || (address & 3u) != 0u)
        return false;
    const uint32_t register_offset = address - USBDCD_BASE;
    if (register_offset == USBDCD_CONTROL) {
        if ((write_value & CONTROL_IACK) != 0u)
            usbdcd->control &= ~CONTROL_IF;
        if ((write_value & CONTROL_SR) != 0u) {
            software_reset(usbdcd);
            return true;
        }
        const uint32_t configuration = write_value & (CONTROL_IE | CONTROL_BC12);
        usbdcd->control = (usbdcd->control & (CONTROL_IF | CONTROL_START)) | configuration;
        if ((write_value & CONTROL_START) != 0u && (usbdcd->status & STATUS_ACTIVE) == 0u)
            start(usbdcd);
        return true;
    }
    if ((usbdcd->status & STATUS_ACTIVE) != 0u)
        return false;
    if (register_offset == USBDCD_CLOCK) {
        usbdcd->clock = write_value & 0x00000ffdu;
        return true;
    }
    if (register_offset == USBDCD_TIMER0) {
        usbdcd->timer0 = write_value & 0x03ff0000u;
        return true;
    }
    if (register_offset == USBDCD_TIMER1) {
        usbdcd->timer1 = write_value & 0x03ff03ffu;
        return true;
    }
    if (register_offset == USBDCD_TIMER2) {
        usbdcd->timer2 = write_value & 0x03ff03ffu;
        return true;
    }
    return false;
}

static uint32_t clock_cycles_per_millisecond(const KinetisUsbDcd* usbdcd) {
    const uint32_t speed = (usbdcd->clock >> 2u) & 0x3ffu;
    if (speed == 0u)
        return 1u;
    return (usbdcd->clock & 1u) != 0u ? speed * 1000u : speed;
}

static void set_phase(KinetisUsbDcd* usbdcd, KinetisUsbDcdPhase phase) {
    usbdcd->phase = phase;
    usbdcd->phase_elapsed = 0u;
}

static void signal_interrupt(KinetisUsbDcd* usbdcd) { usbdcd->control |= CONTROL_IF; }

static void finish(KinetisUsbDcd* usbdcd, uint8_t phase, uint8_t result, bool error) {
    const uint32_t timeout = usbdcd->status & STATUS_TIMEOUT;
    usbdcd->status = timeout | ((uint32_t)phase << STATUS_PHASE_SHIFT) |
                     ((uint32_t)result << STATUS_RESULT_SHIFT) |
                     (error || timeout != 0u ? STATUS_ERROR : 0u);
    usbdcd->phase = KINETIS_USBDCD_IDLE;
    signal_interrupt(usbdcd);
}

static void primary_result(KinetisUsbDcd* usbdcd) {
    if (usbdcd->charger == KINETIS_USB_CHARGER_STANDARD_HOST) {
        finish(usbdcd, 2u, 1u, false);
        return;
    }
    if (usbdcd->charger == KINETIS_USB_CHARGER_ERROR) {
        finish(usbdcd, 2u, 0u, true);
        return;
    }
    set_phase(usbdcd, KINETIS_USBDCD_SECONDARY_DELAY);
}

static void charging_port_result(KinetisUsbDcd* usbdcd) {
    usbdcd->status = (usbdcd->status & (STATUS_TIMEOUT | STATUS_ERROR)) | STATUS_ACTIVE |
                     (2u << STATUS_PHASE_SHIFT) | (2u << STATUS_RESULT_SHIFT);
    signal_interrupt(usbdcd);
    set_phase(usbdcd, (usbdcd->control & CONTROL_BC12) != 0u ? KINETIS_USBDCD_SECONDARY_DETECTION
                                                             : KINETIS_USBDCD_WAIT_PULLUP);
}

static void advance_millisecond(KinetisUsbDcd* usbdcd) {
    uint16_t elapsed = (uint16_t)(usbdcd->timer0 & 0x0fffu);
    if (elapsed < 0x0fffu)
        elapsed++;
    usbdcd->timer0 = (usbdcd->timer0 & 0x03ff0000u) | elapsed;
    if (elapsed >= 1000u && (usbdcd->status & STATUS_TIMEOUT) == 0u) {
        usbdcd->status |= STATUS_TIMEOUT | STATUS_ERROR;
        signal_interrupt(usbdcd);
    }
    usbdcd->phase_elapsed++;
    switch (usbdcd->phase) {
    case KINETIS_USBDCD_DATA_CONTACT:
        if (usbdcd->charger == KINETIS_USB_CHARGER_NONE) {
            usbdcd->phase_elapsed = 0u;
        } else if (usbdcd->phase_elapsed >= ((usbdcd->timer1 >> 16u) & 0x3ffu)) {
            usbdcd->status = (usbdcd->status & (STATUS_TIMEOUT | STATUS_ERROR)) | STATUS_ACTIVE |
                             (1u << STATUS_PHASE_SHIFT);
            set_phase(usbdcd, KINETIS_USBDCD_PRIMARY_DELAY);
        }
        break;
    case KINETIS_USBDCD_PRIMARY_DELAY:
        if (usbdcd->phase_elapsed >= 1u)
            set_phase(usbdcd, KINETIS_USBDCD_PRIMARY_DETECTION);
        break;
    case KINETIS_USBDCD_PRIMARY_DETECTION:
        if (usbdcd->phase_elapsed >= (usbdcd->timer1 & 0x3ffu))
            primary_result(usbdcd);
        break;
    case KINETIS_USBDCD_WAIT_PULLUP:
        if (usbdcd->pullup)
            set_phase(usbdcd, KINETIS_USBDCD_SECONDARY_DETECTION);
        break;
    case KINETIS_USBDCD_SECONDARY_DELAY:
        if (usbdcd->phase_elapsed >= ((usbdcd->timer2 >> 16u) & 0x3ffu))
            charging_port_result(usbdcd);
        break;
    case KINETIS_USBDCD_SECONDARY_DETECTION: {
        const uint16_t duration = (usbdcd->control & CONTROL_BC12) != 0u
                                      ? (uint16_t)(usbdcd->timer2 & 0x3ffu)
                                      : (uint16_t)(usbdcd->timer2 & 0x0fu);
        if (usbdcd->phase_elapsed >= duration)
            finish(usbdcd, 3u, usbdcd->charger == KINETIS_USB_CHARGER_DEDICATED ? 3u : 2u, false);
        break;
    }
    case KINETIS_USBDCD_IDLE:
        break;
    }
}

void kinetis_usbdcd_advance(KinetisUsbDcd* usbdcd, uint64_t cycles) {
    if (usbdcd == NULL || cycles == 0u || (usbdcd->status & STATUS_ACTIVE) == 0u)
        return;
    usbdcd->clock_cycles += cycles;
    const uint32_t period = clock_cycles_per_millisecond(usbdcd);
    while (usbdcd->clock_cycles >= period) {
        usbdcd->clock_cycles -= period;
        advance_millisecond(usbdcd);
    }
}

bool kinetis_usbdcd_set_charger(KinetisUsbDcd* usbdcd, KinetisUsbCharger charger) {
    if (usbdcd == NULL || charger > KINETIS_USB_CHARGER_ERROR)
        return false;
    usbdcd->charger = charger;
    return true;
}

bool kinetis_usbdcd_set_pullup(KinetisUsbDcd* usbdcd, bool enabled) {
    if (usbdcd == NULL)
        return false;
    usbdcd->pullup = enabled;
    return true;
}

bool kinetis_usbdcd_irq(const KinetisUsbDcd* usbdcd) {
    if (usbdcd == NULL)
        return false;
    return (usbdcd->control & (CONTROL_IF | CONTROL_IE)) == (CONTROL_IF | CONTROL_IE);
}
