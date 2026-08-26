#ifndef KINETIS_SIM_GPIO_INTERNAL_H
#define KINETIS_SIM_GPIO_INTERNAL_H

#include "device/kinetis/gpio/io.h"

#include <string.h>

enum {
    KINETIS_BIT_BAND_BASE = 0x42000000u,
    KINETIS_BIT_BAND_LIMIT = 0x44000000u,
    KINETIS_PERIPHERAL_BASE = 0x40000000u,
    KINETIS_PORT_PCR_ISF = 1u << 24,
    KINETIS_PORT_PCR_LK = 1u << 15,
    KINETIS_PORT_PCR_ODE = 1u << 5,
    KINETIS_PORT_PCR_PE = 1u << 1,
    KINETIS_PORT_PCR_PS = 1u,
    KINETIS_PORT_PCR_WRITABLE = 0x010f87f7u,
    KINETIS_USB_ISTAT = 0x80,
    KINETIS_USB_INTEN = 0x84,
    KINETIS_USB_STAT = 0x90,
    KINETIS_USB_CTL = 0x94,
    KINETIS_USB_FRMNUML = 0xa0,
    KINETIS_USB_FRMNUMH = 0xa4,
    KINETIS_CAN_MCR = 0,
    KINETIS_CAN_TIMER = 0x08,
    KINETIS_CAN_RXMGMASK = 0x10,
    KINETIS_CAN_RX14MASK = 0x14,
    KINETIS_CAN_RX15MASK = 0x18,
    KINETIS_CAN_IMASK1 = 0x28,
    KINETIS_CAN_IFLAG1 = 0x30,
    KINETIS_CAN_MB_BASE = 0x80,
    KINETIS_I2S_TCSR = 0,
    KINETIS_I2S_TDR0 = 0x20,
    KINETIS_I2S_RCSR = 0x80,
    KINETIS_I2S_RDR0 = 0xa0,
    KINETIS_I2S_REQUEST_FLAG = 1u << 16,
    KINETIS_I2S_FIFO_ERROR = 1u << 18,
};

#define KINETIS_I2S_ENABLE UINT32_C(0x80000000)

bool kinetis_io_internal_fifo_pop(uint32_t* fifo, uint8_t* read_index, uint8_t* entry_count,
                                  uint32_t* output_value);
bool kinetis_io_internal_fifo_push(uint32_t* fifo, uint8_t* write_index, uint8_t* entry_count,
                                   uint32_t entry_value);
bool kinetis_io_internal_is_gpio(KinetisPeripheralId id);
bool kinetis_io_internal_is_port(KinetisPeripheralId id);
bool kinetis_io_internal_module_clocked(const KinetisIo* io, KinetisPeripheralId id);
bool kinetis_io_internal_pin_exists(const KinetisIo* io, uint8_t port, uint8_t pin);
bool kinetis_io_internal_read_direct(KinetisIo* io, uint32_t address, uint8_t size,
                                     uint32_t* output_value);
bool kinetis_io_internal_read_gpio(KinetisIo* io, KinetisPeripheralLocation location, uint8_t size,
                                   uint32_t* output_value);
bool kinetis_io_internal_read_port(KinetisIo* io, KinetisPeripheralLocation location, uint8_t size,
                                   uint32_t* output_value);
bool kinetis_io_internal_valid_size(uint8_t size);
bool kinetis_io_internal_write_direct(KinetisIo* io, uint32_t address, uint8_t size,
                                      uint32_t write_value);
bool kinetis_io_internal_write_gpio(KinetisIo* io, KinetisPeripheralLocation location, uint8_t size,
                                    uint32_t write_value);
bool kinetis_io_internal_write_port(KinetisIo* io, KinetisPeripheralLocation location, uint8_t size,
                                    uint32_t write_value);
uint32_t kinetis_io_internal_load_bytes(const uint8_t* data, uint8_t size);
uint32_t kinetis_io_internal_pin_level_unfiltered(const KinetisIo* io, uint8_t port);
uint32_t kinetis_io_internal_pin_level(const KinetisIo* io, uint8_t port);
uint8_t kinetis_io_internal_first_set_bit(uint32_t value);
void kinetis_io_internal_commit_pin_level(KinetisIo* io, uint8_t port, uint8_t pin, bool previous,
                                          bool high);
void kinetis_io_internal_emit_can(KinetisIo* io, uint8_t mailbox, uint32_t identifier,
                                  uint32_t control, uint32_t high, uint32_t low);
void kinetis_io_internal_emit(KinetisIo* io, KinetisIoEventType type, uint32_t source,
                              uint32_t value, uint32_t auxiliary);
void kinetis_io_internal_update_i2s_requests(KinetisIo* io);

#endif
