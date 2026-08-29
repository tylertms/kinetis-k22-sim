#ifndef KINETIS_SIM_IO_H
#define KINETIS_SIM_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device/kinetis/variants/profile.h"

enum {
    KINETIS_IO_PORT_COUNT = 5,
    KINETIS_IO_PIN_COUNT = 32,
    KINETIS_IO_FIFO_CAPACITY = 16,
};

typedef enum {
    KINETIS_IO_EVENT_IRQ,
    KINETIS_IO_EVENT_DMA,
    KINETIS_IO_EVENT_GPIO_OUTPUT,
    KINETIS_IO_EVENT_USB_TOKEN,
    KINETIS_IO_EVENT_CAN_TRANSMIT,
    KINETIS_IO_EVENT_I2S_TRANSMIT,
    KINETIS_IO_EVENT_FLEXBUS_TRANSFER,
    KINETIS_IO_EVENT_ACCESS_ERROR,
} KinetisIoEventType;

typedef struct {
    KinetisIoEventType type;
    uint32_t source;
    uint32_t value;
    uint32_t auxiliary;
    uint8_t data[8];
    uint8_t length;
    bool extended;
    bool remote;
} KinetisIoEvent;

typedef void (*KinetisIoEventHandler)(void* context, const KinetisIoEvent* event);

typedef struct {
    const KinetisDeviceProfile* profile;
    uint32_t package_pin_mask[KINETIS_IO_PORT_COUNT];
    uint8_t flash_configuration[16];
    KinetisIoEventHandler event_handler;
    void* event_context;
} KinetisIoConfiguration;

typedef enum {
    KINETIS_SYSMPU_READ,
    KINETIS_SYSMPU_WRITE,
    KINETIS_SYSMPU_EXECUTE,
} KinetisSysMpuAccess;

typedef struct {
    KinetisIoConfiguration configuration;
    bool clock_enabled[KINETIS_PERIPHERAL_COUNT];
    uint32_t port_pcr[KINETIS_IO_PORT_COUNT][KINETIS_IO_PIN_COUNT];
    uint32_t port_isfr[KINETIS_IO_PORT_COUNT];
    uint32_t port_dfer[KINETIS_IO_PORT_COUNT];
    uint8_t port_dfcr[KINETIS_IO_PORT_COUNT];
    uint8_t port_dfwr[KINETIS_IO_PORT_COUNT];
    uint32_t gpio_pdor[KINETIS_IO_PORT_COUNT];
    uint32_t gpio_pddr[KINETIS_IO_PORT_COUNT];
    uint32_t gpio_external[KINETIS_IO_PORT_COUNT];
    uint32_t gpio_external_drive[KINETIS_IO_PORT_COUNT];
    uint32_t gpio_filtered[KINETIS_IO_PORT_COUNT];
    uint32_t gpio_pending[KINETIS_IO_PORT_COUNT];
    uint8_t gpio_filter_age[KINETIS_IO_PORT_COUNT][KINETIS_IO_PIN_COUNT];
    uint8_t usb[0x160];
    uint32_t usb_cycle_remainder;
    uint32_t can[0x8c0 / 4];
    uint32_t i2s[0x108 / 4];
    uint32_t flexbus[0x64 / 4];
    uint32_t mcm[0x44 / 4];
    uint32_t sysmpu[0x830 / 4];
    uint32_t i2s_receive_fifo[KINETIS_IO_FIFO_CAPACITY];
    uint32_t i2s_transmit_fifo[KINETIS_IO_FIFO_CAPACITY];
    uint8_t i2s_receive_read;
    uint8_t i2s_receive_write;
    uint8_t i2s_receive_count;
    uint8_t i2s_transmit_read;
    uint8_t i2s_transmit_write;
    uint8_t i2s_transmit_count;
} KinetisIo;

KinetisIoConfiguration kinetis_io_default_configuration(const KinetisDeviceProfile* profile);
bool kinetis_io_init(KinetisIo* io, KinetisIoConfiguration configuration);
void kinetis_io_reset(KinetisIo* io);
bool kinetis_io_copy(KinetisIo* destination, const KinetisIo* source);
bool kinetis_io_read(KinetisIo* io, uint32_t address, uint8_t size, uint32_t* value);
bool kinetis_io_write(KinetisIo* io, uint32_t address, uint8_t size, uint32_t value);
void kinetis_io_advance(KinetisIo* io, uint32_t cycles);
void kinetis_io_set_clock(KinetisIo* io, KinetisPeripheralId peripheral, bool enabled);
bool kinetis_io_clock_enabled(const KinetisIo* io, KinetisPeripheralId peripheral);
bool kinetis_io_drive_pin(KinetisIo* io, uint8_t port, uint8_t pin, bool high);
bool kinetis_io_release_pin(KinetisIo* io, uint8_t port, uint8_t pin);
uint32_t kinetis_io_pin_input(const KinetisIo* io, uint8_t port);
bool kinetis_io_usb_token(KinetisIo* io, uint8_t endpoint, uint8_t token, bool is_transmit);
bool kinetis_io_can_receive(KinetisIo* io, const KinetisCanFrame* frame);
bool kinetis_io_i2s_receive(KinetisIo* io, uint32_t sample_value);
bool kinetis_io_i2s_transmit(KinetisIo* io, uint32_t* output_sample);
bool kinetis_io_irq_asserted(const KinetisIo* io, uint8_t irq);
bool kinetis_io_flexbus_transfer(KinetisIo* io, uint32_t address, uint8_t access_size,
                                 bool is_write_access, uint32_t write_value);
bool kinetis_io_sysmpu_access(KinetisIo* io, uint32_t address, uint8_t master, bool supervisor,
                              KinetisSysMpuAccess access);

#endif
