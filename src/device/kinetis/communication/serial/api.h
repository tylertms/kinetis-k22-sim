#ifndef KINETIS_SIM_SERIAL_H
#define KINETIS_SIM_SERIAL_H

#include "device/kinetis/variants/profile.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    KINETIS_SERIAL_FIFO_CAPACITY = 256,
    KINETIS_SERIAL_EVENT_CAPACITY = 64,
};

static const KinetisSerialEndpoint KINETIS_SERIAL_ENDPOINT_INVALID =
    (KinetisSerialEndpoint)KINETIS_SERIAL_ENDPOINT_COUNT;

typedef enum {
    KINETIS_SERIAL_IRQ_LPUART0,
    KINETIS_SERIAL_IRQ_SPI0,
    KINETIS_SERIAL_IRQ_SPI1,
    KINETIS_SERIAL_IRQ_SPI2,
    KINETIS_SERIAL_IRQ_I2C0,
    KINETIS_SERIAL_IRQ_I2C1,
    KINETIS_SERIAL_IRQ_I2C2,
    KINETIS_SERIAL_IRQ_UART0,
    KINETIS_SERIAL_IRQ_UART0_ERROR,
    KINETIS_SERIAL_IRQ_UART1,
    KINETIS_SERIAL_IRQ_UART1_ERROR,
    KINETIS_SERIAL_IRQ_UART2,
    KINETIS_SERIAL_IRQ_UART2_ERROR,
    KINETIS_SERIAL_IRQ_UART3,
    KINETIS_SERIAL_IRQ_UART3_ERROR,
    KINETIS_SERIAL_IRQ_UART4,
    KINETIS_SERIAL_IRQ_UART4_ERROR,
    KINETIS_SERIAL_IRQ_UART5,
    KINETIS_SERIAL_IRQ_UART5_ERROR,
    KINETIS_SERIAL_IRQ_COUNT,
} KinetisSerialIrq;

typedef enum {
    KINETIS_SERIAL_DMA_LPUART0_RECEIVE,
    KINETIS_SERIAL_DMA_LPUART0_TRANSMIT,
    KINETIS_SERIAL_DMA_SPI0_RECEIVE,
    KINETIS_SERIAL_DMA_SPI0_TRANSMIT,
    KINETIS_SERIAL_DMA_SPI1_RECEIVE,
    KINETIS_SERIAL_DMA_SPI1_TRANSMIT,
    KINETIS_SERIAL_DMA_SPI2_RECEIVE,
    KINETIS_SERIAL_DMA_SPI2_TRANSMIT,
    KINETIS_SERIAL_DMA_I2C0,
    KINETIS_SERIAL_DMA_I2C1,
    KINETIS_SERIAL_DMA_I2C2,
    KINETIS_SERIAL_DMA_UART0_RECEIVE,
    KINETIS_SERIAL_DMA_UART0_TRANSMIT,
    KINETIS_SERIAL_DMA_UART1_RECEIVE,
    KINETIS_SERIAL_DMA_UART1_TRANSMIT,
    KINETIS_SERIAL_DMA_UART2_RECEIVE,
    KINETIS_SERIAL_DMA_UART2_TRANSMIT,
    KINETIS_SERIAL_DMA_UART3_RECEIVE,
    KINETIS_SERIAL_DMA_UART3_TRANSMIT,
    KINETIS_SERIAL_DMA_UART4_RECEIVE,
    KINETIS_SERIAL_DMA_UART4_TRANSMIT,
    KINETIS_SERIAL_DMA_UART5_RECEIVE,
    KINETIS_SERIAL_DMA_UART5_TRANSMIT,
    KINETIS_SERIAL_DMA_COUNT,
} KinetisSerialDmaRequest;

typedef enum {
    KINETIS_SERIAL_EVENT_I2C_START,
    KINETIS_SERIAL_EVENT_I2C_REPEATED_START,
    KINETIS_SERIAL_EVENT_I2C_STOP,
    KINETIS_SERIAL_EVENT_I2C_WRITE,
    KINETIS_SERIAL_EVENT_I2C_READ,
} KinetisSerialEventType;

typedef struct {
    KinetisSerialEndpoint endpoint;
    KinetisSerialEventType type;
    uint16_t value;
} KinetisSerialEvent;

typedef struct {
    uint16_t data;
    uint8_t chip_selects;
    uint8_t clock_and_transfer_attributes;
    bool continuous_chip_select;
    bool end_of_queue;
} KinetisSerialSpiTransfer;

typedef struct {
    uint16_t values[KINETIS_SERIAL_FIFO_CAPACITY];
    uint16_t metadata[KINETIS_SERIAL_FIFO_CAPACITY];
    uint16_t read_index;
    uint16_t write_index;
    uint16_t count;
} KinetisSerialFifo;

typedef struct {
    KinetisPeripheralId peripheral;
    uint32_t base;
    uint32_t block_size;
    uint8_t registers[0x40];
    KinetisSerialFifo receive;
    KinetisSerialFifo transmit;
    KinetisSerialFifo wire_receive;
    KinetisSerialFifo wire_transmit;
    uint32_t receive_cycles;
    uint32_t transmit_cycles;
    uint8_t fifo_depth;
    bool present;
    bool clock_enabled;
    bool status_read;
} KinetisSerialUart;

typedef struct {
    KinetisPeripheralId peripheral;
    uint32_t base;
    uint32_t block_size;
    uint32_t registers[0x8c / 4];
    KinetisSerialFifo receive;
    KinetisSerialFifo transmit;
    KinetisSerialFifo wire_receive;
    KinetisSerialFifo wire_transmit;
    uint32_t transfer_cycles;
    uint8_t fifo_depth;
    bool present;
    bool clock_enabled;
} KinetisSerialSpi;

typedef struct {
    KinetisPeripheralId peripheral;
    uint32_t base;
    uint32_t block_size;
    uint8_t registers[0x0c];
    KinetisSerialFifo receive;
    KinetisSerialFifo slave_receive;
    KinetisSerialFifo slave_transmit_fifo;
    uint32_t transfer_cycles;
    uint8_t pending_value;
    bool present;
    bool clock_enabled;
    bool acknowledge;
    bool transfer_pending;
    bool read_pending;
    bool slave_transmit;
} KinetisSerialI2c;

typedef struct {
    const KinetisDeviceProfile* profile;
    uint32_t core_clock_hz;
    uint32_t bus_clock_hz;
    KinetisSerialUart lpuart0;
    KinetisSerialUart uart[6];
    KinetisSerialSpi spi[3];
    KinetisSerialI2c i2c[3];
    KinetisSerialEvent events[KINETIS_SERIAL_EVENT_CAPACITY];
    uint8_t event_read_index;
    uint8_t event_write_index;
    uint8_t event_count;
} KinetisSerial;

bool kinetis_serial_init(KinetisSerial* serial, const KinetisDeviceProfile* profile);
void kinetis_serial_reset(KinetisSerial* serial);
bool kinetis_serial_copy(KinetisSerial* destination, const KinetisSerial* source);
void kinetis_serial_set_clocks(KinetisSerial* serial, uint32_t core_clock_hz,
                               uint32_t bus_clock_hz);
bool kinetis_serial_set_clock_gate(KinetisSerial* serial, KinetisPeripheralId peripheral,
                                   bool enabled);
bool kinetis_serial_read(KinetisSerial* serial, uint32_t address, uint8_t byte_count,
                         uint32_t* output_value);
bool kinetis_serial_write(KinetisSerial* serial, uint32_t address, uint8_t byte_count,
                          uint32_t write_value);
void kinetis_serial_advance(KinetisSerial* serial, uint32_t bus_cycles);
void kinetis_serial_advance_endpoint(KinetisSerial* serial, KinetisSerialEndpoint endpoint);
bool kinetis_serial_push_receive(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                                 uint16_t value, uint8_t errors);
bool kinetis_serial_pop_transmit(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                                 uint16_t* value);
bool kinetis_serial_pop_spi_transfer(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                                     KinetisSerialSpiTransfer* transfer);
bool kinetis_serial_i2c_set_acknowledge(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                                        bool acknowledge);
bool kinetis_serial_i2c_detect_start(KinetisSerial* serial, KinetisSerialEndpoint endpoint);
bool kinetis_serial_i2c_detect_stop(KinetisSerial* serial, KinetisSerialEndpoint endpoint);
bool kinetis_serial_i2c_lose_arbitration(KinetisSerial* serial, KinetisSerialEndpoint endpoint);
bool kinetis_serial_i2c_slave_address(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                                      uint16_t address, bool read);
bool kinetis_serial_pop_event(KinetisSerial* serial, KinetisSerialEvent* event);
bool kinetis_serial_irq(const KinetisSerial* serial, KinetisSerialIrq irq);
bool kinetis_serial_dma_request(const KinetisSerial* serial, KinetisSerialDmaRequest request);

#endif
