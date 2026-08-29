#ifndef KINETIS_SIM_SERIAL_INTERNAL_H
#define KINETIS_SIM_SERIAL_INTERNAL_H

#include "device/kinetis/communication/serial/api.h"

#include <string.h>

enum {
    UART_C1 = 0x02,
    UART_C2 = 0x03,
    UART_S1 = 0x04,
    UART_S2 = 0x05,
    UART_C3 = 0x06,
    UART_D = 0x07,
    UART_C4 = 0x0a,
    UART_C5 = 0x0b,
    UART_ED = 0x0c,
    UART_PFIFO = 0x10,
    UART_CFIFO = 0x11,
    UART_SFIFO = 0x12,
    UART_TCFIFO = 0x14,
    UART_RCFIFO = 0x16,
    LPUART_BAUD = 0x00,
    LPUART_STAT = 0x04,
    LPUART_CTRL = 0x08,
    LPUART_DATA = 0x0c,
    SPI_MCR = 0x00,
    SPI_TCR = 0x08,
    SPI_CTAR0 = 0x0c,
    SPI_CTAR1 = 0x10,
    SPI_SR = 0x2c,
    SPI_RSER = 0x30,
    SPI_PUSHR = 0x34,
    SPI_POPR = 0x38,
    I2C_A1 = 0x00,
    I2C_F = 0x01,
    I2C_C1 = 0x02,
    I2C_S = 0x03,
    I2C_D = 0x04,
    I2C_C2 = 0x05,
    I2C_FLT = 0x06,
    I2C_RA = 0x07,
    I2C_SMB = 0x08,
    I2C_A2 = 0x09,
};

bool kinetis_serial_internal_fifo_pop(KinetisSerialFifo* fifo, uint16_t* value, uint16_t* metadata);
bool kinetis_serial_internal_fifo_push(KinetisSerialFifo* fifo, uint16_t capacity, uint16_t value,
                                       uint16_t metadata);
bool kinetis_serial_internal_read_i2c(KinetisSerial* serial, KinetisSerialI2c* i2c,
                                      uint32_t register_offset, uint8_t byte_count,
                                      uint32_t* output_value);
bool kinetis_serial_internal_read_spi(KinetisSerial* serial, KinetisSerialSpi* spi,
                                      uint32_t register_offset,
                                      uint8_t byte_count, uint32_t* output_value);
bool kinetis_serial_internal_read_uart(KinetisSerialUart* uart, bool lpuart,
                                       uint32_t register_offset, uint8_t byte_count,
                                       uint32_t* output_value);
bool kinetis_serial_internal_write_i2c(KinetisSerial* serial, KinetisSerialI2c* i2c,
                                       uint32_t register_offset, uint8_t byte_count,
                                       uint32_t write_value);
bool kinetis_serial_internal_write_spi(KinetisSerial* serial, KinetisSerialSpi* spi,
                                       uint32_t register_offset,
                                       uint8_t byte_count, uint32_t write_value);
bool kinetis_serial_internal_write_uart(KinetisSerialUart* uart, bool lpuart,
                                        uint32_t register_offset, uint8_t byte_count,
                                        uint32_t write_value);
KinetisSerialI2c* kinetis_serial_internal_i2c_at(KinetisSerial* serial, uint32_t address,
                                                 uint32_t* offset);
KinetisSerialSpi* kinetis_serial_internal_spi_at(KinetisSerial* serial, uint32_t address,
                                                 uint32_t* offset);
KinetisSerialUart* kinetis_serial_internal_uart_at(KinetisSerial* serial, uint32_t address,
                                                   bool* lpuart, uint32_t* offset);
uint32_t kinetis_serial_internal_load32(const uint8_t* bytes);
uint8_t kinetis_serial_internal_spi_receive_capacity(const KinetisSerialSpi* spi);
bool kinetis_serial_internal_spi_running(const KinetisSerial* serial,
                                         const KinetisSerialSpi* spi);
uint8_t kinetis_serial_internal_spi_transmit_capacity(const KinetisSerialSpi* spi);
uint8_t kinetis_serial_internal_uart_capacity(const KinetisSerialUart* uart);
void kinetis_serial_internal_refresh_spi(const KinetisSerial* serial, KinetisSerialSpi* spi);
void kinetis_serial_internal_refresh_uart(KinetisSerialUart* uart, bool lpuart);
void kinetis_serial_internal_store32(uint8_t* bytes, uint32_t value);

#endif
