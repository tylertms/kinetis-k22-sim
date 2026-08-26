#include "device/kinetis/communication/serial/api.h"
#include "device/kinetis/communication/serial/internal.h"

#include <string.h>

#include "device/kinetis/variants/manifest.h"

uint32_t kinetis_serial_internal_load32(const uint8_t* bytes) {
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 |
           (uint32_t)bytes[3] << 24;
}

void kinetis_serial_internal_store32(uint8_t* bytes, uint32_t value) {
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
    bytes[2] = (uint8_t)(value >> 16);
    bytes[3] = (uint8_t)(value >> 24);
}

static void reset_register_block(const KinetisDeviceProfile* profile, uint32_t base_address,
                                 uint32_t block_size, uint8_t* registers, size_t capacity) {
    memset(registers, 0, capacity);
    const KinetisRegisterManifest* manifest = kinetis_register_manifest_get(profile->id);
    if (manifest == NULL) {
        return;
    }
    for (size_t index = 0u; index < manifest->register_count; index++) {
        const KinetisRegisterDescriptor* descriptor = &manifest->registers[index];
        const uint8_t size = (uint8_t)(descriptor->width / 8u);
        if (descriptor->address < base_address ||
            descriptor->address - base_address >= block_size ||
            descriptor->address - base_address + size > capacity) {
            continue;
        }
        const uint32_t offset = descriptor->address - base_address;
        const uint32_t reset_value = descriptor->reset_value & descriptor->reset_mask;
        for (uint8_t byte_index = 0u; byte_index < size; byte_index++) {
            registers[offset + byte_index] = (uint8_t)(reset_value >> (byte_index * 8u));
        }
    }
}

static void fifo_clear(KinetisSerialFifo* fifo) { memset(fifo, 0, sizeof(*fifo)); }

bool kinetis_serial_internal_fifo_push(KinetisSerialFifo* fifo, uint16_t capacity, uint16_t value,
                                       uint16_t metadata) {
    if (fifo->count >= capacity || capacity > KINETIS_SERIAL_FIFO_CAPACITY)
        return false;
    fifo->values[fifo->write_index] = value;
    fifo->metadata[fifo->write_index] = metadata;
    fifo->write_index = (uint16_t)((fifo->write_index + 1u) % KINETIS_SERIAL_FIFO_CAPACITY);
    fifo->count++;
    return true;
}

bool kinetis_serial_internal_fifo_pop(KinetisSerialFifo* fifo, uint16_t* value,
                                      uint16_t* metadata) {
    if (fifo->count == 0)
        return false;
    if (value != NULL)
        *value = fifo->values[fifo->read_index];
    if (metadata != NULL)
        *metadata = fifo->metadata[fifo->read_index];
    fifo->read_index = (uint16_t)((fifo->read_index + 1u) % KINETIS_SERIAL_FIFO_CAPACITY);
    fifo->count--;
    return true;
}

static uint8_t fifo_error(const KinetisSerialFifo* fifo) {
    return fifo->count == 0 ? 0 : (uint8_t)fifo->metadata[fifo->read_index];
}

static void push_event(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                       KinetisSerialEventType type, uint16_t value) {
    if (serial->event_count == KINETIS_SERIAL_EVENT_CAPACITY) {
        serial->event_read_index =
            (uint8_t)((serial->event_read_index + 1u) % KINETIS_SERIAL_EVENT_CAPACITY);
        serial->event_count--;
    }
    KinetisSerialEvent* event = &serial->events[serial->event_write_index];
    event->endpoint = endpoint;
    event->type = type;
    event->value = value;
    serial->event_write_index =
        (uint8_t)((serial->event_write_index + 1u) % KINETIS_SERIAL_EVENT_CAPACITY);
    serial->event_count++;
}

static bool configure_block(const KinetisDeviceProfile* profile, KinetisPeripheralId peripheral,
                            uint32_t* base, uint32_t* size) {
    KinetisPeripheralBlock block;
    if (!kinetis_profile_peripheral_block(profile, peripheral, &block))
        return false;
    *base = block.address;
    *size = block.size;
    return true;
}

static void configure_uart(KinetisSerialUart* uart, const KinetisDeviceProfile* profile,
                           KinetisPeripheralId peripheral, uint8_t fifo_depth) {
    memset(uart, 0, sizeof(*uart));
    uart->peripheral = peripheral;
    uart->present = configure_block(profile, peripheral, &uart->base, &uart->block_size);
    uart->fifo_depth = fifo_depth;
}

static void configure_spi(KinetisSerialSpi* spi, const KinetisDeviceProfile* profile,
                          KinetisPeripheralId peripheral, uint8_t fifo_depth) {
    memset(spi, 0, sizeof(*spi));
    spi->peripheral = peripheral;
    spi->present = configure_block(profile, peripheral, &spi->base, &spi->block_size);
    spi->fifo_depth = fifo_depth;
}

static void configure_i2c(KinetisSerialI2c* i2c, const KinetisDeviceProfile* profile,
                          KinetisPeripheralId peripheral) {
    memset(i2c, 0, sizeof(*i2c));
    i2c->peripheral = peripheral;
    i2c->present = configure_block(profile, peripheral, &i2c->base, &i2c->block_size);
    i2c->acknowledge = true;
}

static void reset_uart(KinetisSerialUart* uart, const KinetisDeviceProfile* profile) {
    reset_register_block(profile, uart->base, uart->block_size, uart->registers,
                         sizeof(uart->registers));
    fifo_clear(&uart->receive);
    fifo_clear(&uart->transmit);
    fifo_clear(&uart->wire_receive);
    fifo_clear(&uart->wire_transmit);
    uart->receive_cycles = 0;
    uart->transmit_cycles = 0;
    uart->clock_enabled = false;
    uart->status_read = false;
}

static void reset_spi(KinetisSerialSpi* spi, const KinetisDeviceProfile* profile) {
    reset_register_block(profile, spi->base, spi->block_size, (uint8_t*)spi->registers,
                         sizeof(spi->registers));
    fifo_clear(&spi->receive);
    fifo_clear(&spi->transmit);
    fifo_clear(&spi->wire_receive);
    fifo_clear(&spi->wire_transmit);
    spi->transfer_cycles = 0;
    spi->push_command = 0;
    spi->clock_enabled = false;
}

static void reset_i2c(KinetisSerialI2c* i2c, const KinetisDeviceProfile* profile) {
    reset_register_block(profile, i2c->base, i2c->block_size, i2c->registers,
                         sizeof(i2c->registers));
    fifo_clear(&i2c->receive);
    fifo_clear(&i2c->slave_receive);
    fifo_clear(&i2c->slave_transmit_fifo);
    i2c->transfer_cycles = 0;
    i2c->pending_value = 0;
    i2c->clock_enabled = false;
    i2c->acknowledge = true;
    i2c->transfer_pending = false;
    i2c->read_pending = false;
    i2c->slave_transmit = false;
}

bool kinetis_serial_init(KinetisSerial* serial, const KinetisDeviceProfile* profile) {
    if (serial == NULL || profile == NULL)
        return false;
    memset(serial, 0, sizeof(*serial));
    serial->profile = profile;
    serial->core_clock_hz = profile->cpu.maximum_core_clock_hz;
    serial->bus_clock_hz = profile->cpu.maximum_core_clock_hz / 2u;
    configure_uart(&serial->lpuart0, profile, KINETIS_PERIPHERAL_LPUART0, 1);
    configure_uart(&serial->uart[0], profile, KINETIS_PERIPHERAL_UART0, 8);
    configure_uart(&serial->uart[1], profile, KINETIS_PERIPHERAL_UART1, 1);
    configure_uart(&serial->uart[2], profile, KINETIS_PERIPHERAL_UART2, 1);
    configure_uart(&serial->uart[3], profile, KINETIS_PERIPHERAL_UART3, 1);
    configure_uart(&serial->uart[4], profile, KINETIS_PERIPHERAL_UART4, 1);
    configure_uart(&serial->uart[5], profile, KINETIS_PERIPHERAL_UART5, 1);
    configure_spi(&serial->spi[0], profile, KINETIS_PERIPHERAL_SPI0, 4);
    configure_spi(&serial->spi[1], profile, KINETIS_PERIPHERAL_SPI1, 4);
    configure_spi(&serial->spi[2], profile, KINETIS_PERIPHERAL_SPI2, 4);
    configure_i2c(&serial->i2c[0], profile, KINETIS_PERIPHERAL_I2C0);
    configure_i2c(&serial->i2c[1], profile, KINETIS_PERIPHERAL_I2C1);
    configure_i2c(&serial->i2c[2], profile, KINETIS_PERIPHERAL_I2C2);
    kinetis_serial_reset(serial);
    return true;
}

void kinetis_serial_reset(KinetisSerial* serial) {
    if (serial == NULL)
        return;
    reset_uart(&serial->lpuart0, serial->profile);
    for (size_t index = 0; index < 6; index++)
        reset_uart(&serial->uart[index], serial->profile);
    for (size_t index = 0; index < 3; index++)
        reset_spi(&serial->spi[index], serial->profile);
    for (size_t index = 0; index < 3; index++)
        reset_i2c(&serial->i2c[index], serial->profile);
    serial->event_read_index = 0;
    serial->event_write_index = 0;
    serial->event_count = 0;
}

bool kinetis_serial_copy(KinetisSerial* destination, const KinetisSerial* source) {
    if (destination == NULL || source == NULL)
        return false;
    *destination = *source;
    return true;
}

void kinetis_serial_set_clocks(KinetisSerial* serial, uint32_t core_clock_hz,
                               uint32_t bus_clock_hz) {
    if (serial == NULL)
        return;
    serial->core_clock_hz = core_clock_hz;
    serial->bus_clock_hz = bus_clock_hz;
}

static KinetisSerialUart* find_uart_by_peripheral(KinetisSerial* serial,
                                                  KinetisPeripheralId peripheral, bool* is_lpuart) {
    if (peripheral == KINETIS_PERIPHERAL_LPUART0) {
        *is_lpuart = true;
        return &serial->lpuart0;
    }
    *is_lpuart = false;
    if (peripheral >= KINETIS_PERIPHERAL_UART0 && peripheral <= KINETIS_PERIPHERAL_UART5)
        return &serial->uart[peripheral - KINETIS_PERIPHERAL_UART0];
    return NULL;
}

static KinetisSerialSpi* find_spi_by_peripheral(KinetisSerial* serial,
                                                KinetisPeripheralId peripheral) {
    if (peripheral >= KINETIS_PERIPHERAL_SPI0 && peripheral <= KINETIS_PERIPHERAL_SPI2)
        return &serial->spi[peripheral - KINETIS_PERIPHERAL_SPI0];
    return NULL;
}

static KinetisSerialI2c* find_i2c_by_peripheral(KinetisSerial* serial,
                                                KinetisPeripheralId peripheral) {
    if (peripheral >= KINETIS_PERIPHERAL_I2C0 && peripheral <= KINETIS_PERIPHERAL_I2C2)
        return &serial->i2c[peripheral - KINETIS_PERIPHERAL_I2C0];
    return NULL;
}

bool kinetis_serial_set_clock_gate(KinetisSerial* serial, KinetisPeripheralId peripheral,
                                   bool enabled) {
    if (serial == NULL)
        return false;
    bool is_lpuart;
    KinetisSerialUart* uart = find_uart_by_peripheral(serial, peripheral, &is_lpuart);
    if (uart != NULL)
        return uart->present ? (uart->clock_enabled = enabled, true) : false;
    KinetisSerialSpi* spi = find_spi_by_peripheral(serial, peripheral);
    if (spi != NULL)
        return spi->present ? (spi->clock_enabled = enabled, true) : false;
    KinetisSerialI2c* i2c = find_i2c_by_peripheral(serial, peripheral);
    if (i2c != NULL)
        return i2c->present ? (i2c->clock_enabled = enabled, true) : false;
    return false;
}

KinetisSerialUart* kinetis_serial_internal_uart_at(KinetisSerial* serial, uint32_t address,
                                                   bool* lpuart, uint32_t* offset) {
    KinetisSerialUart* candidates[] = {
        &serial->lpuart0, &serial->uart[0], &serial->uart[1], &serial->uart[2],
        &serial->uart[3], &serial->uart[4], &serial->uart[5],
    };
    for (size_t index = 0; index < 7; index++) {
        KinetisSerialUart* uart = candidates[index];
        if (uart->present && address >= uart->base && address - uart->base < uart->block_size) {
            *lpuart = index == 0;
            *offset = address - uart->base;
            return uart;
        }
    }
    return NULL;
}

KinetisSerialSpi* kinetis_serial_internal_spi_at(KinetisSerial* serial, uint32_t address,
                                                 uint32_t* offset) {
    for (size_t index = 0; index < 3; index++) {
        KinetisSerialSpi* spi = &serial->spi[index];
        if (spi->present && address >= spi->base && address - spi->base < spi->block_size) {
            *offset = address - spi->base;
            return spi;
        }
    }
    return NULL;
}

KinetisSerialI2c* kinetis_serial_internal_i2c_at(KinetisSerial* serial, uint32_t address,
                                                 uint32_t* offset) {
    for (size_t index = 0; index < 3; index++) {
        KinetisSerialI2c* i2c = &serial->i2c[index];
        if (i2c->present && address >= i2c->base && address - i2c->base < i2c->block_size) {
            *offset = address - i2c->base;
            return i2c;
        }
    }
    return NULL;
}

uint8_t kinetis_serial_internal_uart_capacity(const KinetisSerialUart* uart) {
    const bool receive_fifo_enabled = (uart->registers[UART_PFIFO] & 0x08u) != 0;
    return receive_fifo_enabled ? uart->fifo_depth : 1;
}

static uint8_t uart_transmit_capacity(const KinetisSerialUart* uart) {
    const bool transmit_fifo_enabled = (uart->registers[UART_PFIFO] & 0x80u) != 0;
    return transmit_fifo_enabled ? uart->fifo_depth : 1;
}

void kinetis_serial_internal_refresh_uart(KinetisSerialUart* uart, bool lpuart) {
    if (lpuart) {
        uint32_t status = kinetis_serial_internal_load32(&uart->registers[LPUART_STAT]);
        status &= ~0x00e00000u;
        if (uart->receive.count != 0)
            status |= 1u << 21;
        if (uart->transmit.count < 1)
            status |= 1u << 23;
        if (uart->transmit.count == 0 && uart->transmit_cycles == 0)
            status |= 1u << 22;
        status |= (uint32_t)(fifo_error(&uart->receive) & 0x0fu) << 16;
        kinetis_serial_internal_store32(&uart->registers[LPUART_STAT], status);
        return;
    }
    uint8_t status = uart->registers[UART_S1] & 0x1fu;
    if (uart->receive.count != 0)
        status |= 0x20u;
    if (uart->transmit.count < uart_transmit_capacity(uart))
        status |= 0x80u;
    if (uart->transmit.count == 0 && uart->transmit_cycles == 0)
        status |= 0x40u;
    status |= fifo_error(&uart->receive) & 0x0fu;
    uart->registers[UART_S1] = status;
    uart->registers[UART_RCFIFO] = (uint8_t)uart->receive.count;
    uart->registers[UART_TCFIFO] = (uint8_t)uart->transmit.count;
}

void kinetis_serial_internal_refresh_spi(KinetisSerialSpi* spi) {
    uint32_t status = spi->registers[SPI_SR / 4];
    status &= ~0x0000f0f0u;
    status |= (uint32_t)(spi->transmit.count & 0x0fu) << 12;
    status |= (uint32_t)(spi->receive.count & 0x0fu) << 4;
    if (spi->transmit.count < spi->fifo_depth)
        status |= 1u << 25;
    else
        status &= ~(1u << 25);
    if (spi->receive.count != 0)
        status |= 1u << 17;
    else
        status &= ~(1u << 17);
    if (spi->transfer_cycles != 0)
        status |= 1u << 30;
    else
        status &= ~(1u << 30);
    spi->registers[SPI_SR / 4] = status;
}

bool kinetis_serial_internal_read_uart(KinetisSerialUart* uart, bool lpuart,
                                       uint32_t register_offset, uint8_t byte_count,
                                       uint32_t* output_value) {
    if (!uart->clock_enabled ||
        (lpuart ? byte_count != 4 || (register_offset & 3u) != 0 : byte_count != 1))
        return false;
    kinetis_serial_internal_refresh_uart(uart, lpuart);
    if (lpuart) {
        if (register_offset == LPUART_DATA) {
            uint16_t received = 0;
            uint16_t errors = 0;
            if (kinetis_serial_internal_fifo_pop(&uart->receive, &received, &errors))
                *output_value = received | (uint32_t)errors << 16;
            else
                *output_value = kinetis_serial_internal_load32(&uart->registers[register_offset]);
            kinetis_serial_internal_refresh_uart(uart, true);
            return true;
        }
        *output_value = kinetis_serial_internal_load32(&uart->registers[register_offset]);
        return true;
    }
    if (register_offset == UART_S1)
        uart->status_read = true;
    if (register_offset == UART_D) {
        uint16_t received = 0;
        uint16_t errors = 0;
        if (kinetis_serial_internal_fifo_pop(&uart->receive, &received, &errors)) {
            *output_value = received & 0xffu;
            uart->registers[UART_ED] = (uint8_t)errors;
        } else {
            *output_value = uart->registers[UART_D];
        }
        if (uart->status_read)
            uart->registers[UART_S1] &= 0xf0u;
        uart->status_read = false;
        kinetis_serial_internal_refresh_uart(uart, false);
        return true;
    }
    *output_value = uart->registers[register_offset];
    return true;
}

static bool write_lpuart(KinetisSerialUart* uart, uint32_t register_offset, uint32_t write_value) {
    if (register_offset == LPUART_STAT) {
        uint32_t current = kinetis_serial_internal_load32(&uart->registers[register_offset]);
        current &= ~(write_value & 0xc01f0000u);
        kinetis_serial_internal_store32(&uart->registers[register_offset], current);
        return true;
    }
    if (register_offset == LPUART_DATA) {
        if (!kinetis_serial_internal_fifo_push(&uart->transmit, 1, (uint16_t)(write_value & 0x3ffu),
                                               0)) {
            uint32_t status = kinetis_serial_internal_load32(&uart->registers[LPUART_STAT]);
            kinetis_serial_internal_store32(&uart->registers[LPUART_STAT], status | (1u << 19));
        }
        return true;
    }
    if (register_offset == LPUART_BAUD)
        write_value &= 0xffe73fffu;
    if (register_offset == LPUART_CTRL)
        write_value &= 0xfffdfefdu;
    kinetis_serial_internal_store32(&uart->registers[register_offset], write_value);
    return true;
}

bool kinetis_serial_internal_write_uart(KinetisSerialUart* uart, bool lpuart,
                                        uint32_t register_offset, uint8_t byte_count,
                                        uint32_t write_value) {
    if (!uart->clock_enabled ||
        (lpuart ? byte_count != 4 || (register_offset & 3u) != 0 : byte_count != 1))
        return false;
    if (lpuart) {
        bool result = write_lpuart(uart, register_offset, write_value);
        kinetis_serial_internal_refresh_uart(uart, true);
        return result;
    }
    const uint8_t register_value = (uint8_t)write_value;
    if (register_offset == UART_S1) {
        uart->registers[UART_S1] &= (uint8_t)~(register_value & 0x1fu);
    } else if (register_offset == UART_S2) {
        uart->registers[UART_S2] &= (uint8_t)~(register_value & 0xc0u);
        uart->registers[UART_S2] = (uart->registers[UART_S2] & 0xc0u) | (register_value & 0x3fu);
    } else if (register_offset == UART_D) {
        if (!kinetis_serial_internal_fifo_push(&uart->transmit, uart_transmit_capacity(uart),
                                               register_value, 0))
            uart->registers[UART_SFIFO] |= 0x02u;
    } else if (register_offset == UART_CFIFO) {
        if ((register_value & 0x40u) != 0)
            fifo_clear(&uart->receive);
        if ((register_value & 0x80u) != 0)
            fifo_clear(&uart->transmit);
        uart->registers[UART_CFIFO] = register_value & 0x0cu;
    } else if (register_offset == UART_SFIFO) {
        uart->registers[UART_SFIFO] &= (uint8_t)~(register_value & 0xc3u);
    } else if (register_offset == UART_PFIFO) {
        uart->registers[UART_PFIFO] =
            (uart->registers[UART_PFIFO] & 0x77u) | (register_value & 0x88u);
    } else if (register_offset == UART_ED || register_offset == UART_TCFIFO ||
               register_offset == UART_RCFIFO) {
    } else {
        uart->registers[register_offset] = register_value;
    }
    kinetis_serial_internal_refresh_uart(uart, false);
    return true;
}

bool kinetis_serial_internal_read_spi(KinetisSerialSpi* spi, uint32_t register_offset,
                                      uint8_t byte_count, uint32_t* output_value) {
    if (!spi->clock_enabled)
        return false;
    kinetis_serial_internal_refresh_spi(spi);
    if (register_offset == SPI_POPR && (byte_count == 1u || byte_count == 2u || byte_count == 4u)) {
        uint16_t received = 0;
        if (kinetis_serial_internal_fifo_pop(&spi->receive, &received, NULL))
            *output_value = received & (UINT32_MAX >> ((4u - byte_count) * 8u));
        else {
            spi->registers[SPI_SR / 4] |= 1u << 19;
            *output_value = spi->registers[SPI_POPR / 4] & (UINT32_MAX >> ((4u - byte_count) * 8u));
        }
        kinetis_serial_internal_refresh_spi(spi);
        return true;
    }
    if (byte_count != 4u || (register_offset & 3u) != 0u)
        return false;
    *output_value = spi->registers[register_offset / 4];
    return true;
}

bool kinetis_serial_internal_write_spi(KinetisSerialSpi* spi, uint32_t register_offset,
                                       uint8_t byte_count, uint32_t write_value) {
    if (!spi->clock_enabled)
        return false;
    bool master = (spi->registers[SPI_MCR / 4] & (1u << 31)) != 0u;
    if (master && register_offset == SPI_PUSHR + 2u && byte_count == 2u) {
        spi->push_command = (uint16_t)write_value;
    } else if (master && register_offset >= SPI_PUSHR + 2u && register_offset <= SPI_PUSHR + 3u &&
               byte_count == 1u) {
        uint8_t shift = (uint8_t)((register_offset - SPI_PUSHR - 2u) * 8u);
        spi->push_command = (uint16_t)((spi->push_command & ~(UINT16_C(0xff) << shift)) |
                                       ((uint16_t)(write_value & UINT8_MAX) << shift));
    } else if (register_offset == SPI_PUSHR &&
               (byte_count == 1u || byte_count == 2u || byte_count == 4u)) {
        if (master && byte_count == 4u)
            spi->push_command = (uint16_t)(write_value >> 16);
        uint16_t command = master ? spi->push_command : 0u;
        if (!kinetis_serial_internal_fifo_push(&spi->transmit, spi->fifo_depth,
                                               (uint16_t)write_value, command))
            spi->registers[SPI_SR / 4] |= 1u << 27;
    } else if (byte_count != 4u || (register_offset & 3u) != 0u) {
        return false;
    } else if (register_offset == SPI_MCR) {
        if ((write_value & (1u << 10)) != 0)
            fifo_clear(&spi->receive);
        if ((write_value & (1u << 11)) != 0)
            fifo_clear(&spi->transmit);
        spi->registers[register_offset / 4] = write_value & ~0x00000c00u;
    } else if (register_offset == SPI_SR) {
        spi->registers[register_offset / 4] &= ~(write_value & 0xba0a0000u);
    } else if (register_offset == SPI_POPR) {
    } else {
        spi->registers[register_offset / 4] = write_value;
    }
    kinetis_serial_internal_refresh_spi(spi);
    return true;
}

static KinetisSerialEndpoint i2c_endpoint(const KinetisSerial* serial,
                                          const KinetisSerialI2c* i2c) {
    return (KinetisSerialEndpoint)(KINETIS_SERIAL_I2C0 + (i2c - serial->i2c));
}

bool kinetis_serial_internal_read_i2c(KinetisSerial* serial, KinetisSerialI2c* i2c,
                                      uint32_t register_offset, uint8_t byte_count,
                                      uint32_t* output_value) {
    if (!i2c->clock_enabled || byte_count != 1)
        return false;
    if (register_offset == I2C_D) {
        *output_value = i2c->registers[I2C_D];
        if ((i2c->registers[I2C_C1] & 0x30u) == 0) {
            if (i2c->slave_receive.count != 0)
                kinetis_serial_internal_fifo_pop(&i2c->slave_receive, NULL, NULL);
        } else if ((i2c->registers[I2C_C1] & 0x10u) == 0) {
            i2c->read_pending = true;
            i2c->transfer_pending = true;
            i2c->transfer_cycles = 0;
            push_event(serial, i2c_endpoint(serial, i2c), KINETIS_SERIAL_EVENT_I2C_READ, 0);
        }
        return true;
    }
    *output_value = i2c->registers[register_offset];
    return true;
}

static void i2c_stop(KinetisSerial* serial, KinetisSerialI2c* i2c) {
    i2c->registers[I2C_S] &= 0xdfu;
    i2c->transfer_pending = false;
    i2c->transfer_cycles = 0;
    push_event(serial, i2c_endpoint(serial, i2c), KINETIS_SERIAL_EVENT_I2C_STOP, 0);
}

bool kinetis_serial_internal_write_i2c(KinetisSerial* serial, KinetisSerialI2c* i2c,
                                       uint32_t register_offset, uint8_t byte_count,
                                       uint32_t write_value) {
    if (!i2c->clock_enabled || byte_count != 1)
        return false;
    const uint8_t register_value = (uint8_t)write_value;
    if (register_offset == I2C_C1) {
        const uint8_t previous_control = i2c->registers[I2C_C1];
        i2c->registers[I2C_C1] = register_value & 0xfbu;
        if ((register_value & 0x80u) == 0) {
            if ((previous_control & 0x20u) != 0 || (i2c->registers[I2C_S] & 0x20u) != 0)
                i2c_stop(serial, i2c);
            else {
                i2c->transfer_pending = false;
                i2c->transfer_cycles = 0;
            }
            i2c->registers[I2C_S] = 0;
        } else if ((register_value & 0x20u) != 0 && (previous_control & 0x20u) == 0) {
            i2c->registers[I2C_S] |= 0x20u;
            push_event(serial, i2c_endpoint(serial, i2c), KINETIS_SERIAL_EVENT_I2C_START, 0);
        } else if ((register_value & 0x04u) != 0 && (previous_control & 0x04u) == 0 &&
                   (register_value & 0x20u) != 0) {
            push_event(serial, i2c_endpoint(serial, i2c), KINETIS_SERIAL_EVENT_I2C_REPEATED_START,
                       0);
        } else if ((register_value & 0x20u) == 0 && (previous_control & 0x20u) != 0) {
            i2c_stop(serial, i2c);
        }
    } else if (register_offset == I2C_S) {
        i2c->registers[I2C_S] &= (uint8_t)~(register_value & 0x12u);
    } else if (register_offset == I2C_FLT) {
        i2c->registers[I2C_FLT] =
            (register_value & 0xafu) | (i2c->registers[I2C_FLT] & (uint8_t)~register_value & 0x50u);
    } else if (register_offset == I2C_D) {
        i2c->registers[I2C_D] = register_value;
        if ((i2c->registers[I2C_C1] & 0x30u) == 0x30u) {
            i2c->pending_value = register_value;
            i2c->read_pending = false;
            i2c->transfer_pending = true;
            i2c->transfer_cycles = 0;
            push_event(serial, i2c_endpoint(serial, i2c), KINETIS_SERIAL_EVENT_I2C_WRITE,
                       register_value);
        } else if (i2c->slave_transmit) {
            i2c->pending_value = register_value;
            kinetis_serial_internal_fifo_push(&i2c->slave_transmit_fifo,
                                              KINETIS_SERIAL_FIFO_CAPACITY, register_value, 0);
        }
    } else {
        i2c->registers[register_offset] = register_value;
    }
    return true;
}
