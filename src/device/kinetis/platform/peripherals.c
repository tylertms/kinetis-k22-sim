#include "device/kinetis/internal.h"
#include "device/kinetis/timing/internal.h"

#include <string.h>

uint32_t kinetis_internal_width_mask(uint8_t byte_count) {
    return byte_count == 1 ? 0xffu : byte_count == 2 ? 0xffffu : UINT32_MAX;
}

bool kinetis_internal_pop_serial_event(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                                       KinetisSerialEvent* output_event) {
    for (uint8_t event_offset = 0; event_offset < serial->event_count; event_offset++) {
        const uint8_t event_index =
            (uint8_t)((serial->event_read_index + event_offset) % KINETIS_SERIAL_EVENT_CAPACITY);
        if (serial->events[event_index].endpoint != endpoint) {
            continue;
        }
        *output_event = serial->events[event_index];
        for (uint8_t current_offset = event_offset; current_offset + 1u < serial->event_count;
             current_offset++) {
            const uint8_t destination = (uint8_t)((serial->event_read_index + current_offset) %
                                                  KINETIS_SERIAL_EVENT_CAPACITY);
            const uint8_t next_event_index =
                (uint8_t)((serial->event_read_index + current_offset + 1u) %
                          KINETIS_SERIAL_EVENT_CAPACITY);
            serial->events[destination] = serial->events[next_event_index];
        }
        serial->event_write_index =
            (uint8_t)((serial->event_write_index + KINETIS_SERIAL_EVENT_CAPACITY - 1u) %
                      KINETIS_SERIAL_EVENT_CAPACITY);
        serial->event_count--;
        return true;
    }
    return false;
}

uint32_t kinetis_internal_raw_load(const Kinetis* device, uint32_t address, uint8_t byte_count) {
    if (address >= KINETIS_PRIVATE_PERIPHERAL_BASE &&
        address - KINETIS_PRIVATE_PERIPHERAL_BASE <=
            (uint32_t)KINETIS_PRIVATE_PERIPHERAL_SIZE - byte_count) {
        const uint8_t* bytes =
            device->private_peripheral + (address - KINETIS_PRIVATE_PERIPHERAL_BASE);
        uint32_t value = 0u;
        for (uint8_t index = 0u; index < byte_count; index++) {
            value |= (uint32_t)bytes[index] << (index * 8u);
        }
        return value;
    }
    if (address < KINETIS_PERIPHERAL_BASE ||
        address - KINETIS_PERIPHERAL_BASE > (uint32_t)KINETIS_PERIPHERAL_SIZE - byte_count) {
        return 0;
    }
    const uint8_t* peripheral_bytes = device->peripheral + (address - KINETIS_PERIPHERAL_BASE);
    uint32_t register_value = 0;
    for (uint8_t byte_index = 0; byte_index < byte_count; byte_index++) {
        register_value |= (uint32_t)peripheral_bytes[byte_index] << (byte_index * 8u);
    }
    return register_value;
}

void kinetis_internal_raw_store(Kinetis* device, uint32_t address, uint8_t byte_count,
                                uint32_t write_value) {
    if (address >= KINETIS_PRIVATE_PERIPHERAL_BASE &&
        address - KINETIS_PRIVATE_PERIPHERAL_BASE <=
            (uint32_t)KINETIS_PRIVATE_PERIPHERAL_SIZE - byte_count) {
        uint8_t* bytes = device->private_peripheral + (address - KINETIS_PRIVATE_PERIPHERAL_BASE);
        for (uint8_t index = 0u; index < byte_count; index++) {
            bytes[index] = (uint8_t)(write_value >> (index * 8u));
        }
        return;
    }
    if (address < KINETIS_PERIPHERAL_BASE ||
        address - KINETIS_PERIPHERAL_BASE > (uint32_t)KINETIS_PERIPHERAL_SIZE - byte_count) {
        return;
    }
    uint8_t* peripheral_bytes = device->peripheral + (address - KINETIS_PERIPHERAL_BASE);
    for (uint8_t byte_index = 0; byte_index < byte_count; byte_index++) {
        peripheral_bytes[byte_index] = (uint8_t)(write_value >> (byte_index * 8u));
    }
}

bool kinetis_internal_aips_access_allowed(const Kinetis* device, uint32_t address,
                                          CortexM4Access access, bool write_access) {
    if (!kinetis_profile_has_peripheral(device->profile, KINETIS_PERIPHERAL_AIPS0) ||
        access == CORTEX_M4_ACCESS_DEBUG || address < KINETIS_PERIPHERAL_BASE ||
        address >= KINETIS_PERIPHERAL_BASE + KINETIS_PERIPHERAL_SIZE) {
        return true;
    }
    const uint32_t aperture = address < KINETIS_AIPS1 ? KINETIS_AIPS0 : KINETIS_AIPS1;
    const uint8_t port_slot = (uint8_t)((address - aperture) >> 12u);
    const uint32_t access_control =
        kinetis_internal_raw_load(device, aperture + 0x20u + (uint32_t)(port_slot / 8u) * 4u, 4u);
    const uint8_t permission_shift = (uint8_t)((7u - port_slot % 8u) * 4u);
    const uint8_t access_permission = (uint8_t)(access_control >> permission_shift) & 7u;
    if (write_access && (access_permission & 2u) != 0u) {
        return false;
    }
    return !cortex_m4_access_is_unprivileged_data(device->cpu, access) ||
           (access_permission & 4u) == 0u;
}

bool kinetis_internal_axbs_write_allowed(const Kinetis* device, uint32_t address) {
    if (address < KINETIS_AXBS || address >= KINETIS_AXBS + 0x500u) {
        return true;
    }
    const uint32_t peripheral_offset = address - KINETIS_AXBS;
    const uint32_t register_offset = peripheral_offset & 0xffu;
    if (register_offset != 0u && register_offset != 0x10u) {
        return true;
    }
    const uint32_t write_control =
        kinetis_internal_raw_load(device, address - register_offset + 0x10u, 4u);
    return (write_control & 0x80000000u) == 0u;
}

static void cmt_clear_eoc(Kinetis* device);

static bool cmt_read(Kinetis* device, uint32_t address, uint8_t byte_count,
                     uint32_t* output_value) {
    if (address < KINETIS_CMT || address >= KINETIS_CMT + 0x0cu || byte_count != 1u) {
        return false;
    }
    *output_value = kinetis_internal_raw_load(device, address, 1u);
    if (address == KINETIS_CMT + 5u && (*output_value & 0x80u) != 0u)
        device->cmt_eoc_read = true;
    else if ((address == KINETIS_CMT + 7u || address == KINETIS_CMT + 9u) && device->cmt_eoc_read &&
             (kinetis_internal_raw_load(device, KINETIS_CMT + 0x0bu, 1u) & 1u) == 0u)
        cmt_clear_eoc(device);
    return true;
}

static void cmt_refresh_irq(Kinetis* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_internal_raw_load(device, KINETIS_CMT + 5u, 1u);
    const uint8_t dma_control = (uint8_t)kinetis_internal_raw_load(device, KINETIS_CMT + 0x0bu, 1u);
    if (device->cpu != NULL)
        cortex_m4_set_irq_level(device->cpu, 45u,
                                (cmt_control & 0x82u) == 0x82u && (dma_control & 1u) == 0u);
}

static void cmt_refresh_dma(Kinetis* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_internal_raw_load(device, KINETIS_CMT + 5u, 1u);
    const uint8_t dma_control = (uint8_t)kinetis_internal_raw_load(device, KINETIS_CMT + 0x0bu, 1u);
    if ((cmt_control & 0x82u) == 0x82u && (dma_control & 1u) != 0u && !device->cmt_dma_pending)
        device->cmt_dma_pending = kinetis_data_dma_request(device->data, 47u);
}

static void cmt_raise_cycle(Kinetis* device, uint8_t cmt_control) {
    kinetis_internal_raw_store(device, KINETIS_CMT + 5u, 1u, cmt_control | 0x80u);
    cmt_refresh_dma(device);
    cmt_refresh_irq(device);
}

static uint64_t cmt_clock_ticks(const Kinetis* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_internal_raw_load(device, KINETIS_CMT + 5u, 1u);
    const uint64_t clock_divider = kinetis_internal_raw_load(device, KINETIS_CMT + 0x0au, 1u) + 1u;
    return clock_divider << ((cmt_control >> 5u) & 3u);
}

static void cmt_load_cycle(Kinetis* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_internal_raw_load(device, KINETIS_CMT + 5u, 1u);
    const uint64_t clock_ticks = cmt_clock_ticks(device);
    const uint64_t mark_count = (kinetis_internal_raw_load(device, KINETIS_CMT + 6u, 1u) << 8u) |
                                kinetis_internal_raw_load(device, KINETIS_CMT + 7u, 1u);
    const uint64_t space_count = (kinetis_internal_raw_load(device, KINETIS_CMT + 8u, 1u) << 8u) |
                                 kinetis_internal_raw_load(device, KINETIS_CMT + 9u, 1u);
    uint64_t time_unit_ticks = clock_ticks * 8u;
    const uint8_t carrier_offset = device->cmt_fsk_secondary ? 2u : 0u;
    const uint64_t carrier_high_count =
        kinetis_internal_raw_load(device, KINETIS_CMT + carrier_offset, 1u);
    const uint64_t carrier_low_count =
        kinetis_internal_raw_load(device, KINETIS_CMT + carrier_offset + 1u, 1u);
    device->cmt_carrier_high_ticks = carrier_high_count * clock_ticks;
    device->cmt_carrier_period_ticks = (carrier_high_count + carrier_low_count) * clock_ticks;
    if ((cmt_control & 4u) != 0u && (cmt_control & 8u) == 0u &&
        device->cmt_carrier_period_ticks != 0u)
        time_unit_ticks = device->cmt_carrier_period_ticks;
    device->cmt_extended_space = (cmt_control & 0x10u) != 0u;
    device->cmt_mark_ticks = device->cmt_extended_space ? 0u : (mark_count + 1u) * time_unit_ticks;
    device->cmt_period_ticks = (mark_count + 1u + space_count) * time_unit_ticks;
    device->cmt_carrier_offset_ticks = 0u;
    device->cmt_cycles = 0u;
}

static void cmt_start(Kinetis* device, uint8_t cmt_control) {
    device->cmt_running = true;
    device->cmt_stop_pending = false;
    device->cmt_fsk_secondary = false;
    cmt_load_cycle(device);
    const uint64_t clock_divider = kinetis_internal_raw_load(device, KINETIS_CMT + 0x0au, 1u);
    device->cmt_output_delay_ticks =
        ((cmt_control >> 5u) & 3u) == 0u ? clock_divider + 2u : clock_divider * 2u + 3u;
    device->cmt_carrier_offset_ticks = device->cmt_output_delay_ticks;
    cmt_raise_cycle(device, cmt_control);
}

static void cmt_clear_eoc(Kinetis* device) {
    kinetis_internal_raw_store(device, KINETIS_CMT + 5u, 1u,
                               kinetis_internal_raw_load(device, KINETIS_CMT + 5u, 1u) & 0x7fu);
    device->cmt_eoc_read = false;
    cmt_refresh_irq(device);
}

static bool cmt_write(Kinetis* device, uint32_t address, uint8_t byte_count, uint32_t write_value) {
    if (address < KINETIS_CMT || address >= KINETIS_CMT + 0x0cu || byte_count != 1u)
        return false;
    const uint32_t register_offset = address - KINETIS_CMT;
    if (register_offset == 5u) {
        const uint8_t previous_control = (uint8_t)kinetis_internal_raw_load(device, address, 1u);
        const uint8_t cmt_control = (uint8_t)(write_value & 0x7fu);
        kinetis_internal_raw_store(device, address, 1u, (previous_control & 0x80u) | cmt_control);
        if ((cmt_control & 1u) == 0u && device->cmt_running)
            device->cmt_stop_pending = true;
        else if ((cmt_control & 1u) != 0u && !device->cmt_running)
            cmt_start(device, cmt_control);
        else if ((cmt_control & 1u) != 0u)
            device->cmt_stop_pending = false;
        cmt_refresh_dma(device);
        cmt_refresh_irq(device);
        return true;
    }
    const KinetisRegisterDescriptor* descriptor =
        kinetis_register_manifest_lookup(device->profile->id, address, 8u);
    if (descriptor == NULL || (descriptor->access & KINETIS_REGISTER_ACCESS_WRITE) == 0u)
        return false;
    kinetis_internal_raw_store(device, address, 1u, write_value & descriptor->write_mask);
    if ((register_offset == 7u || register_offset == 9u) && device->cmt_eoc_read &&
        (kinetis_internal_raw_load(device, KINETIS_CMT + 0x0bu, 1u) & 1u) == 0u)
        cmt_clear_eoc(device);
    if (register_offset == 0x0bu) {
        cmt_refresh_dma(device);
        cmt_refresh_irq(device);
    }
    return true;
}

void kinetis_internal_cmt_advance(Kinetis* device, uint32_t cycle_count) {
    if (!device->cmt_running || !kinetis_timing_bus_clock_running(&device->timing))
        return;
    const uint64_t core_clock_hz = kinetis_timing_core_clock_hz(&device->timing);
    const uint64_t bus_clock_hz = kinetis_timing_bus_clock_hz(&device->timing);
    const uint64_t scaled_ticks = device->cmt_bus_remainder + (uint64_t)cycle_count * bus_clock_hz;
    uint64_t elapsed_ticks = core_clock_hz == 0u ? 0u : scaled_ticks / core_clock_hz;
    device->cmt_bus_remainder = core_clock_hz == 0u ? 0u : scaled_ticks % core_clock_hz;
    if (device->cmt_output_delay_ticks > elapsed_ticks)
        device->cmt_output_delay_ticks -= elapsed_ticks;
    else
        device->cmt_output_delay_ticks = 0u;
    while (elapsed_ticks != 0u && device->cmt_running) {
        const uint64_t remaining_ticks = device->cmt_period_ticks - device->cmt_cycles;
        if (elapsed_ticks < remaining_ticks) {
            device->cmt_cycles += elapsed_ticks;
            break;
        }
        elapsed_ticks -= remaining_ticks;
        if (device->cmt_stop_pending ||
            (kinetis_internal_raw_load(device, KINETIS_CMT + 5u, 1u) & 1u) == 0u) {
            device->cmt_running = false;
            device->cmt_stop_pending = false;
            device->cmt_cycles = 0u;
            break;
        }
        const uint8_t cmt_control =
            (uint8_t)kinetis_internal_raw_load(device, KINETIS_CMT + 5u, 1u);
        if ((cmt_control & 0x0cu) == 4u)
            device->cmt_fsk_secondary = !device->cmt_fsk_secondary;
        cmt_load_cycle(device);
        cmt_raise_cycle(device, cmt_control);
    }
}

static bool serial_peripheral(KinetisPeripheralId id) {
    return id == KINETIS_PERIPHERAL_LPUART0 || id == KINETIS_PERIPHERAL_SPI0 ||
           id == KINETIS_PERIPHERAL_SPI1 || id == KINETIS_PERIPHERAL_SPI2 ||
           id == KINETIS_PERIPHERAL_I2C0 || id == KINETIS_PERIPHERAL_I2C1 ||
           id == KINETIS_PERIPHERAL_I2C2 || id == KINETIS_PERIPHERAL_UART0 ||
           id == KINETIS_PERIPHERAL_UART1 || id == KINETIS_PERIPHERAL_UART2 ||
           id == KINETIS_PERIPHERAL_UART3 || id == KINETIS_PERIPHERAL_UART4 ||
           id == KINETIS_PERIPHERAL_UART5;
}

static bool package_serial_extension(KinetisPeripheralId id) {
    return id == KINETIS_PERIPHERAL_SPI1 || id == KINETIS_PERIPHERAL_SPI2 ||
           id == KINETIS_PERIPHERAL_UART3 || id == KINETIS_PERIPHERAL_UART4 ||
           id == KINETIS_PERIPHERAL_UART5;
}

bool kinetis_internal_manifest_extension(KinetisPeripheralId id) {
    return package_serial_extension(id) || id == KINETIS_PERIPHERAL_SDHC ||
           id == KINETIS_PERIPHERAL_DAC1;
}

KinetisPeripheralId kinetis_internal_serial_endpoint_peripheral(KinetisSerialEndpoint endpoint) {
    static const KinetisPeripheralId peripherals[KINETIS_SERIAL_ENDPOINT_COUNT] = {
        KINETIS_PERIPHERAL_LPUART0, KINETIS_PERIPHERAL_SPI0,  KINETIS_PERIPHERAL_SPI1,
        KINETIS_PERIPHERAL_SPI2,    KINETIS_PERIPHERAL_I2C0,  KINETIS_PERIPHERAL_I2C1,
        KINETIS_PERIPHERAL_I2C2,    KINETIS_PERIPHERAL_UART0, KINETIS_PERIPHERAL_UART1,
        KINETIS_PERIPHERAL_UART2,   KINETIS_PERIPHERAL_UART3, KINETIS_PERIPHERAL_UART4,
        KINETIS_PERIPHERAL_UART5,
    };
    return endpoint < KINETIS_SERIAL_ENDPOINT_COUNT ? peripherals[endpoint]
                                                    : KINETIS_PERIPHERAL_COUNT;
}

bool kinetis_internal_serial_endpoint_available(const Kinetis* device,
                                                KinetisSerialEndpoint endpoint) {
    const KinetisPeripheralId peripheral = kinetis_internal_serial_endpoint_peripheral(endpoint);
    return device != NULL && peripheral < KINETIS_PERIPHERAL_COUNT &&
           kinetis_package_has_peripheral(device->package, peripheral);
}

static bool data_peripheral(KinetisPeripheralId id) {
    return id == KINETIS_PERIPHERAL_FLASH_CONFIG || id == KINETIS_PERIPHERAL_DMA ||
           id == KINETIS_PERIPHERAL_FTFA || id == KINETIS_PERIPHERAL_FTFE ||
           id == KINETIS_PERIPHERAL_DMAMUX || id == KINETIS_PERIPHERAL_ADC0 ||
           id == KINETIS_PERIPHERAL_ADC1 || id == KINETIS_PERIPHERAL_DAC0 ||
           id == KINETIS_PERIPHERAL_DAC1 || id == KINETIS_PERIPHERAL_RNG ||
           id == KINETIS_PERIPHERAL_CRC || id == KINETIS_PERIPHERAL_CMP0 ||
           id == KINETIS_PERIPHERAL_CMP1 || id == KINETIS_PERIPHERAL_CMP2 ||
           id == KINETIS_PERIPHERAL_VREF;
}

static uint32_t integer_square_root(uint32_t value) {
    uint32_t root = 0u;
    uint32_t bit = UINT32_C(1) << 30u;
    while (bit > value) {
        bit >>= 2u;
    }
    while (bit != 0u) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1u) + bit;
        } else {
            root >>= 1u;
        }
        bit >>= 2u;
    }
    return root;
}

static uint8_t mmdvsq_divide_cycles(uint32_t magnitude) {
    if (magnitude == 0u)
        return 1u;
    uint8_t most_significant_bit = 0u;
    while ((magnitude >>= 1u) != 0u)
        most_significant_bit++;
    return (uint8_t)(most_significant_bit / 2u + 2u);
}

static void mmdvsq_complete(Kinetis* device) {
    if (device->mmdvsq_cycles_remaining == 0u)
        return;
    device->mmdvsq_cycles_remaining = 0u;
    kinetis_internal_raw_store(device, 0xf0004008u, 4u, device->mmdvsq_pending_control);
    kinetis_internal_raw_store(device, 0xf000400cu, 4u, device->mmdvsq_pending_result);
}

void kinetis_internal_mmdvsq_advance(Kinetis* device, uint32_t cycle_count) {
    if (device->mmdvsq_cycles_remaining == 0u)
        return;
    if (cycle_count >= device->mmdvsq_cycles_remaining) {
        mmdvsq_complete(device);
    } else {
        device->mmdvsq_cycles_remaining -= (uint8_t)cycle_count;
    }
}

uint32_t kinetis_internal_wait_states(void* context, uint32_t address, uint8_t byte_count,
                                      CortexM4Access access, bool write, bool sequential) {
    Kinetis* device = context;
    (void)sequential;
    if (device != NULL && device->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
        address >= 0x44000000u && address < 0x60000000u &&
        (access == CORTEX_M4_ACCESS_DATA || access == CORTEX_M4_ACCESS_UNPRIVILEGED_DATA))
        return 1u;
    if (device == NULL || device->profile->id != KINETIS_PROFILE_MKV10Z1287 || byte_count != 4u ||
        access == CORTEX_M4_ACCESS_INSTRUCTION || device->mmdvsq_cycles_remaining == 0u ||
        address < 0xf0004000u || address > 0xf0004010u ||
        (!write && (address == 0xf0004008u || address == 0xf0004010u)))
        return 0u;
    device->mmdvsq_stall_cycles = device->mmdvsq_cycles_remaining;
    return device->mmdvsq_cycles_remaining;
}

static void mmdvsq_divide(Kinetis* device) {
    const uint32_t dividend = kinetis_internal_raw_load(device, 0xf0004000u, 4u);
    const uint32_t divisor = kinetis_internal_raw_load(device, 0xf0004004u, 4u);
    uint32_t control = kinetis_internal_raw_load(device, 0xf0004008u, 4u) & 0x2eu;
    uint32_t result;
    if (divisor == 0u) {
        control |= 0x10u;
        result = 0u;
    } else if ((control & 2u) != 0u) {
        result = (control & 4u) != 0u ? dividend % divisor : dividend / divisor;
    } else if ((int32_t)dividend == INT32_MIN && (int32_t)divisor == -1) {
        result = (control & 4u) != 0u ? 0u : 0x80000000u;
    } else {
        result = (control & 4u) != 0u ? (uint32_t)((int32_t)dividend % (int32_t)divisor)
                                      : (uint32_t)((int32_t)dividend / (int32_t)divisor);
    }
    const uint32_t magnitude =
        (control & 2u) != 0u || (int32_t)dividend >= 0 ? dividend : 0u - dividend;
    device->mmdvsq_pending_result = result;
    device->mmdvsq_pending_control = control | 0x40000000u;
    device->mmdvsq_cycles_remaining = mmdvsq_divide_cycles(magnitude);
    kinetis_internal_raw_store(device, 0xf0004008u, 4u,
                               device->mmdvsq_pending_control | 0x80000000u);
}

static void mmdvsq_square_root(Kinetis* device, uint32_t radicand) {
    device->mmdvsq_pending_result = integer_square_root(radicand);
    device->mmdvsq_pending_control =
        (kinetis_internal_raw_load(device, 0xf0004008u, 4u) & 0x2eu) | 0x20000000u;
    device->mmdvsq_cycles_remaining =
        radicand == 0u ? 2u : mmdvsq_divide_cycles(radicand);
    kinetis_internal_raw_store(device, 0xf0004008u, 4u,
                               device->mmdvsq_pending_control | 0x80000000u);
}

static bool mmdvsq_read(Kinetis* device, uint32_t address, uint8_t byte_count,
                        uint32_t* output_value) {
    if (byte_count != 4u || address < 0xf0004000u || address > 0xf0004010u) {
        return false;
    }
    if (address == 0xf0004010u)
        return false;
    if (address != 0xf0004008u)
        mmdvsq_complete(device);
    if (address == 0xf000400cu &&
        (kinetis_internal_raw_load(device, 0xf0004008u, 4u) & 0x18u) == 0x18u) {
        return false;
    }
    *output_value = kinetis_internal_raw_load(device, address, 4u);
    return true;
}

static bool mmdvsq_write(Kinetis* device, uint32_t address, uint8_t byte_count,
                         uint32_t write_value) {
    if (byte_count != 4u || address < 0xf0004000u || address > 0xf0004010u) {
        return false;
    }
    mmdvsq_complete(device);
    if (address == 0xf0004000u) {
        kinetis_internal_raw_store(device, address, 4u, write_value);
        return true;
    }
    if (address == 0xf0004004u) {
        kinetis_internal_raw_store(device, address, 4u, write_value);
        if ((kinetis_internal_raw_load(device, 0xf0004008u, 4u) & 0x20u) == 0u) {
            mmdvsq_divide(device);
        }
        return true;
    }
    if (address == 0xf0004008u) {
        const uint32_t previous = kinetis_internal_raw_load(device, address, 4u);
        kinetis_internal_raw_store(device, address, 4u,
                                   (previous & 0x60000010u) | (write_value & 0x3eu));
        if ((write_value & 0x21u) == 0x21u) {
            mmdvsq_divide(device);
        }
        return true;
    }
    if (address == 0xf000400cu) {
        kinetis_internal_raw_store(device, address, 4u, write_value);
        return true;
    }
    if (address == 0xf0004010u) {
        kinetis_internal_raw_store(device, address, 4u, write_value);
        mmdvsq_square_root(device, write_value);
        return true;
    }
    return true;
}

static bool timing_peripheral(KinetisPeripheralId id) {
    return id == KINETIS_PERIPHERAL_FTM0 || id == KINETIS_PERIPHERAL_FTM1 ||
           id == KINETIS_PERIPHERAL_FTM2 || id == KINETIS_PERIPHERAL_FTM3 ||
           id == KINETIS_PERIPHERAL_FTM4 || id == KINETIS_PERIPHERAL_FTM5 ||
           id == KINETIS_PERIPHERAL_PDB0 || id == KINETIS_PERIPHERAL_PDB1 ||
           id == KINETIS_PERIPHERAL_PIT || id == KINETIS_PERIPHERAL_RTC ||
           id == KINETIS_PERIPHERAL_LPTMR0 || id == KINETIS_PERIPHERAL_SIM ||
           id == KINETIS_PERIPHERAL_WDOG || id == KINETIS_PERIPHERAL_EWM ||
           id == KINETIS_PERIPHERAL_MCG || id == KINETIS_PERIPHERAL_OSC ||
           id == KINETIS_PERIPHERAL_LLWU || id == KINETIS_PERIPHERAL_PMC ||
           id == KINETIS_PERIPHERAL_SMC || id == KINETIS_PERIPHERAL_RCM;
}

static bool io_peripheral(KinetisPeripheralId id) {
    return id == KINETIS_PERIPHERAL_FB || id == KINETIS_PERIPHERAL_SYSMPU ||
           id == KINETIS_PERIPHERAL_CAN0 || id == KINETIS_PERIPHERAL_I2S0 ||
           id == KINETIS_PERIPHERAL_USB0 || id == KINETIS_PERIPHERAL_PORTA ||
           id == KINETIS_PERIPHERAL_PORTB || id == KINETIS_PERIPHERAL_PORTC ||
           id == KINETIS_PERIPHERAL_PORTD || id == KINETIS_PERIPHERAL_PORTE ||
           id == KINETIS_PERIPHERAL_GPIOA || id == KINETIS_PERIPHERAL_GPIOB ||
           id == KINETIS_PERIPHERAL_GPIOC || id == KINETIS_PERIPHERAL_GPIOD ||
           id == KINETIS_PERIPHERAL_GPIOE || id == KINETIS_PERIPHERAL_FGPIOA ||
           id == KINETIS_PERIPHERAL_FGPIOB || id == KINETIS_PERIPHERAL_FGPIOC ||
           id == KINETIS_PERIPHERAL_FGPIOD || id == KINETIS_PERIPHERAL_FGPIOE ||
           id == KINETIS_PERIPHERAL_MCM || id == KINETIS_PERIPHERAL_MCG;
}

static uint8_t data_irq(const Kinetis* device, KinetisDataInterrupt interrupt) {
    if (device->profile->id == KINETIS_PROFILE_MKV10Z1287) {
        static const uint8_t irqs[KINETIS_DATA_INTERRUPT_COUNT] = {
            0u,        1u,        2u,        3u,        0u,        1u,        2u,
            3u,        UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX,
            UINT8_MAX, UINT8_MAX, 4u,        5u,        5u,        15u,       16u,
            25u,       UINT8_MAX, 20u,       21u,       UINT8_MAX, UINT8_MAX,
        };
        return irqs[interrupt];
    }
    static const uint8_t irqs[KINETIS_DATA_INTERRUPT_COUNT] = {
        0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13,
        14, 15, 16, 18, 19, 39, 73, 56, 72, 40, 41, 70, 23,
    };
    return irqs[interrupt];
}

static void adc_alternate_trigger(Kinetis* device, uint8_t trigger_source) {
    for (uint8_t adc_instance = 0u; adc_instance < 2u; adc_instance++) {
        const uint8_t selection = (uint8_t)(device->timing.sim_sopt7 >> (adc_instance * 8u));
        if (((selection >> 6u) & 3u) == 2u && (selection & 15u) == trigger_source)
            kinetis_data_adc_pretrigger(device->data, adc_instance,
                                        (uint8_t)((selection >> 4u) & 1u));
    }
}

static uint32_t mkv10_ftm_mux(const Kinetis* device, uint8_t instance) {
    return instance < 3u ? device->timing.sim_sopt4 : device->timing.sim_sopt6;
}

static void mkv10_refresh_ftm_comparator_routes(Kinetis* device) {
    if (device->profile->id != KINETIS_PROFILE_MKV10Z1287)
        return;
    for (uint8_t instance = 0u; instance < KINETIS_FTM_COUNT; instance++) {
        const uint32_t mux = mkv10_ftm_mux(device, instance);
        const uint8_t local = instance % 3u;
        const uint8_t fault_bit = local == 0u ? 0u : (uint8_t)(local + 1u);
        if ((mux & (1u << fault_bit)) != 0u)
            kinetis_timing_internal_set_ftm_routed_fault(&device->timing, instance, 0u,
                                                          device->comparator_output[0]);
        if (instance == 0u && (mux & 2u) != 0u)
            kinetis_timing_internal_set_ftm_routed_fault(&device->timing, 0u, 1u,
                                                          device->comparator_output[1]);
        if (local != 0u) {
            const uint8_t shift = local == 1u ? 18u : 20u;
            const uint8_t source = (uint8_t)((mux >> shift) & 3u);
            kinetis_timing_internal_set_ftm_routed_input(
                &device->timing, instance, 0u,
                source == 1u ? device->comparator_output[0]
                             : source == 2u ? device->comparator_output[1] : false);
        }
    }
}

static void mkv10_trigger_ftm_comparator_routes(Kinetis* device, uint8_t comparator) {
    for (uint8_t instance = 0u; instance < KINETIS_FTM_COUNT; instance++) {
        const uint32_t mux = mkv10_ftm_mux(device, instance);
        const uint8_t local = instance % 3u;
        if (comparator == 0u && (mux & (1u << (7u + local * 3u))) == 0u)
            kinetis_timing_trigger_ftm_hardware(&device->timing, instance, 0u);
        const uint8_t trigger2_comparator = (uint8_t)((mux >> (9u + local * 3u)) & 1u);
        if (comparator == trigger2_comparator)
            kinetis_timing_trigger_ftm_hardware(&device->timing, instance, 2u);
    }
}

static void mkv10_trigger_ftm_output_routes(Kinetis* device, uint8_t source) {
    static const uint8_t trigger0_sources[2][3] = {{1u, 0u, 0u}, {2u, 0u, 0u}};
    static const uint8_t trigger1_sources[2][3] = {{2u, 2u, 1u}, {1u, 2u, 1u}};
    const uint8_t group = source / 3u;
    const uint8_t local_source = source % 3u;
    const uint8_t first = (uint8_t)(group * 3u);
    for (uint8_t local = 0u; local < 3u; local++) {
        const uint32_t mux = mkv10_ftm_mux(device, (uint8_t)(first + local));
        if ((mux & (1u << (7u + local * 3u))) != 0u &&
            trigger0_sources[group][local] == local_source)
            kinetis_timing_trigger_ftm_hardware(&device->timing, (uint8_t)(first + local), 0u);
        if ((mux & (1u << (8u + local * 3u))) != 0u &&
            trigger1_sources[group][local] == local_source)
            kinetis_timing_trigger_ftm_hardware(&device->timing, (uint8_t)(first + local), 1u);
    }
}

void kinetis_internal_data_interrupt(void* context, KinetisDataInterrupt interrupt, bool asserted) {
    Kinetis* device = context;
    if (interrupt < KINETIS_DATA_INTERRUPT_COUNT)
        device->data_interrupt[interrupt] = asserted;
    if (interrupt >= KINETIS_DATA_INTERRUPT_CMP0 && interrupt <= KINETIS_DATA_INTERRUPT_CMP2 &&
        device->data != NULL) {
        const uint8_t comparator_instance = (uint8_t)(interrupt - KINETIS_DATA_INTERRUPT_CMP0);
        bool comparator_high = false;
        if (kinetis_data_get_cmp_output(device->data, comparator_instance, &comparator_high)) {
            if (comparator_instance == 0u)
                kinetis_timing_set_lptmr_input(&device->timing, 0u, comparator_high);
            if (comparator_high && !device->comparator_output[comparator_instance]) {
                adc_alternate_trigger(device, (uint8_t)(1u + comparator_instance));
                if (comparator_instance < 2u)
                    kinetis_timing_trigger_pdb_input(&device->timing,
                                                     (uint8_t)(1u + comparator_instance));
                if (device->profile->id == KINETIS_PROFILE_MKV10Z1287 && comparator_instance < 2u)
                    mkv10_trigger_ftm_comparator_routes(device, comparator_instance);
            }
            device->comparator_output[comparator_instance] = comparator_high;
            mkv10_refresh_ftm_comparator_routes(device);
        }
    }
    if (device->cpu != NULL && interrupt < KINETIS_DATA_INTERRUPT_COUNT) {
        if (device->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
            interrupt >= KINETIS_DATA_INTERRUPT_DMA0 &&
            interrupt <= KINETIS_DATA_INTERRUPT_DMA7) {
            const uint8_t channel = (uint8_t)(interrupt - KINETIS_DATA_INTERRUPT_DMA0);
            const uint8_t shared_channel = channel & 3u;
            cortex_m4_set_irq_level(
                device->cpu, shared_channel,
                device->data_interrupt[KINETIS_DATA_INTERRUPT_DMA0 + shared_channel] ||
                    device->data_interrupt[KINETIS_DATA_INTERRUPT_DMA0 + shared_channel + 4u]);
            return;
        }
        if (device->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
            (interrupt == KINETIS_DATA_INTERRUPT_FTFA ||
             interrupt == KINETIS_DATA_INTERRUPT_FLASH_COLLISION)) {
            cortex_m4_set_irq_level(device->cpu, 5u,
                                    device->data_interrupt[KINETIS_DATA_INTERRUPT_FTFA] ||
                                        device->data_interrupt
                                            [KINETIS_DATA_INTERRUPT_FLASH_COLLISION]);
            return;
        }
        const uint8_t irq = data_irq(device, interrupt);
        if (irq != UINT8_MAX)
            cortex_m4_set_irq_level(device->cpu, irq, asserted);
    }
}

void kinetis_internal_data_dma_complete(void* context, uint8_t channel, uint8_t request_source) {
    Kinetis* device = context;
    kinetis_timing_internal_ftm_dma_complete(&device->timing, request_source);
    if (device->profile->id == KINETIS_PROFILE_MKV10Z1287 && channel < 8u)
        kinetis_timing_internal_trigger_pdb(&device->timing, channel / 4u,
                                            (uint8_t)(4u + channel % 4u));
    if (request_source == 47u && device->cmt_dma_pending) {
        device->cmt_dma_pending = false;
        cmt_clear_eoc(device);
    }
}

bool kinetis_internal_data_bus_read(void* context, uint32_t address, uint8_t byte_count,
                                    uint32_t* output_value) {
    return kinetis_dma_read(context, address, byte_count, output_value);
}

bool kinetis_internal_data_bus_write(void* context, uint32_t address, uint8_t byte_count,
                                     uint32_t write_value) {
    return kinetis_dma_write(context, address, byte_count, write_value);
}

bool kinetis_internal_flash_bus_read(void* context, uint32_t address, uint8_t byte_count,
                                     uint32_t* output_value) {
    return kinetis_flash_controller_read(context, address, byte_count, output_value);
}

bool kinetis_internal_flash_bus_write(void* context, uint32_t address, uint8_t byte_count,
                                      uint32_t write_value) {
    return kinetis_flash_controller_write(context, address, byte_count, write_value);
}

void kinetis_internal_timing_irq(void* context, uint8_t irq, bool asserted) {
    Kinetis* device = context;
    if (device->cpu != NULL) {
        cortex_m4_set_irq_level(device->cpu, irq, asserted);
    }
}

void kinetis_internal_timing_dma(void* context, uint8_t request_source) {
    Kinetis* device = context;
    kinetis_data_dma_request(device->data, request_source);
}

void kinetis_internal_timing_dma_trigger(void* context, uint8_t channel) {
    Kinetis* device = context;
    kinetis_data_dma_trigger(device->data, channel);
}

void kinetis_internal_timing_pdb_pulse(void* context, uint8_t instance, uint8_t output,
                                       bool asserted) {
    Kinetis* device = context;
    if (device->data == NULL || instance >= 2u || output >= 2u)
        return;
    const bool combined = asserted || device->timing.pdb_pulse_output[instance ^ 1u][output];
    kinetis_data_set_cmp_sample(device->data, output, combined);
}

void kinetis_internal_data_adc_complete(void* context, uint8_t instance, uint8_t pretrigger) {
    Kinetis* device = context;
    kinetis_timing_adc_complete(&device->timing, instance, pretrigger);
    if (device->profile->id == KINETIS_PROFILE_MKV10Z1287 && instance == 0u &&
        pretrigger == 0u) {
        kinetis_timing_trigger_pdb_dac_input(&device->timing, 0u, 0u);
        kinetis_timing_trigger_pdb_dac_input(&device->timing, 1u, 0u);
    }
}

void kinetis_internal_timing_reset(void* context, uint8_t reset_cause_0, uint8_t reset_cause_1) {
    kinetis_warm_reset(context, reset_cause_0, reset_cause_1);
}

void kinetis_internal_timing_trigger(void* context, KinetisTimingTrigger type, uint8_t instance,
                                     uint8_t channel, uint8_t source_instance) {
    Kinetis* device = context;
    if (type == KINETIS_TIMING_TRIGGER_PDB_ADC) {
        const uint8_t selection = (uint8_t)(device->timing.sim_sopt7 >> (instance * 8u));
        const uint8_t source = (selection >> 6u) & 3u;
        if (source == 3u || source == source_instance)
            kinetis_data_adc_pretrigger(device->data, instance, channel);
    } else if (type == KINETIS_TIMING_TRIGGER_PDB_DAC) {
        kinetis_data_dac_trigger(device->data,
                                 device->profile->id == KINETIS_PROFILE_MKV10Z1287 ? 0u
                                                                                   : instance);
    } else if (type == KINETIS_TIMING_TRIGGER_PDB_FTM) {
        const uint8_t first_ftm = instance == 0u ? 0u : 3u;
        for (uint8_t ftm = first_ftm; ftm < first_ftm + 3u; ftm++) {
            const uint8_t local = ftm % 3u;
            if (device->profile->id != KINETIS_PROFILE_MKV10Z1287 ||
                (mkv10_ftm_mux(device, ftm) & (1u << (8u + local * 3u))) == 0u)
                kinetis_timing_trigger_ftm_hardware(&device->timing, ftm, channel);
        }
    } else if (type == KINETIS_TIMING_TRIGGER_FTM_OUTPUT) {
        if (device->profile->id == KINETIS_PROFILE_MKV10Z1287)
            mkv10_trigger_ftm_output_routes(device, source_instance);
    } else if (type == KINETIS_TIMING_TRIGGER_CMP) {
        kinetis_data_cmp_trigger(
            device->data, kinetis_timing_lptmr_trigger_delay_cycles(&device->timing));
    } else {
        adc_alternate_trigger(device, instance);
    }
}

static void queue_event(Kinetis* device, const KinetisIoEvent* incoming_event) {
    if (device->event_count == KINETIS_EVENT_CAPACITY) {
        device->event_read_index =
            (uint8_t)((device->event_read_index + 1u) % KINETIS_EVENT_CAPACITY);
        device->event_count--;
    }
    KinetisEvent* event = &device->events[device->event_write_index];
    event->type = (KinetisEventType)incoming_event->type;
    event->source = incoming_event->source;
    event->value = incoming_event->value;
    event->auxiliary = incoming_event->auxiliary;
    memcpy(event->data, incoming_event->data, sizeof(event->data));
    event->length = incoming_event->length;
    event->extended = incoming_event->extended;
    event->remote = incoming_event->remote;
    device->event_write_index =
        (uint8_t)((device->event_write_index + 1u) % KINETIS_EVENT_CAPACITY);
    device->event_count++;
}

void kinetis_internal_io_event(void* context, const KinetisIoEvent* event) {
    Kinetis* device = context;
    queue_event(device, event);
    if (event->type == KINETIS_IO_EVENT_DMA) {
        const uint8_t request_source = event->auxiliary == 0 ? (event->source == 0 ? 13u : 12u)
                                                             : (uint8_t)(49u + event->source / 32u);
        kinetis_data_dma_request(device->data, request_source);
    }
}

static bool gate(uint32_t register_value, uint8_t bit) {
    return (register_value & (1u << bit)) != 0;
}

static bool mkv10_clock_enabled(const Kinetis* device, KinetisPeripheralId id) {
    const uint32_t scgc4 = device->timing.sim_scgc4;
    const uint32_t scgc5 = device->timing.sim_scgc5;
    const uint32_t scgc6 = device->timing.sim_scgc6;
    const uint32_t scgc7 = device->timing.sim_scgc7;
    if (id >= KINETIS_PERIPHERAL_PORTA && id <= KINETIS_PERIPHERAL_PORTE)
        return gate(scgc5, (uint8_t)(9u + id - KINETIS_PERIPHERAL_PORTA));
    switch (id) {
    case KINETIS_PERIPHERAL_EWM:
        return gate(scgc4, 1u);
    case KINETIS_PERIPHERAL_I2C0:
        return gate(scgc4, 6u);
    case KINETIS_PERIPHERAL_UART0:
        return gate(scgc4, 10u);
    case KINETIS_PERIPHERAL_UART1:
        return gate(scgc4, 11u);
    case KINETIS_PERIPHERAL_CMP0:
    case KINETIS_PERIPHERAL_CMP1:
        return gate(scgc4, 19u);
    case KINETIS_PERIPHERAL_LPTMR0:
        return gate(scgc5, 0u);
    case KINETIS_PERIPHERAL_FTFA:
        return gate(scgc6, 0u);
    case KINETIS_PERIPHERAL_DMAMUX:
        return gate(scgc6, 1u);
    case KINETIS_PERIPHERAL_FTM3:
        return gate(scgc6, 6u);
    case KINETIS_PERIPHERAL_FTM4:
        return gate(scgc6, 7u);
    case KINETIS_PERIPHERAL_FTM5:
        return gate(scgc6, 8u);
    case KINETIS_PERIPHERAL_SPI0:
        return gate(scgc6, 12u);
    case KINETIS_PERIPHERAL_PDB1:
        return gate(scgc6, 17u);
    case KINETIS_PERIPHERAL_CRC:
        return gate(scgc6, 18u);
    case KINETIS_PERIPHERAL_PDB0:
        return gate(scgc6, 22u);
    case KINETIS_PERIPHERAL_FTM0:
        return gate(scgc6, 24u);
    case KINETIS_PERIPHERAL_FTM1:
        return gate(scgc6, 25u);
    case KINETIS_PERIPHERAL_FTM2:
        return gate(scgc6, 26u);
    case KINETIS_PERIPHERAL_ADC0:
        return gate(scgc6, 27u);
    case KINETIS_PERIPHERAL_ADC1:
        return gate(scgc6, 28u);
    case KINETIS_PERIPHERAL_DAC0:
        return gate(scgc6, 31u);
    case KINETIS_PERIPHERAL_DMA:
        return gate(scgc7, 8u);
    default:
        return true;
    }
}

bool kinetis_internal_peripheral_clock_enabled(const Kinetis* device, KinetisPeripheralId id) {
    const uint32_t scgc1 = kinetis_internal_raw_load(device, KINETIS_SIM_SCGC1, 4);
    const uint32_t scgc3 = device->timing.sim_scgc3;
    const uint32_t scgc4 = device->timing.sim_scgc4;
    const uint32_t scgc5 = device->timing.sim_scgc5;
    const uint32_t scgc6 = device->timing.sim_scgc6;
    const uint32_t scgc7 = device->timing.sim_scgc7;
    const bool extended_clock_gates =
        kinetis_profile_has_peripheral(device->profile, KINETIS_PERIPHERAL_AIPS1);
    if (device->profile->id == KINETIS_PROFILE_MKV10Z1287)
        return mkv10_clock_enabled(device, id);
    if (id >= KINETIS_PERIPHERAL_PORTA && id <= KINETIS_PERIPHERAL_PORTE) {
        return gate(scgc5, (uint8_t)(9u + id - KINETIS_PERIPHERAL_PORTA));
    }
    switch (id) {
    case KINETIS_PERIPHERAL_DMA:
        return gate(scgc7, 1);
    case KINETIS_PERIPHERAL_FB:
        return gate(scgc7, 0);
    case KINETIS_PERIPHERAL_SYSMPU:
        return gate(scgc7, 2);
    case KINETIS_PERIPHERAL_FTFA:
    case KINETIS_PERIPHERAL_FTFE:
        return gate(scgc6, 0);
    case KINETIS_PERIPHERAL_DMAMUX:
        return gate(scgc6, 1);
    case KINETIS_PERIPHERAL_CAN0:
        return gate(scgc6, 4);
    case KINETIS_PERIPHERAL_FTM0:
    case KINETIS_PERIPHERAL_FTM1:
        return gate(scgc6, (uint8_t)(24u + id - KINETIS_PERIPHERAL_FTM0));
    case KINETIS_PERIPHERAL_FTM2:
        return extended_clock_gates ? gate(scgc3, 24) : gate(scgc6, 26);
    case KINETIS_PERIPHERAL_FTM3:
        return extended_clock_gates ? gate(scgc3, 25) : gate(scgc6, 6);
    case KINETIS_PERIPHERAL_FTM4:
        return gate(scgc6, 7);
    case KINETIS_PERIPHERAL_FTM5:
        return gate(scgc6, 8);
    case KINETIS_PERIPHERAL_ADC0:
        return gate(scgc6, 27);
    case KINETIS_PERIPHERAL_ADC1:
        return extended_clock_gates ? gate(scgc3, 27) : gate(scgc6, 7);
    case KINETIS_PERIPHERAL_DAC0:
        return extended_clock_gates
                   ? gate(kinetis_internal_raw_load(device, KINETIS_SIM_SCGC2, 4), 12)
                   : gate(scgc6, 31);
    case KINETIS_PERIPHERAL_DAC1:
        return extended_clock_gates
                   ? gate(kinetis_internal_raw_load(device, KINETIS_SIM_SCGC2, 4), 13)
                   : gate(scgc6, 8);
    case KINETIS_PERIPHERAL_RNG:
        return gate(scgc6, 9);
    case KINETIS_PERIPHERAL_LPUART0:
        return gate(scgc6, 10);
    case KINETIS_PERIPHERAL_SPI0:
        return gate(scgc6, 12);
    case KINETIS_PERIPHERAL_SPI1:
        return gate(scgc6, 13);
    case KINETIS_PERIPHERAL_SPI2:
        return gate(scgc3, 12);
    case KINETIS_PERIPHERAL_SDHC:
        return gate(scgc3, 17);
    case KINETIS_PERIPHERAL_I2S0:
        return gate(scgc6, 15);
    case KINETIS_PERIPHERAL_CRC:
        return gate(scgc6, 18);
    case KINETIS_PERIPHERAL_USBDCD:
        return gate(scgc6, 21);
    case KINETIS_PERIPHERAL_PDB0:
        return gate(scgc6, 22);
    case KINETIS_PERIPHERAL_PDB1:
        return gate(scgc6, 17);
    case KINETIS_PERIPHERAL_PIT:
        return gate(scgc6, 23);
    case KINETIS_PERIPHERAL_RTC:
        return gate(scgc6, 29);
    case KINETIS_PERIPHERAL_LPTMR0:
        return gate(scgc5, 0);
    case KINETIS_PERIPHERAL_CMT:
        return gate(scgc4, 2);
    case KINETIS_PERIPHERAL_I2C0:
        return gate(scgc4, 6);
    case KINETIS_PERIPHERAL_I2C1:
        return gate(scgc4, 7);
    case KINETIS_PERIPHERAL_I2C2:
        return gate(scgc1, 6);
    case KINETIS_PERIPHERAL_UART0:
        return gate(scgc4, 10);
    case KINETIS_PERIPHERAL_UART1:
        return gate(scgc4, 11);
    case KINETIS_PERIPHERAL_UART2:
        return gate(scgc4, 12);
    case KINETIS_PERIPHERAL_UART3:
        return gate(scgc4, 13);
    case KINETIS_PERIPHERAL_UART4:
        return gate(scgc1, 10);
    case KINETIS_PERIPHERAL_UART5:
        return gate(scgc1, 11);
    case KINETIS_PERIPHERAL_USB0:
        return gate(scgc4, 18);
    case KINETIS_PERIPHERAL_CMP0:
    case KINETIS_PERIPHERAL_CMP1:
    case KINETIS_PERIPHERAL_CMP2:
        return gate(scgc4, 19);
    case KINETIS_PERIPHERAL_VREF:
        return gate(scgc4, 20);
    default:
        return true;
    }
}

void kinetis_sync_clock_gates(Kinetis* device) {
    const uint32_t scgc1 = kinetis_internal_raw_load(device, KINETIS_SIM_SCGC1, 4);
    const uint32_t scgc3 = device->timing.sim_scgc3;
    const uint32_t scgc4 = device->timing.sim_scgc4;
    const uint32_t scgc5 = device->timing.sim_scgc5;
    const uint32_t scgc6 = device->timing.sim_scgc6;
    const uint32_t scgc7 = device->timing.sim_scgc7;
    kinetis_timing_internal_refresh_ftm_pin_routes(&device->timing);
    mkv10_refresh_ftm_comparator_routes(device);
    kinetis_serial_set_clocks(&device->serial, kinetis_timing_core_clock_hz(&device->timing),
                              kinetis_timing_bus_clock_hz(&device->timing),
                              kinetis_timing_lpuart_clock_hz(&device->timing));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_LPUART0, gate(scgc6, 10));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_SPI0, gate(scgc6, 12));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_SPI1, gate(scgc6, 13));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_SPI2, gate(scgc3, 12));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_I2C0, gate(scgc4, 6));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_I2C1, gate(scgc4, 7));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_I2C2, gate(scgc1, 6));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART0, gate(scgc4, 10));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART1, gate(scgc4, 11));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART2, gate(scgc4, 12));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART3, gate(scgc4, 13));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART4, gate(scgc1, 10));
    kinetis_serial_set_clock_gate(&device->serial, KINETIS_PERIPHERAL_UART5, gate(scgc1, 11));
    kinetis_sdhc_set_clock(&device->sdhc, gate(scgc3, 17));
    for (uint8_t port = 0; port < 5; port++) {
        kinetis_io_set_clock(&device->io, (KinetisPeripheralId)(KINETIS_PERIPHERAL_PORTA + port),
                             gate(scgc5, (uint8_t)(9u + port)));
    }
    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_USB0, gate(scgc4, 18));
    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_I2S0, gate(scgc6, 15));
    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_FB, gate(scgc7, 0));
    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_CAN0, gate(scgc6, 4));
    kinetis_io_set_clock(&device->io, KINETIS_PERIPHERAL_SYSMPU, gate(scgc7, 2));
}

void kinetis_internal_refresh_serial_signals(Kinetis* device) {
    static const uint8_t irqs[KINETIS_SERIAL_IRQ_COUNT] = {
        30, 26, 27, 65, 24, 25, 74, 31, 32, 33, 34, 35, 36, 37, 38, 66, 67, 68, 69,
    };
    static const uint8_t dma_sources[KINETIS_SERIAL_DMA_COUNT] = {
        58, 59, 14, 15, 16, 16, 17, 17, 18, 19, 19, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 11,
    };
    static const uint8_t mkv10_irqs[KINETIS_SERIAL_IRQ_COUNT] = {
        UINT8_MAX, 10u,       UINT8_MAX, UINT8_MAX, 8u,        UINT8_MAX, UINT8_MAX,
        12u,       12u,       13u,       13u,       UINT8_MAX, UINT8_MAX, UINT8_MAX,
        UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX,
    };
    static const uint8_t mkv10_dma_sources[KINETIS_SERIAL_DMA_COUNT] = {
        UINT8_MAX, UINT8_MAX, 16u,       17u,       UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX,
        22u,       UINT8_MAX, UINT8_MAX, 2u,        3u,        4u,        5u,        UINT8_MAX,
        UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX,
    };
    const bool mkv10 = device->profile->id == KINETIS_PROFILE_MKV10Z1287;
    if (device->cpu != NULL) {
        for (uint8_t irq_index = 0; irq_index < KINETIS_SERIAL_IRQ_COUNT; irq_index++) {
            const uint8_t irq = mkv10 ? mkv10_irqs[irq_index] : irqs[irq_index];
            if (irq == UINT8_MAX ||
                (mkv10 && (irq_index == KINETIS_SERIAL_IRQ_UART0_ERROR ||
                           irq_index == KINETIS_SERIAL_IRQ_UART1_ERROR)))
                continue;
            bool asserted =
                kinetis_serial_irq(&device->serial, (KinetisSerialIrq)irq_index);
            if (mkv10 &&
                (irq_index == KINETIS_SERIAL_IRQ_UART0 ||
                 irq_index == KINETIS_SERIAL_IRQ_UART1))
                asserted |= kinetis_serial_irq(&device->serial,
                                               (KinetisSerialIrq)(irq_index + 1u));
            cortex_m4_set_irq_level(device->cpu, irq, asserted);
        }
    }
    for (uint8_t dma_index = 0; dma_index < KINETIS_SERIAL_DMA_COUNT; dma_index++) {
        const uint8_t dma_source = mkv10 ? mkv10_dma_sources[dma_index] : dma_sources[dma_index];
        if (dma_source != UINT8_MAX &&
            kinetis_serial_dma_request(&device->serial, (KinetisSerialDmaRequest)dma_index)) {
            kinetis_data_dma_request(device->data, dma_source);
        }
    }
}

void kinetis_refresh_signals(Kinetis* device) {
    if (device != NULL) {
        kinetis_internal_refresh_serial_signals(device);
        if (device->cpu != NULL) {
            if (device->profile->id == KINETIS_PROFILE_MKV10Z1287) {
                cortex_m4_set_irq_level(device->cpu, 30u,
                                        kinetis_io_irq_asserted(&device->io, 30u));
                cortex_m4_set_irq_level(device->cpu, 31u,
                                        kinetis_io_irq_asserted(&device->io, 31u));
                return;
            }
            static const uint8_t irqs[] = {28u, 29u, 53u, 59u, 60u, 61u, 62u, 63u, 75u};
            for (size_t irq_index = 0; irq_index < sizeof(irqs); irq_index++) {
                cortex_m4_set_irq_level(device->cpu, irqs[irq_index],
                                        kinetis_io_irq_asserted(&device->io, irqs[irq_index]));
            }
            cortex_m4_set_irq_level(device->cpu, 54u, kinetis_usbdcd_irq(&device->usbdcd));
            cortex_m4_set_irq_level(device->cpu, 81u, kinetis_sdhc_irq(&device->sdhc));
        }
    }
}

bool kinetis_internal_enable_debug_clock(Kinetis* device, KinetisPeripheralId id) {
    if (serial_peripheral(id)) {
        return kinetis_serial_set_clock_gate(&device->serial, id, true);
    }
    if (id == KINETIS_PERIPHERAL_SDHC) {
        kinetis_sdhc_set_clock(&device->sdhc, true);
        return true;
    }
    if (io_peripheral(id)) {
        kinetis_io_set_clock(&device->io, id, true);
        return true;
    }
    return false;
}

bool kinetis_internal_semantic_read(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value) {
    if (id == KINETIS_PERIPHERAL_MCG && device->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
        address == 0x40064004u && byte_count == 1u) {
        *output_value = 0u;
        return true;
    }
    if (id == KINETIS_PERIPHERAL_MCM && address == 0xf0003040u && byte_count == 4u) {
        *output_value = kinetis_internal_raw_load(device, address, byte_count) & 7u;
        return true;
    }
    if (id == KINETIS_PERIPHERAL_MTB || id == KINETIS_PERIPHERAL_MTBDWT) {
        return kinetis_internal_mtb_read(device, id, address, byte_count, output_value);
    }
    if (id == KINETIS_PERIPHERAL_MMDVSQ) {
        return mmdvsq_read(device, address, byte_count, output_value);
    }
    if (id == KINETIS_PERIPHERAL_CMT) {
        return cmt_read(device, address, byte_count, output_value);
    }
    if (id == KINETIS_PERIPHERAL_USBDCD) {
        return kinetis_usbdcd_read(&device->usbdcd, address, byte_count, output_value);
    }
    if (timing_peripheral(id) &&
        kinetis_timing_read(&device->timing, address, byte_count, output_value)) {
        return true;
    }
    if (data_peripheral(id) && kinetis_data_read(device->data, address, byte_count, output_value)) {
        return true;
    }
    if (serial_peripheral(id) &&
        kinetis_serial_read(&device->serial, address, byte_count, output_value)) {
        return true;
    }
    if (id == KINETIS_PERIPHERAL_SDHC)
        return kinetis_sdhc_read(&device->sdhc, address, byte_count, output_value);
    if (!io_peripheral(id))
        return false;
    return kinetis_io_read(&device->io, address, byte_count, output_value);
}

bool kinetis_internal_semantic_write(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value) {
    if (id == KINETIS_PERIPHERAL_MCG && device->profile->id == KINETIS_PROFILE_MKV10Z1287 &&
        address == 0x40064004u && byte_count == 1u)
        return true;
    if (id == KINETIS_PERIPHERAL_MCM && address == 0xf0003040u && byte_count == 4u) {
        const uint32_t request = write_value & 1u;
        kinetis_internal_raw_store(device, address, byte_count, (write_value & 4u) | request);
        device->cpo_entry_pending = request != 0u;
        kinetis_timing_set_cpu_only(&device->timing, false);
        return true;
    }
    if (id == KINETIS_PERIPHERAL_MTB || id == KINETIS_PERIPHERAL_MTBDWT) {
        return kinetis_internal_mtb_write(device, id, address, byte_count, write_value);
    }
    if (id == KINETIS_PERIPHERAL_MMDVSQ) {
        return mmdvsq_write(device, address, byte_count, write_value);
    }
    if (id == KINETIS_PERIPHERAL_CMT) {
        return cmt_write(device, address, byte_count, write_value);
    }
    if (id == KINETIS_PERIPHERAL_USBDCD) {
        return kinetis_usbdcd_write(&device->usbdcd, address, byte_count, write_value);
    }
    if (timing_peripheral(id) &&
        kinetis_timing_write(&device->timing, address, byte_count, write_value)) {
        kinetis_sync_clock_gates(device);
        return true;
    }
    if (data_peripheral(id) && kinetis_data_write(device->data, address, byte_count, write_value)) {
        return true;
    }
    if (serial_peripheral(id) &&
        kinetis_serial_write(&device->serial, address, byte_count, write_value)) {
        return true;
    }
    if (id == KINETIS_PERIPHERAL_SDHC)
        return kinetis_sdhc_write(&device->sdhc, address, byte_count, write_value);
    if (!io_peripheral(id))
        return false;
    return kinetis_io_write(&device->io, address, byte_count, write_value);
}

void kinetis_internal_exception_vector(void* context) {
    Kinetis* device = context;
    const uint32_t control = kinetis_internal_raw_load(device, 0xf0003040u, 4u);
    if ((control & 4u) != 0u) {
        kinetis_internal_raw_store(device, 0xf0003040u, 4u, control & ~3u);
        kinetis_timing_set_cpu_only(&device->timing, false);
    }
}

const KinetisRegisterDescriptor*
kinetis_internal_manifest_descriptor_for_access(const Kinetis* device, uint32_t address,
                                                uint8_t byte_count) {
    const KinetisRegisterDescriptor* exact =
        kinetis_register_manifest_lookup(device->profile->id, address, (uint8_t)(byte_count * 8u));
    if (exact != NULL) {
        return exact;
    }
    const KinetisRegisterManifest* manifest = device->manifest;
    const uint32_t end = address + byte_count;
    for (size_t register_index = 0u; register_index < manifest->register_count; register_index++) {
        const KinetisRegisterDescriptor* candidate = &manifest->registers[register_index];
        const uint32_t candidate_end = candidate->address + candidate->width / 8u;
        if (candidate->address <= address && candidate_end >= end)
            return candidate;
    }
    return NULL;
}

uint32_t kinetis_internal_manifest_access_mask(const KinetisRegisterDescriptor* descriptor,
                                               uint32_t address, uint32_t mask) {
    return mask >> ((address - descriptor->address) * 8u);
}
