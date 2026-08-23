#include "device/kinetis_k22/internal.h"

#include <string.h>

uint32_t kinetis_k22_internal_width_mask(uint8_t byte_count) {
    return byte_count == 1 ? 0xffu : byte_count == 2 ? 0xffffu : UINT32_MAX;
}

bool kinetis_k22_internal_pop_serial_event(K22Serial* serial, K22SerialEndpoint endpoint,
                                           K22SerialEvent* output_event) {
    for (uint8_t event_offset = 0; event_offset < serial->event_count; event_offset++) {
        const uint8_t event_index =
            (uint8_t)((serial->event_read_index + event_offset) % K22_SERIAL_EVENT_CAPACITY);
        if (serial->events[event_index].endpoint != endpoint) {
            continue;
        }
        *output_event = serial->events[event_index];
        for (uint8_t current_offset = event_offset; current_offset + 1u < serial->event_count;
             current_offset++) {
            const uint8_t destination =
                (uint8_t)((serial->event_read_index + current_offset) % K22_SERIAL_EVENT_CAPACITY);
            const uint8_t next_event_index =
                (uint8_t)((serial->event_read_index + current_offset + 1u) %
                          K22_SERIAL_EVENT_CAPACITY);
            serial->events[destination] = serial->events[next_event_index];
        }
        serial->event_write_index =
            (uint8_t)((serial->event_write_index + K22_SERIAL_EVENT_CAPACITY - 1u) %
                      K22_SERIAL_EVENT_CAPACITY);
        serial->event_count--;
        return true;
    }
    return false;
}

uint32_t kinetis_k22_internal_raw_load(const KinetisK22* device, uint32_t address,
                                       uint8_t byte_count) {
    if (address < K22_PERIPHERAL_BASE ||
        address - K22_PERIPHERAL_BASE > (uint32_t)K22_PERIPHERAL_SIZE - byte_count) {
        return 0;
    }
    const uint8_t* peripheral_bytes = device->peripheral + address - K22_PERIPHERAL_BASE;
    uint32_t register_value = 0;
    for (uint8_t byte_index = 0; byte_index < byte_count; byte_index++) {
        register_value |= (uint32_t)peripheral_bytes[byte_index] << (byte_index * 8u);
    }
    return register_value;
}

void kinetis_k22_internal_raw_store(KinetisK22* device, uint32_t address, uint8_t byte_count,
                                    uint32_t write_value) {
    if (address < K22_PERIPHERAL_BASE ||
        address - K22_PERIPHERAL_BASE > (uint32_t)K22_PERIPHERAL_SIZE - byte_count) {
        return;
    }
    uint8_t* peripheral_bytes = device->peripheral + address - K22_PERIPHERAL_BASE;
    for (uint8_t byte_index = 0; byte_index < byte_count; byte_index++) {
        peripheral_bytes[byte_index] = (uint8_t)(write_value >> (byte_index * 8u));
    }
}

bool kinetis_k22_internal_aips_access_allowed(const KinetisK22* device, uint32_t address,
                                              CortexM4Access access, bool write_access) {
    if (device->profile->id < K22_PROFILE_MK22FN1M012 || access == CORTEX_M4_ACCESS_DEBUG ||
        address < K22_PERIPHERAL_BASE || address >= K22_PERIPHERAL_BASE + K22_PERIPHERAL_SIZE) {
        return true;
    }
    const uint32_t aperture = address < K22_AIPS1 ? K22_AIPS0 : K22_AIPS1;
    const uint8_t port_slot = (uint8_t)((address - aperture) >> 12u);
    const uint32_t access_control = kinetis_k22_internal_raw_load(
        device, aperture + 0x20u + (uint32_t)(port_slot / 8u) * 4u, 4u);
    const uint8_t permission_shift = (uint8_t)((7u - port_slot % 8u) * 4u);
    const uint8_t access_permission = (uint8_t)(access_control >> permission_shift) & 7u;
    if (write_access && (access_permission & 2u) != 0u) {
        return false;
    }
    return !cortex_m4_access_is_unprivileged_data(device->cpu, access) ||
           (access_permission & 4u) == 0u;
}

bool kinetis_k22_internal_axbs_write_allowed(const KinetisK22* device, uint32_t address) {
    if (address < K22_AXBS || address >= K22_AXBS + 0x500u) {
        return true;
    }
    const uint32_t peripheral_offset = address - K22_AXBS;
    const uint32_t register_offset = peripheral_offset & 0xffu;
    if (register_offset != 0u && register_offset != 0x10u) {
        return true;
    }
    const uint32_t write_control =
        kinetis_k22_internal_raw_load(device, address - register_offset + 0x10u, 4u);
    return (write_control & 0x80000000u) == 0u;
}

static void cmt_clear_eoc(KinetisK22* device);

static bool cmt_read(KinetisK22* device, uint32_t address, uint8_t byte_count,
                     uint32_t* output_value) {
    if (address < K22_CMT || address >= K22_CMT + 0x0cu || byte_count != 1u) {
        return false;
    }
    *output_value = kinetis_k22_internal_raw_load(device, address, 1u);
    if (address == K22_CMT + 5u && (*output_value & 0x80u) != 0u)
        device->cmt_eoc_read = true;
    else if ((address == K22_CMT + 7u || address == K22_CMT + 9u) && device->cmt_eoc_read &&
             (kinetis_k22_internal_raw_load(device, K22_CMT + 0x0bu, 1u) & 1u) == 0u)
        cmt_clear_eoc(device);
    return true;
}

static void cmt_refresh_irq(KinetisK22* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u);
    const uint8_t dma = (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 0x0bu, 1u);
    if (device->cpu != NULL)
        cortex_m4_set_irq_level(device->cpu, 45u,
                                (cmt_control & 0x82u) == 0x82u && (dma & 1u) == 0u);
}

static void cmt_refresh_dma(KinetisK22* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u);
    const uint8_t dma = (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 0x0bu, 1u);
    if ((cmt_control & 0x82u) == 0x82u && (dma & 1u) != 0u && !device->cmt_dma_pending)
        device->cmt_dma_pending = k22_data_dma_request(device->data, 47u);
}

static void cmt_raise_cycle(KinetisK22* device, uint8_t cmt_control) {
    kinetis_k22_internal_raw_store(device, K22_CMT + 5u, 1u, cmt_control | 0x80u);
    cmt_refresh_dma(device);
    cmt_refresh_irq(device);
}

static uint64_t cmt_clock_ticks(const KinetisK22* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u);
    const uint64_t clock_divider = kinetis_k22_internal_raw_load(device, K22_CMT + 0x0au, 1u) + 1u;
    return clock_divider << ((cmt_control >> 5u) & 3u);
}

static void cmt_load_cycle(KinetisK22* device) {
    const uint8_t cmt_control = (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u);
    const uint64_t clock_ticks = cmt_clock_ticks(device);
    const uint64_t mark_count = (kinetis_k22_internal_raw_load(device, K22_CMT + 6u, 1u) << 8u) |
                                kinetis_k22_internal_raw_load(device, K22_CMT + 7u, 1u);
    const uint64_t space_count = (kinetis_k22_internal_raw_load(device, K22_CMT + 8u, 1u) << 8u) |
                                 kinetis_k22_internal_raw_load(device, K22_CMT + 9u, 1u);
    uint64_t time_unit_ticks = clock_ticks * 8u;
    const uint8_t carrier_offset = device->cmt_fsk_secondary ? 2u : 0u;
    const uint64_t carrier_high_count =
        kinetis_k22_internal_raw_load(device, K22_CMT + carrier_offset, 1u);
    const uint64_t carrier_low_count =
        kinetis_k22_internal_raw_load(device, K22_CMT + carrier_offset + 1u, 1u);
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

static void cmt_start(KinetisK22* device, uint8_t cmt_control) {
    device->cmt_running = true;
    device->cmt_stop_pending = false;
    device->cmt_fsk_secondary = false;
    cmt_load_cycle(device);
    const uint64_t clock_divider = kinetis_k22_internal_raw_load(device, K22_CMT + 0x0au, 1u);
    device->cmt_output_delay_ticks =
        ((cmt_control >> 5u) & 3u) == 0u ? clock_divider + 2u : clock_divider * 2u + 3u;
    device->cmt_carrier_offset_ticks = device->cmt_output_delay_ticks;
    cmt_raise_cycle(device, cmt_control);
}

static void cmt_clear_eoc(KinetisK22* device) {
    kinetis_k22_internal_raw_store(device, K22_CMT + 5u, 1u,
                                   kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u) & 0x7fu);
    device->cmt_eoc_read = false;
    cmt_refresh_irq(device);
}

static bool cmt_write(KinetisK22* device, uint32_t address, uint8_t byte_count,
                      uint32_t write_value) {
    if (address < K22_CMT || address >= K22_CMT + 0x0cu || byte_count != 1u)
        return false;
    const uint32_t register_offset = address - K22_CMT;
    if (register_offset == 5u) {
        const uint8_t previous_control =
            (uint8_t)kinetis_k22_internal_raw_load(device, address, 1u);
        const uint8_t cmt_control = (uint8_t)(write_value & 0x7fu);
        kinetis_k22_internal_raw_store(device, address, 1u,
                                       (previous_control & 0x80u) | cmt_control);
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
    const K22RegisterDescriptor* descriptor =
        k22_register_manifest_lookup(device->profile->id, address, 8u);
    if (descriptor == NULL || (descriptor->access & K22_REGISTER_ACCESS_WRITE) == 0u)
        return false;
    kinetis_k22_internal_raw_store(device, address, 1u, write_value & descriptor->write_mask);
    if ((register_offset == 7u || register_offset == 9u) && device->cmt_eoc_read &&
        (kinetis_k22_internal_raw_load(device, K22_CMT + 0x0bu, 1u) & 1u) == 0u)
        cmt_clear_eoc(device);
    if (register_offset == 0x0bu) {
        cmt_refresh_dma(device);
        cmt_refresh_irq(device);
    }
    return true;
}

void kinetis_k22_internal_cmt_advance(KinetisK22* device, uint32_t cycle_count) {
    if (!device->cmt_running ||
        (device->cpu != NULL && device->cpu->sleeping && (device->cpu->scr & 4u) != 0u))
        return;
    const uint64_t core_clock_hz = k22_timing_core_clock_hz(&device->timing);
    const uint64_t bus_clock_hz = k22_timing_bus_clock_hz(&device->timing);
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
            (kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u) & 1u) == 0u) {
            device->cmt_running = false;
            device->cmt_stop_pending = false;
            device->cmt_cycles = 0u;
            break;
        }
        const uint8_t cmt_control =
            (uint8_t)kinetis_k22_internal_raw_load(device, K22_CMT + 5u, 1u);
        if ((cmt_control & 0x0cu) == 4u)
            device->cmt_fsk_secondary = !device->cmt_fsk_secondary;
        cmt_load_cycle(device);
        cmt_raise_cycle(device, cmt_control);
    }
}

static bool serial_peripheral(K22PeripheralId id) {
    return id == K22_PERIPHERAL_LPUART0 || id == K22_PERIPHERAL_SPI0 || id == K22_PERIPHERAL_SPI1 ||
           id == K22_PERIPHERAL_SPI2 || id == K22_PERIPHERAL_I2C0 || id == K22_PERIPHERAL_I2C1 ||
           id == K22_PERIPHERAL_I2C2 || id == K22_PERIPHERAL_UART0 || id == K22_PERIPHERAL_UART1 ||
           id == K22_PERIPHERAL_UART2 || id == K22_PERIPHERAL_UART3 || id == K22_PERIPHERAL_UART4 ||
           id == K22_PERIPHERAL_UART5;
}

static bool package_serial_extension(K22PeripheralId id) {
    return id == K22_PERIPHERAL_SPI1 || id == K22_PERIPHERAL_SPI2 || id == K22_PERIPHERAL_UART3 ||
           id == K22_PERIPHERAL_UART4 || id == K22_PERIPHERAL_UART5;
}

bool kinetis_k22_internal_manifest_extension(K22PeripheralId id) {
    return package_serial_extension(id) || id == K22_PERIPHERAL_SDHC || id == K22_PERIPHERAL_DAC1;
}

K22PeripheralId kinetis_k22_internal_serial_endpoint_peripheral(KinetisK22SerialEndpoint endpoint) {
    static const K22PeripheralId peripherals[KINETIS_K22_SERIAL_ENDPOINT_COUNT] = {
        K22_PERIPHERAL_LPUART0, K22_PERIPHERAL_SPI0,  K22_PERIPHERAL_SPI1,  K22_PERIPHERAL_SPI2,
        K22_PERIPHERAL_I2C0,    K22_PERIPHERAL_I2C1,  K22_PERIPHERAL_I2C2,  K22_PERIPHERAL_UART0,
        K22_PERIPHERAL_UART1,   K22_PERIPHERAL_UART2, K22_PERIPHERAL_UART3, K22_PERIPHERAL_UART4,
        K22_PERIPHERAL_UART5,
    };
    return endpoint < KINETIS_K22_SERIAL_ENDPOINT_COUNT ? peripherals[endpoint]
                                                        : K22_PERIPHERAL_COUNT;
}

bool kinetis_k22_internal_serial_endpoint_available(const KinetisK22* device,
                                                    KinetisK22SerialEndpoint endpoint) {
    const K22PeripheralId peripheral = kinetis_k22_internal_serial_endpoint_peripheral(endpoint);
    return device != NULL && peripheral < K22_PERIPHERAL_COUNT &&
           k22_package_has_peripheral(device->package, peripheral);
}

static bool data_peripheral(K22PeripheralId id) {
    return id == K22_PERIPHERAL_FLASH_CONFIG || id == K22_PERIPHERAL_DMA ||
           id == K22_PERIPHERAL_FTFA || id == K22_PERIPHERAL_FTFE || id == K22_PERIPHERAL_DMAMUX ||
           id == K22_PERIPHERAL_ADC0 || id == K22_PERIPHERAL_ADC1 || id == K22_PERIPHERAL_DAC0 ||
           id == K22_PERIPHERAL_DAC1 || id == K22_PERIPHERAL_RNG || id == K22_PERIPHERAL_CRC ||
           id == K22_PERIPHERAL_CMP0 || id == K22_PERIPHERAL_CMP1 || id == K22_PERIPHERAL_CMP2 ||
           id == K22_PERIPHERAL_VREF;
}

static bool timing_peripheral(K22PeripheralId id) {
    return id == K22_PERIPHERAL_FTM0 || id == K22_PERIPHERAL_FTM1 || id == K22_PERIPHERAL_FTM2 ||
           id == K22_PERIPHERAL_FTM3 || id == K22_PERIPHERAL_PDB0 || id == K22_PERIPHERAL_PIT ||
           id == K22_PERIPHERAL_RTC || id == K22_PERIPHERAL_LPTMR0 || id == K22_PERIPHERAL_SIM ||
           id == K22_PERIPHERAL_WDOG || id == K22_PERIPHERAL_EWM || id == K22_PERIPHERAL_MCG ||
           id == K22_PERIPHERAL_OSC || id == K22_PERIPHERAL_LLWU || id == K22_PERIPHERAL_PMC ||
           id == K22_PERIPHERAL_SMC || id == K22_PERIPHERAL_RCM;
}

static bool io_peripheral(K22PeripheralId id) {
    return id == K22_PERIPHERAL_FB || id == K22_PERIPHERAL_SYSMPU || id == K22_PERIPHERAL_CAN0 ||
           id == K22_PERIPHERAL_I2S0 || id == K22_PERIPHERAL_USB0 || id == K22_PERIPHERAL_PORTA ||
           id == K22_PERIPHERAL_PORTB || id == K22_PERIPHERAL_PORTC || id == K22_PERIPHERAL_PORTD ||
           id == K22_PERIPHERAL_PORTE || id == K22_PERIPHERAL_GPIOA || id == K22_PERIPHERAL_GPIOB ||
           id == K22_PERIPHERAL_GPIOC || id == K22_PERIPHERAL_GPIOD || id == K22_PERIPHERAL_GPIOE ||
           id == K22_PERIPHERAL_MCM;
}

static uint8_t data_irq(K22DataInterrupt interrupt) {
    static const uint8_t irqs[K22_DATA_INTERRUPT_COUNT] = {
        0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13,
        14, 15, 16, 18, 19, 39, 73, 56, 72, 40, 41, 70, 23,
    };
    return irqs[interrupt];
}

static void adc_alternate_trigger(KinetisK22* device, uint8_t trigger_source) {
    for (uint8_t adc_instance = 0u; adc_instance < 2u; adc_instance++) {
        const uint8_t selection = (uint8_t)(device->timing.sim_sopt7 >> (adc_instance * 8u));
        if ((selection & 0x80u) != 0u && (selection & 15u) == trigger_source)
            k22_data_adc_pretrigger(device->data, adc_instance, (uint8_t)((selection >> 4u) & 1u));
    }
}

void kinetis_k22_internal_data_interrupt(void* context, K22DataInterrupt interrupt, bool asserted) {
    KinetisK22* device = context;
    if (interrupt >= K22_DATA_INTERRUPT_CMP0 && interrupt <= K22_DATA_INTERRUPT_CMP2 &&
        device->data != NULL) {
        const uint8_t comparator_instance = (uint8_t)(interrupt - K22_DATA_INTERRUPT_CMP0);
        bool comparator_high = false;
        if (k22_data_get_cmp_output(device->data, comparator_instance, &comparator_high)) {
            if (comparator_instance == 0u)
                k22_timing_set_lptmr_input(&device->timing, 0u, comparator_high);
            if (comparator_high && !device->comparator_output[comparator_instance])
                adc_alternate_trigger(device, (uint8_t)(1u + comparator_instance));
            device->comparator_output[comparator_instance] = comparator_high;
        }
    }
    if (device->cpu != NULL && interrupt < K22_DATA_INTERRUPT_COUNT) {
        cortex_m4_set_irq_level(device->cpu, data_irq(interrupt), asserted);
    }
}

void kinetis_k22_internal_data_dma_complete(void* context, uint8_t request_source) {
    KinetisK22* device = context;
    if (request_source == 47u && device->cmt_dma_pending) {
        device->cmt_dma_pending = false;
        cmt_clear_eoc(device);
    }
}

bool kinetis_k22_internal_data_bus_read(void* context, uint32_t address, uint8_t byte_count,
                                        uint32_t* output_value) {
    return kinetis_k22_dma_read(context, address, byte_count, output_value);
}

bool kinetis_k22_internal_data_bus_write(void* context, uint32_t address, uint8_t byte_count,
                                         uint32_t write_value) {
    return kinetis_k22_dma_write(context, address, byte_count, write_value);
}

bool kinetis_k22_internal_flash_bus_write(void* context, uint32_t address, uint8_t byte_count,
                                          uint32_t write_value) {
    return kinetis_k22_flash_controller_write(context, address, byte_count, write_value);
}

void kinetis_k22_internal_timing_irq(void* context, uint8_t irq, bool asserted) {
    KinetisK22* device = context;
    if (device->cpu != NULL) {
        cortex_m4_set_irq_level(device->cpu, irq, asserted);
    }
}

void kinetis_k22_internal_timing_dma(void* context, uint8_t request_source) {
    KinetisK22* device = context;
    k22_data_dma_request(device->data, request_source);
}

void kinetis_k22_internal_timing_dma_trigger(void* context, uint8_t channel) {
    KinetisK22* device = context;
    k22_data_dma_trigger(device->data, channel);
}

void kinetis_k22_internal_timing_reset(void* context, uint8_t reset_cause_0,
                                       uint8_t reset_cause_1) {
    kinetis_k22_warm_reset(context, reset_cause_0, reset_cause_1);
}

void kinetis_k22_internal_timing_trigger(void* context, K22TimingTrigger type, uint8_t instance,
                                         uint8_t channel) {
    KinetisK22* device = context;
    if (type == K22_TIMING_TRIGGER_PDB_ADC) {
        const uint8_t selection = (uint8_t)(device->timing.sim_sopt7 >> (instance * 8u));
        if ((selection & 0x80u) == 0u)
            k22_data_adc_pretrigger(device->data, instance, channel);
    } else if (type == K22_TIMING_TRIGGER_PDB_DAC) {
        k22_data_dac_trigger(device->data, instance);
    } else {
        adc_alternate_trigger(device, instance);
    }
}

static void queue_event(KinetisK22* device, const K22IoEvent* incoming_event) {
    if (device->event_count == K22_EVENT_CAPACITY) {
        device->event_read_index = (uint8_t)((device->event_read_index + 1u) % K22_EVENT_CAPACITY);
        device->event_count--;
    }
    KinetisK22Event* event = &device->events[device->event_write_index];
    event->type = (KinetisK22EventType)incoming_event->type;
    event->source = incoming_event->source;
    event->value = incoming_event->value;
    event->auxiliary = incoming_event->auxiliary;
    memcpy(event->data, incoming_event->data, sizeof(event->data));
    event->length = incoming_event->length;
    event->extended = incoming_event->extended;
    event->remote = incoming_event->remote;
    device->event_write_index = (uint8_t)((device->event_write_index + 1u) % K22_EVENT_CAPACITY);
    device->event_count++;
}

void kinetis_k22_internal_io_event(void* context, const K22IoEvent* event) {
    KinetisK22* device = context;
    queue_event(device, event);
    if (event->type == K22_IO_EVENT_DMA) {
        const uint8_t request_source = event->auxiliary == 0 ? (event->source == 0 ? 13u : 12u)
                                                             : (uint8_t)(49u + event->source / 32u);
        k22_data_dma_request(device->data, request_source);
    }
}

static bool gate(uint32_t register_value, uint8_t bit) {
    return (register_value & (1u << bit)) != 0;
}

bool kinetis_k22_internal_peripheral_clock_enabled(const KinetisK22* device, K22PeripheralId id) {
    const uint32_t scgc1 = kinetis_k22_internal_raw_load(device, K22_SIM_SCGC1, 4);
    const uint32_t scgc3 = device->timing.sim_scgc3;
    const uint32_t scgc4 = device->timing.sim_scgc4;
    const uint32_t scgc5 = device->timing.sim_scgc5;
    const uint32_t scgc6 = device->timing.sim_scgc6;
    const uint32_t scgc7 = device->timing.sim_scgc7;
    if (id >= K22_PERIPHERAL_PORTA && id <= K22_PERIPHERAL_PORTE) {
        return gate(scgc5, (uint8_t)(9u + id - K22_PERIPHERAL_PORTA));
    }
    if (id >= K22_PERIPHERAL_GPIOA && id <= K22_PERIPHERAL_GPIOE) {
        return gate(scgc5, (uint8_t)(9u + id - K22_PERIPHERAL_GPIOA));
    }
    switch (id) {
    case K22_PERIPHERAL_DMA:
        return gate(scgc7, 1);
    case K22_PERIPHERAL_FB:
        return gate(scgc7, 0);
    case K22_PERIPHERAL_SYSMPU:
        return gate(scgc7, 2);
    case K22_PERIPHERAL_FTFA:
    case K22_PERIPHERAL_FTFE:
        return gate(scgc6, 0);
    case K22_PERIPHERAL_DMAMUX:
        return gate(scgc6, 1);
    case K22_PERIPHERAL_CAN0:
        return gate(scgc6, 4);
    case K22_PERIPHERAL_FTM0:
    case K22_PERIPHERAL_FTM1:
        return gate(scgc6, (uint8_t)(24u + id - K22_PERIPHERAL_FTM0));
    case K22_PERIPHERAL_FTM2:
        return device->profile->id >= K22_PROFILE_MK22FN1M012 ? gate(scgc3, 24) : gate(scgc6, 26);
    case K22_PERIPHERAL_FTM3:
        return device->profile->id >= K22_PROFILE_MK22FN1M012 ? gate(scgc3, 25) : gate(scgc6, 6);
    case K22_PERIPHERAL_ADC0:
        return gate(scgc6, 27);
    case K22_PERIPHERAL_ADC1:
        return device->profile->id >= K22_PROFILE_MK22FN1M012 ? gate(scgc3, 27) : gate(scgc6, 7);
    case K22_PERIPHERAL_DAC0:
        return device->profile->id >= K22_PROFILE_MK22FN1M012
                   ? gate(kinetis_k22_internal_raw_load(device, K22_SIM_SCGC2, 4), 12)
                   : gate(scgc6, 31);
    case K22_PERIPHERAL_DAC1:
        return device->profile->id >= K22_PROFILE_MK22FN1M012
                   ? gate(kinetis_k22_internal_raw_load(device, K22_SIM_SCGC2, 4), 13)
                   : gate(scgc6, 8);
    case K22_PERIPHERAL_RNG:
        return gate(scgc6, 9);
    case K22_PERIPHERAL_LPUART0:
        return gate(scgc6, 10);
    case K22_PERIPHERAL_SPI0:
        return gate(scgc6, 12);
    case K22_PERIPHERAL_SPI1:
        return gate(scgc6, 13);
    case K22_PERIPHERAL_SPI2:
        return gate(scgc3, 12);
    case K22_PERIPHERAL_SDHC:
        return gate(scgc3, 17);
    case K22_PERIPHERAL_I2S0:
        return gate(scgc6, 15);
    case K22_PERIPHERAL_CRC:
        return gate(scgc6, 18);
    case K22_PERIPHERAL_USBDCD:
        return gate(scgc6, 21);
    case K22_PERIPHERAL_PDB0:
        return gate(scgc6, 22);
    case K22_PERIPHERAL_PIT:
        return gate(scgc6, 23);
    case K22_PERIPHERAL_RTC:
        return gate(scgc6, 29);
    case K22_PERIPHERAL_LPTMR0:
        return gate(scgc5, 0);
    case K22_PERIPHERAL_CMT:
        return gate(scgc4, 2);
    case K22_PERIPHERAL_I2C0:
        return gate(scgc4, 6);
    case K22_PERIPHERAL_I2C1:
        return gate(scgc4, 7);
    case K22_PERIPHERAL_I2C2:
        return gate(scgc1, 6);
    case K22_PERIPHERAL_UART0:
        return gate(scgc4, 10);
    case K22_PERIPHERAL_UART1:
        return gate(scgc4, 11);
    case K22_PERIPHERAL_UART2:
        return gate(scgc4, 12);
    case K22_PERIPHERAL_UART3:
        return gate(scgc4, 13);
    case K22_PERIPHERAL_UART4:
        return gate(scgc1, 10);
    case K22_PERIPHERAL_UART5:
        return gate(scgc1, 11);
    case K22_PERIPHERAL_USB0:
        return gate(scgc4, 18);
    case K22_PERIPHERAL_CMP0:
    case K22_PERIPHERAL_CMP1:
    case K22_PERIPHERAL_CMP2:
        return gate(scgc4, 19);
    case K22_PERIPHERAL_VREF:
        return gate(scgc4, 20);
    default:
        return true;
    }
}

void kinetis_k22_sync_clock_gates(KinetisK22* device) {
    const uint32_t scgc1 = kinetis_k22_internal_raw_load(device, K22_SIM_SCGC1, 4);
    const uint32_t scgc3 = device->timing.sim_scgc3;
    const uint32_t scgc4 = device->timing.sim_scgc4;
    const uint32_t scgc5 = device->timing.sim_scgc5;
    const uint32_t scgc6 = device->timing.sim_scgc6;
    const uint32_t scgc7 = device->timing.sim_scgc7;
    k22_serial_set_clocks(&device->serial, k22_timing_core_clock_hz(&device->timing),
                          k22_timing_bus_clock_hz(&device->timing));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_LPUART0, gate(scgc6, 10));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_SPI0, gate(scgc6, 12));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_SPI1, gate(scgc6, 13));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_SPI2, gate(scgc3, 12));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_I2C0, gate(scgc4, 6));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_I2C1, gate(scgc4, 7));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_I2C2, gate(scgc1, 6));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART0, gate(scgc4, 10));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART1, gate(scgc4, 11));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART2, gate(scgc4, 12));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART3, gate(scgc4, 13));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART4, gate(scgc1, 10));
    k22_serial_set_clock_gate(&device->serial, K22_PERIPHERAL_UART5, gate(scgc1, 11));
    k22_sdhc_set_clock(&device->sdhc, gate(scgc3, 17));
    for (uint8_t port = 0; port < 5; port++) {
        k22_io_set_clock(&device->io, (K22PeripheralId)(K22_PERIPHERAL_PORTA + port),
                         gate(scgc5, (uint8_t)(9u + port)));
    }
    k22_io_set_clock(&device->io, K22_PERIPHERAL_USB0, gate(scgc4, 18));
    k22_io_set_clock(&device->io, K22_PERIPHERAL_I2S0, gate(scgc6, 15));
    k22_io_set_clock(&device->io, K22_PERIPHERAL_FB, gate(scgc7, 0));
    k22_io_set_clock(&device->io, K22_PERIPHERAL_CAN0, gate(scgc6, 4));
    k22_io_set_clock(&device->io, K22_PERIPHERAL_SYSMPU, gate(scgc7, 2));
}

void kinetis_k22_internal_refresh_serial_signals(KinetisK22* device) {
    static const uint8_t irqs[K22_SERIAL_IRQ_COUNT] = {
        30, 26, 27, 65, 24, 25, 74, 31, 32, 33, 34, 35, 36, 37, 38, 66, 67, 68, 69,
    };
    static const uint8_t dma_sources[K22_SERIAL_DMA_COUNT] = {
        58, 59, 14, 15, 16, 16, 17, 17, 18, 19,        19,        2,
        3,  4,  5,  6,  7,  8,  9,  10, 10, UINT8_MAX, UINT8_MAX,
    };
    if (device->cpu != NULL) {
        for (uint8_t irq_index = 0; irq_index < K22_SERIAL_IRQ_COUNT; irq_index++) {
            cortex_m4_set_irq_level(device->cpu, irqs[irq_index],
                                    k22_serial_irq(&device->serial, (K22SerialIrq)irq_index));
        }
    }
    for (uint8_t dma_index = 0; dma_index < K22_SERIAL_DMA_COUNT; dma_index++) {
        if (dma_sources[dma_index] != UINT8_MAX &&
            k22_serial_dma_request(&device->serial, (K22SerialDmaRequest)dma_index)) {
            k22_data_dma_request(device->data, dma_sources[dma_index]);
        }
    }
}

void kinetis_k22_refresh_signals(KinetisK22* device) {
    if (device != NULL) {
        kinetis_k22_internal_refresh_serial_signals(device);
        if (device->cpu != NULL) {
            static const uint8_t irqs[] = {28u, 29u, 53u, 59u, 60u, 61u, 62u, 63u, 75u};
            for (size_t irq_index = 0; irq_index < sizeof(irqs); irq_index++) {
                cortex_m4_set_irq_level(device->cpu, irqs[irq_index],
                                        k22_io_irq_asserted(&device->io, irqs[irq_index]));
            }
            cortex_m4_set_irq_level(device->cpu, 54u, k22_usbdcd_irq(&device->usbdcd));
            cortex_m4_set_irq_level(device->cpu, 81u, k22_sdhc_irq(&device->sdhc));
        }
    }
}

bool kinetis_k22_internal_enable_debug_clock(KinetisK22* device, K22PeripheralId id) {
    if (serial_peripheral(id)) {
        return k22_serial_set_clock_gate(&device->serial, id, true);
    }
    if (id == K22_PERIPHERAL_SDHC) {
        k22_sdhc_set_clock(&device->sdhc, true);
        return true;
    }
    if (io_peripheral(id)) {
        k22_io_set_clock(&device->io, id, true);
        return true;
    }
    return false;
}

bool kinetis_k22_internal_semantic_read(KinetisK22* device, K22PeripheralId id, uint32_t address,
                                        uint8_t byte_count, uint32_t* output_value) {
    if (id == K22_PERIPHERAL_CMT) {
        return cmt_read(device, address, byte_count, output_value);
    }
    if (id == K22_PERIPHERAL_USBDCD) {
        return k22_usbdcd_read(&device->usbdcd, address, byte_count, output_value);
    }
    if (timing_peripheral(id) &&
        k22_timing_read(&device->timing, address, byte_count, output_value)) {
        return true;
    }
    if (data_peripheral(id) && k22_data_read(device->data, address, byte_count, output_value)) {
        return true;
    }
    if (serial_peripheral(id) &&
        k22_serial_read(&device->serial, address, byte_count, output_value)) {
        return true;
    }
    if (id == K22_PERIPHERAL_SDHC)
        return k22_sdhc_read(&device->sdhc, address, byte_count, output_value);
    if (!io_peripheral(id))
        return false;
    return k22_io_read(&device->io, address, byte_count, output_value);
}

bool kinetis_k22_internal_semantic_write(KinetisK22* device, K22PeripheralId id, uint32_t address,
                                         uint8_t byte_count, uint32_t write_value) {
    if (id == K22_PERIPHERAL_CMT) {
        return cmt_write(device, address, byte_count, write_value);
    }
    if (id == K22_PERIPHERAL_USBDCD) {
        return k22_usbdcd_write(&device->usbdcd, address, byte_count, write_value);
    }
    if (timing_peripheral(id) &&
        k22_timing_write(&device->timing, address, byte_count, write_value)) {
        kinetis_k22_sync_clock_gates(device);
        return true;
    }
    if (data_peripheral(id) && k22_data_write(device->data, address, byte_count, write_value)) {
        return true;
    }
    if (serial_peripheral(id) &&
        k22_serial_write(&device->serial, address, byte_count, write_value)) {
        return true;
    }
    if (id == K22_PERIPHERAL_SDHC)
        return k22_sdhc_write(&device->sdhc, address, byte_count, write_value);
    if (!io_peripheral(id))
        return false;
    return k22_io_write(&device->io, address, byte_count, write_value);
}

const K22RegisterDescriptor*
kinetis_k22_internal_manifest_descriptor_for_access(const KinetisK22* device, uint32_t address,
                                                    uint8_t byte_count) {
    const K22RegisterDescriptor* exact =
        k22_register_manifest_lookup(device->profile->id, address, (uint8_t)(byte_count * 8u));
    if (exact != NULL) {
        return exact;
    }
    const K22RegisterManifest* manifest = device->manifest;
    const uint32_t end = address + byte_count;
    for (size_t register_index = 0u; register_index < manifest->register_count; register_index++) {
        const K22RegisterDescriptor* candidate = &manifest->registers[register_index];
        const uint32_t candidate_end = candidate->address + candidate->width / 8u;
        if (candidate->address <= address && candidate_end >= end)
            return candidate;
    }
    return NULL;
}

uint32_t kinetis_k22_internal_manifest_access_mask(const K22RegisterDescriptor* descriptor,
                                                   uint32_t address, uint32_t mask) {
    return mask >> ((address - descriptor->address) * 8u);
}
