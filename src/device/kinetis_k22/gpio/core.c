#include "device/kinetis_k22/gpio/internal.h"
#include "device/kinetis_k22/gpio/io.h"

#include <string.h>

#include "device/kinetis_k22/variants/manifest.h"

uint32_t k22_io_internal_load_bytes(const uint8_t* data, uint8_t size) {
    uint32_t output_value = 0;
    for (uint8_t byte_index = 0; byte_index < size; byte_index++)
        output_value |= (uint32_t)data[byte_index] << (byte_index * 8u);
    return output_value;
}

static uint32_t width_mask(uint8_t size) {
    return size == 4 ? UINT32_MAX : (1u << (size * 8u)) - 1u;
}

static uint32_t merge_value(uint32_t previous_value, uint32_t offset, uint8_t size,
                            uint32_t write_value) {
    const uint32_t shift = (offset & 3u) * 8u;
    const uint32_t bit_mask = width_mask(size) << shift;
    return (previous_value & ~bit_mask) | ((write_value << shift) & bit_mask);
}

void k22_io_internal_emit(K22Io* io, K22IoEventType type, uint32_t source, uint32_t event_value,
                          uint32_t auxiliary) {
    if (io->configuration.event_handler == NULL)
        return;
    const K22IoEvent event = {
        .type = type, .source = source, .value = event_value, .auxiliary = auxiliary};
    io->configuration.event_handler(io->configuration.event_context, &event);
}

void k22_io_internal_emit_can(K22Io* io, uint8_t mailbox, uint32_t identifier,
                              uint32_t control_value, uint32_t upper_word, uint32_t lower_word) {
    if (io->configuration.event_handler == NULL)
        return;
    K22IoEvent event = {K22_IO_EVENT_CAN_TRANSMIT,
                        mailbox,
                        identifier,
                        (control_value >> 16) & 15u,
                        {0},
                        0,
                        false,
                        false};
    event.length = (uint8_t)event.auxiliary;
    if (event.length > 8)
        event.length = 8;
    event.extended = (control_value & (1u << 21)) != 0;
    event.remote = (control_value & (1u << 20)) != 0;
    for (uint8_t byte_index = 0; byte_index < 4; byte_index++) {
        event.data[byte_index] = (uint8_t)(upper_word >> ((3u - byte_index) * 8u));
        event.data[byte_index + 4u] = (uint8_t)(lower_word >> ((3u - byte_index) * 8u));
    }
    io->configuration.event_handler(io->configuration.event_context, &event);
}

bool k22_io_internal_valid_size(uint8_t size) { return size == 1 || size == 2 || size == 4; }

uint8_t k22_io_internal_first_set_bit(uint32_t bit_mask) {
    uint8_t bit = 0;
    while ((bit_mask & 1u) == 0) {
        bit_mask >>= 1;
        bit++;
    }
    return bit;
}

bool k22_io_internal_pin_exists(const K22Io* io, uint8_t port, uint8_t pin) {
    return port < K22_IO_PORT_COUNT && pin < K22_IO_PIN_COUNT &&
           (io->configuration.package_pin_mask[port] & (1u << pin)) != 0;
}

static uint8_t port_index(K22PeripheralId id) {
    if (id >= K22_PERIPHERAL_PORTA && id <= K22_PERIPHERAL_PORTE)
        return (uint8_t)(id - K22_PERIPHERAL_PORTA);
    return (uint8_t)(id - K22_PERIPHERAL_GPIOA);
}

bool k22_io_internal_is_port(K22PeripheralId id) {
    return id >= K22_PERIPHERAL_PORTA && id <= K22_PERIPHERAL_PORTE;
}

bool k22_io_internal_is_gpio(K22PeripheralId id) {
    return id >= K22_PERIPHERAL_GPIOA && id <= K22_PERIPHERAL_GPIOE;
}

uint32_t k22_io_internal_pin_level_unfiltered(const K22Io* io, uint8_t port) {
    uint32_t pin_levels = 0;
    for (uint8_t pin = 0; pin < K22_IO_PIN_COUNT; pin++) {
        const uint32_t bit = 1u << pin;
        if (!k22_io_internal_pin_exists(io, port, pin))
            continue;
        const uint32_t pcr = io->port_pcr[port][pin];
        const bool is_output = (io->gpio_pddr[port] & bit) != 0 && ((pcr >> 8) & 7u) == 1u;
        const bool externally_driven = (io->gpio_external_drive[port] & bit) != 0;
        bool pin_high = false;
        if (is_output && ((pcr & K22_PORT_PCR_ODE) == 0 || (io->gpio_pdor[port] & bit) == 0)) {
            pin_high = (io->gpio_pdor[port] & bit) != 0;
        } else if (externally_driven) {
            pin_high = (io->gpio_external[port] & bit) != 0;
        } else if ((pcr & K22_PORT_PCR_PE) != 0) {
            pin_high = (pcr & K22_PORT_PCR_PS) != 0;
        }
        if (pin_high)
            pin_levels |= bit;
    }
    return pin_levels;
}

uint32_t k22_io_internal_pin_level(const K22Io* io, uint8_t port) {
    uint32_t pin_levels = k22_io_internal_pin_level_unfiltered(io, port);
    const uint32_t filtered = io->port_dfer[port] & io->configuration.package_pin_mask[port];
    pin_levels = (pin_levels & ~filtered) | (io->gpio_filtered[port] & filtered);
    return pin_levels;
}

static void update_pin_event(K22Io* io, uint8_t port, uint8_t pin, bool previous, bool current) {
    if (previous == current)
        return;
    const uint32_t interrupt_config = (io->port_pcr[port][pin] >> 16) & 15u;
    bool event_triggered = false;
    if (interrupt_config == 1u || interrupt_config == 9u)
        event_triggered = !previous && current;
    else if (interrupt_config == 2u || interrupt_config == 10u)
        event_triggered = previous && !current;
    else if (interrupt_config == 3u || interrupt_config == 11u)
        event_triggered = true;
    else if (interrupt_config == 8u)
        event_triggered = !current;
    else if (interrupt_config == 12u)
        event_triggered = current;
    if (!event_triggered)
        return;
    const uint32_t bit = 1u << pin;
    io->port_isfr[port] |= bit;
    io->port_pcr[port][pin] |= K22_PORT_PCR_ISF;
    if (interrupt_config <= 3u)
        k22_io_internal_emit(io, K22_IO_EVENT_DMA, (uint32_t)port * 32u + pin, current,
                             interrupt_config);
    else
        k22_io_internal_emit(io, K22_IO_EVENT_IRQ, 59u + port, bit, interrupt_config);
}

static void update_output_events(K22Io* io, uint8_t port, uint32_t previous_pin_level) {
    const uint32_t current_pin_level = k22_io_internal_pin_level(io, port);
    uint32_t changed = (previous_pin_level ^ current_pin_level) & io->gpio_pddr[port] &
                       io->configuration.package_pin_mask[port];
    while (changed != 0) {
        const uint8_t pin = k22_io_internal_first_set_bit(changed);
        const uint32_t bit = 1u << pin;
        k22_io_internal_emit(io, K22_IO_EVENT_GPIO_OUTPUT, (uint32_t)port * 32u + pin,
                             (current_pin_level & bit) != 0, (io->port_pcr[port][pin] >> 8) & 7u);
        changed &= ~bit;
    }
}

void k22_io_internal_commit_pin_level(K22Io* io, uint8_t port, uint8_t pin, bool previous,
                                      bool pin_high) {
    const uint32_t bit = 1u << pin;
    if (pin_high)
        io->gpio_filtered[port] |= bit;
    else
        io->gpio_filtered[port] &= ~bit;
    update_pin_event(io, port, pin, previous, pin_high);
}

bool k22_io_internal_module_clocked(const K22Io* io, K22PeripheralId id) {
    if (id == K22_PERIPHERAL_FLASH_CONFIG || id == K22_PERIPHERAL_MCM)
        return true;
    return io->clock_enabled[id];
}

K22IoConfiguration k22_io_default_configuration(const K22Profile* profile) {
    K22IoConfiguration configuration;
    memset(&configuration, 0, sizeof(configuration));
    configuration.profile = profile;
    for (uint8_t port = 0; port < K22_IO_PORT_COUNT; port++)
        configuration.package_pin_mask[port] = UINT32_MAX;
    memset(configuration.flash_configuration, 0xff, sizeof(configuration.flash_configuration));
    configuration.flash_configuration[0x0c] = 0xfeu;
    return configuration;
}

bool k22_io_init(K22Io* io, K22IoConfiguration configuration) {
    if (io == NULL || configuration.profile == NULL)
        return false;
    memset(io, 0, sizeof(*io));
    io->configuration = configuration;
    k22_io_reset(io);
    return true;
}

static void reset_peripheral_registers(const K22IoConfiguration* configuration,
                                       K22PeripheralId peripheral, uint8_t* registers,
                                       size_t capacity) {
    K22PeripheralBlock block;
    const K22RegisterManifest* manifest = k22_register_manifest_get(configuration->profile->id);
    if (manifest == NULL ||
        !k22_profile_peripheral_block(configuration->profile, peripheral, &block)) {
        return;
    }
    for (size_t index = 0u; index < manifest->register_count; index++) {
        const K22RegisterDescriptor* descriptor = &manifest->registers[index];
        const uint8_t size = (uint8_t)(descriptor->width / 8u);
        if (descriptor->address < block.address ||
            descriptor->address - block.address + size > capacity) {
            continue;
        }
        const uint32_t offset = descriptor->address - block.address;
        const uint32_t reset_value = descriptor->reset_value & descriptor->reset_mask;
        for (uint8_t byte_index = 0u; byte_index < size; byte_index++) {
            registers[offset + byte_index] = (uint8_t)(reset_value >> (byte_index * 8u));
        }
    }
}

void k22_io_reset(K22Io* io) {
    if (io == NULL)
        return;
    const K22IoConfiguration configuration = io->configuration;
    memset(io, 0, sizeof(*io));
    io->configuration = configuration;
    for (uint8_t port = 0u; port < K22_IO_PORT_COUNT; port++) {
        for (uint8_t pin = 0u; pin < K22_IO_PIN_COUNT; pin++) {
            const uint32_t address = 0x40049000u + (uint32_t)port * 0x1000u + (uint32_t)pin * 4u;
            const K22RegisterDescriptor* descriptor =
                k22_register_manifest_lookup(configuration.profile->id, address, 32u);
            if (descriptor != NULL) {
                io->port_pcr[port][pin] = descriptor->reset_value & descriptor->reset_mask;
            }
        }
    }
    io->clock_enabled[K22_PERIPHERAL_FLASH_CONFIG] = true;
    io->clock_enabled[K22_PERIPHERAL_MCM] = true;
    reset_peripheral_registers(&configuration, K22_PERIPHERAL_USB0, io->usb, sizeof(io->usb));
    reset_peripheral_registers(&configuration, K22_PERIPHERAL_CAN0, (uint8_t*)io->can,
                               sizeof(io->can));
    reset_peripheral_registers(&configuration, K22_PERIPHERAL_I2S0, (uint8_t*)io->i2s,
                               sizeof(io->i2s));
    reset_peripheral_registers(&configuration, K22_PERIPHERAL_FB, (uint8_t*)io->flexbus,
                               sizeof(io->flexbus));
    reset_peripheral_registers(&configuration, K22_PERIPHERAL_SYSMPU, (uint8_t*)io->sysmpu,
                               sizeof(io->sysmpu));
    const bool has_large_memory_profile = configuration.profile->id == K22_PROFILE_MK22FN1M012 ||
                                          configuration.profile->id == K22_PROFILE_MK22FX51212;
    const uint32_t crossbar_port_mask =
        configuration.profile->id >= K22_PROFILE_MK22FN51212 ? 0x1fu : 0x0fu;
    io->mcm[0] = (has_large_memory_profile ? 0x00370000u : 0x00170000u) | crossbar_port_mask;
    if (!has_large_memory_profile)
        io->mcm[8u / 4u] = 0x00020000u;
    for (uint8_t port = 0; port < K22_IO_PORT_COUNT; port++) {
        io->gpio_filtered[port] = k22_io_internal_pin_level_unfiltered(io, port);
        io->gpio_pending[port] = io->gpio_filtered[port];
    }
}

bool k22_io_copy(K22Io* destination, const K22Io* source) {
    if (destination == NULL || source == NULL || destination == source)
        return destination == source && destination != NULL;
    *destination = *source;
    return true;
}

void k22_io_set_clock(K22Io* io, K22PeripheralId peripheral, bool enabled) {
    if (io == NULL || peripheral < 0 || peripheral >= K22_PERIPHERAL_COUNT ||
        !k22_profile_has_peripheral(io->configuration.profile, peripheral))
        return;
    io->clock_enabled[peripheral] = enabled;
    if (k22_io_internal_is_port(peripheral))
        io->clock_enabled[K22_PERIPHERAL_GPIOA + port_index(peripheral)] = enabled;
    else if (k22_io_internal_is_gpio(peripheral))
        io->clock_enabled[K22_PERIPHERAL_PORTA + port_index(peripheral)] = enabled;
}

bool k22_io_clock_enabled(const K22Io* io, K22PeripheralId peripheral) {
    return io != NULL && peripheral >= 0 && peripheral < K22_PERIPHERAL_COUNT &&
           k22_profile_has_peripheral(io->configuration.profile, peripheral) &&
           k22_io_internal_module_clocked(io, peripheral);
}

bool k22_io_internal_read_port(K22Io* io, K22PeripheralLocation location, uint8_t size,
                               uint32_t* output_value) {
    const uint8_t port = port_index(location.id);
    if (location.offset < 0x80u) {
        const uint8_t pin = (uint8_t)(location.offset / 4u);
        if (!k22_io_internal_pin_exists(io, port, pin) || location.offset % 4u + size > 4u)
            return false;
        *output_value =
            (io->port_pcr[port][pin] >> ((location.offset & 3u) * 8u)) & width_mask(size);
        return true;
    }
    if (location.offset == 0x80u || location.offset == 0x84u) {
        if (size != 4)
            return false;
        *output_value = 0;
        return true;
    }
    if (location.offset == 0xa0u && size == 4) {
        *output_value = io->port_isfr[port];
        return true;
    }
    if (location.offset == 0xc0u && size == 4) {
        *output_value = io->port_dfer[port];
        return true;
    }
    if (location.offset == 0xc4u && size == 1) {
        *output_value = io->port_dfcr[port];
        return true;
    }
    if (location.offset == 0xc8u && size == 1) {
        *output_value = io->port_dfwr[port];
        return true;
    }
    return false;
}

static void write_pcr(K22Io* io, uint8_t port, uint8_t pin, uint32_t requested_value,
                      bool clear_isf) {
    const uint32_t previous_value = io->port_pcr[port][pin];
    if ((previous_value & K22_PORT_PCR_LK) != 0) {
        if (clear_isf) {
            io->port_pcr[port][pin] &= ~K22_PORT_PCR_ISF;
            io->port_isfr[port] &= ~(1u << pin);
        }
        return;
    }
    uint32_t register_value = requested_value & (K22_PORT_PCR_WRITABLE & ~K22_PORT_PCR_ISF);
    if (!clear_isf)
        register_value |= previous_value & K22_PORT_PCR_ISF;
    else
        io->port_isfr[port] &= ~(1u << pin);
    io->port_pcr[port][pin] = register_value;
}

bool k22_io_internal_write_port(K22Io* io, K22PeripheralLocation location, uint8_t size,
                                uint32_t write_value) {
    const uint8_t port = port_index(location.id);
    const uint32_t before = k22_io_internal_pin_level(io, port);
    if (location.offset < 0x80u) {
        const uint8_t pin = (uint8_t)(location.offset / 4u);
        if (!k22_io_internal_pin_exists(io, port, pin) || location.offset % 4u + size > 4u)
            return false;
        const uint8_t register_byte = (uint8_t)(location.offset & 3u);
        const bool clear_isf = register_byte <= 3u && register_byte + size > 3u &&
                               (write_value & (1u << ((3u - register_byte) * 8u))) != 0;
        write_pcr(io, port, pin,
                  merge_value(io->port_pcr[port][pin], location.offset, size, write_value),
                  clear_isf);
        update_output_events(io, port, before);
        return true;
    }
    if ((location.offset == 0x80u || location.offset == 0x84u) && size == 4) {
        const uint16_t register_value = (uint16_t)write_value;
        const uint16_t selected_pins = (uint16_t)(write_value >> 16);
        const uint8_t first = location.offset == 0x80u ? 0 : 16;
        for (uint8_t pin_offset = 0; pin_offset < 16; pin_offset++) {
            const uint8_t pin = first + pin_offset;
            if ((selected_pins & (1u << pin_offset)) != 0 &&
                k22_io_internal_pin_exists(io, port, pin))
                write_pcr(io, port, pin, register_value, false);
        }
        update_output_events(io, port, before);
        return true;
    }
    if (location.offset == 0xa0u && size == 4) {
        io->port_isfr[port] &= ~write_value;
        for (uint8_t pin = 0; pin < K22_IO_PIN_COUNT; pin++) {
            if ((write_value & (1u << pin)) != 0)
                io->port_pcr[port][pin] &= ~K22_PORT_PCR_ISF;
        }
        return true;
    }
    if (location.offset == 0xc0u && size == 4) {
        io->port_dfer[port] = write_value & io->configuration.package_pin_mask[port];
        io->gpio_filtered[port] = k22_io_internal_pin_level_unfiltered(io, port);
        return true;
    }
    if (location.offset == 0xc4u && size == 1) {
        io->port_dfcr[port] = (uint8_t)write_value & 1u;
        return true;
    }
    if (location.offset == 0xc8u && size == 1) {
        io->port_dfwr[port] = (uint8_t)write_value & 0x1fu;
        return true;
    }
    return false;
}

bool k22_io_internal_read_gpio(K22Io* io, K22PeripheralLocation location, uint8_t size,
                               uint32_t* output_value) {
    const uint32_t register_offset = location.offset & ~3u;
    const uint32_t byte_offset = location.offset & 3u;
    if (byte_offset + size > 4u)
        return false;
    const uint8_t port = port_index(location.id);
    uint32_t register_value = 0;
    if (register_offset == 0)
        register_value = io->gpio_pdor[port];
    else if (register_offset == 0x10u)
        register_value = k22_io_internal_pin_level(io, port);
    else if (register_offset == 0x14u)
        register_value = io->gpio_pddr[port];
    *output_value = (register_value >> (byte_offset * 8u)) & width_mask(size);
    return true;
}

bool k22_io_internal_write_gpio(K22Io* io, K22PeripheralLocation location, uint8_t size,
                                uint32_t write_value) {
    const uint32_t register_offset = location.offset & ~3u;
    const uint32_t byte_offset = location.offset & 3u;
    if (byte_offset + size > 4u)
        return false;
    const uint8_t port = port_index(location.id);
    const uint32_t package_mask = io->configuration.package_pin_mask[port];
    const uint32_t before = k22_io_internal_pin_level(io, port);
    const uint32_t shifted_value = (write_value & width_mask(size)) << (byte_offset * 8u);
    if (register_offset == 0)
        io->gpio_pdor[port] =
            merge_value(io->gpio_pdor[port], location.offset, size, write_value) & package_mask;
    else if (register_offset == 4u)
        io->gpio_pdor[port] |= shifted_value & package_mask;
    else if (register_offset == 8u)
        io->gpio_pdor[port] &= ~(shifted_value & package_mask);
    else if (register_offset == 0x0cu)
        io->gpio_pdor[port] ^= shifted_value & package_mask;
    else if (register_offset == 0x14u)
        io->gpio_pddr[port] =
            merge_value(io->gpio_pddr[port], location.offset, size, write_value) & package_mask;
    else
        return false;
    update_output_events(io, port, before);
    return true;
}
