#include "device/kinetis/gpio/internal.h"

bool kinetis_io_read(KinetisIo* io, uint32_t address, uint8_t size, uint32_t* output_value) {
    if (io == NULL || output_value == NULL || !kinetis_io_internal_valid_size(size))
        return false;
    if (io->configuration.profile->cpu.architecture == KINETIS_CPU_ARCHITECTURE_ARMV7E_M &&
        address >= KINETIS_BIT_BAND_BASE && address < KINETIS_BIT_BAND_LIMIT && size == 4) {
        const uint32_t alias_address = address - KINETIS_BIT_BAND_BASE;
        const uint32_t source_byte_address = KINETIS_PERIPHERAL_BASE + alias_address / 32u;
        const uint8_t bit_index = (uint8_t)((alias_address / 4u) & 7u);
        uint32_t source_byte_value = 0;
        if (!kinetis_io_internal_read_direct(io, source_byte_address, 1, &source_byte_value))
            return false;
        *output_value = (source_byte_value >> bit_index) & 1u;
        return true;
    }
    return kinetis_io_internal_read_direct(io, address, size, output_value);
}

bool kinetis_io_write(KinetisIo* io, uint32_t address, uint8_t size, uint32_t write_value) {
    if (io == NULL || !kinetis_io_internal_valid_size(size))
        return false;
    if (io->configuration.profile->cpu.architecture == KINETIS_CPU_ARCHITECTURE_ARMV7E_M &&
        address >= KINETIS_BIT_BAND_BASE && address < KINETIS_BIT_BAND_LIMIT && size == 4) {
        const uint32_t alias_address = address - KINETIS_BIT_BAND_BASE;
        const uint32_t source_byte_address = KINETIS_PERIPHERAL_BASE + alias_address / 32u;
        const uint8_t bit_index = (uint8_t)((alias_address / 4u) & 7u);
        uint32_t source_byte_value = 0;
        if (!kinetis_io_internal_read_direct(io, source_byte_address, 1, &source_byte_value))
            return false;
        source_byte_value = (write_value & 1u) != 0 ? source_byte_value | (1u << bit_index)
                                                    : source_byte_value & ~(1u << bit_index);
        return kinetis_io_internal_write_direct(io, source_byte_address, 1, source_byte_value);
    }
    return kinetis_io_internal_write_direct(io, address, size, write_value);
}

bool kinetis_io_drive_pin(KinetisIo* io, uint8_t port, uint8_t pin, bool pin_high) {
    if (io == NULL || !kinetis_io_internal_pin_exists(io, port, pin))
        return false;
    const uint32_t bit = 1u << pin;
    const bool previous_level = (kinetis_io_internal_pin_level(io, port) & bit) != 0;
    io->gpio_external_drive[port] |= bit;
    if (pin_high)
        io->gpio_external[port] |= bit;
    else
        io->gpio_external[port] &= ~bit;
    const bool target_level = (kinetis_io_internal_pin_level_unfiltered(io, port) & bit) != 0;
    if ((io->port_dfer[port] & bit) != 0) {
        io->gpio_filter_age[port][pin] = 0;
        if (target_level)
            io->gpio_pending[port] |= bit;
        else
            io->gpio_pending[port] &= ~bit;
    } else
        kinetis_io_internal_commit_pin_level(io, port, pin, previous_level, target_level);
    return true;
}

bool kinetis_io_release_pin(KinetisIo* io, uint8_t port, uint8_t pin) {
    if (io == NULL || !kinetis_io_internal_pin_exists(io, port, pin))
        return false;
    const uint32_t bit = 1u << pin;
    const bool previous_level = (kinetis_io_internal_pin_level(io, port) & bit) != 0;
    io->gpio_external_drive[port] &= ~bit;
    const bool target_level = (kinetis_io_internal_pin_level_unfiltered(io, port) & bit) != 0;
    if ((io->port_dfer[port] & bit) != 0) {
        io->gpio_filter_age[port][pin] = 0;
        if (target_level)
            io->gpio_pending[port] |= bit;
        else
            io->gpio_pending[port] &= ~bit;
    } else
        kinetis_io_internal_commit_pin_level(io, port, pin, previous_level, target_level);
    return true;
}

uint32_t kinetis_io_pin_input(const KinetisIo* io, uint8_t port) {
    return io == NULL || port >= KINETIS_IO_PORT_COUNT ? 0
                                                       : kinetis_io_internal_pin_level(io, port);
}

bool kinetis_io_usb_token(KinetisIo* io, uint8_t endpoint, uint8_t token, bool is_transmit) {
    if (io == NULL || endpoint >= 16 || !kinetis_io_clock_enabled(io, KINETIS_PERIPHERAL_USB0) ||
        (io->usb[KINETIS_USB_CTL] & 1u) == 0)
        return false;
    io->usb[KINETIS_USB_STAT] = (uint8_t)((endpoint << 4) | (is_transmit ? 8u : 0));
    io->usb[KINETIS_USB_ISTAT] |= 1u << 3;
    kinetis_io_internal_emit(io, KINETIS_IO_EVENT_USB_TOKEN, endpoint, token, is_transmit);
    if ((io->usb[KINETIS_USB_INTEN] & (1u << 3)) != 0)
        kinetis_io_internal_emit(io, KINETIS_IO_EVENT_IRQ, 53u, 1u << 3, endpoint);
    return true;
}

bool kinetis_io_can_receive(KinetisIo* io, const KinetisCanFrame* frame) {
    if (io == NULL || frame == NULL || frame->length > 8 ||
        !kinetis_io_clock_enabled(io, KINETIS_PERIPHERAL_CAN0) ||
        (io->can[KINETIS_CAN_MCR / 4] & (1u << 31)) != 0)
        return false;
    const uint8_t maximum_mailbox = (uint8_t)(io->can[KINETIS_CAN_MCR / 4] & 0x7fu);
    for (uint8_t mailbox_index = 0u; mailbox_index <= maximum_mailbox && mailbox_index < 16;
         mailbox_index++) {
        const uint32_t offset = KINETIS_CAN_MB_BASE + (uint32_t)mailbox_index * 16u;
        const uint8_t mailbox_code = (uint8_t)(io->can[offset / 4u] >> 24) & 15u;
        if (mailbox_code != 4u)
            continue;
        const uint32_t configured_identifier = io->can[(offset + 4u) / 4u];
        const uint32_t identifier_mask = mailbox_index == 14   ? io->can[KINETIS_CAN_RX14MASK / 4]
                                         : mailbox_index == 15 ? io->can[KINETIS_CAN_RX15MASK / 4]
                                                               : io->can[KINETIS_CAN_RXMGMASK / 4];
        if (((configured_identifier ^ frame->identifier) & identifier_mask) != 0)
            continue;
        uint32_t control_status = (2u << 24) | ((uint32_t)frame->length << 16);
        if (frame->extended)
            control_status |= 1u << 21;
        if (frame->remote)
            control_status |= 1u << 20;
        io->can[offset / 4u] = control_status | (io->can[KINETIS_CAN_TIMER / 4] & 0xffffu);
        io->can[(offset + 4u) / 4u] = frame->identifier;
        uint32_t upper_word = 0;
        uint32_t lower_word = 0;
        for (uint8_t byte_index = 0; byte_index < 4; byte_index++) {
            upper_word = (upper_word << 8) | frame->data[byte_index];
            lower_word = (lower_word << 8) | frame->data[byte_index + 4u];
        }
        io->can[(offset + 8u) / 4u] = upper_word;
        io->can[(offset + 12u) / 4u] = lower_word;
        io->can[KINETIS_CAN_IFLAG1 / 4] |= 1u << mailbox_index;
        if ((io->can[KINETIS_CAN_IMASK1 / 4] & (1u << mailbox_index)) != 0)
            kinetis_io_internal_emit(io, KINETIS_IO_EVENT_IRQ, 75u, 1u << mailbox_index, 0);
        return true;
    }
    return false;
}

bool kinetis_io_i2s_receive(KinetisIo* io, uint32_t sample_value) {
    if (io == NULL || !kinetis_io_clock_enabled(io, KINETIS_PERIPHERAL_I2S0) ||
        (io->i2s[KINETIS_I2S_RCSR / 4] & KINETIS_I2S_ENABLE) == 0 ||
        !kinetis_io_internal_fifo_push(io->i2s_receive_fifo, &io->i2s_receive_write,
                                       &io->i2s_receive_count, sample_value))
        return false;
    kinetis_io_internal_update_i2s_requests(io);
    return true;
}

bool kinetis_io_i2s_transmit(KinetisIo* io, uint32_t* output_sample) {
    if (io == NULL || output_sample == NULL ||
        !kinetis_io_internal_fifo_pop(io->i2s_transmit_fifo, &io->i2s_transmit_read,
                                      &io->i2s_transmit_count, output_sample))
        return false;
    kinetis_io_internal_update_i2s_requests(io);
    return true;
}

static bool i2s_irq_asserted(uint32_t control) {
    return ((control & (1u << 8)) != 0 && (control & KINETIS_I2S_REQUEST_FLAG) != 0) ||
           ((control & (1u << 10)) != 0 && (control & KINETIS_I2S_FIFO_ERROR) != 0) ||
           ((control & (1u << 11)) != 0 && (control & (1u << 19)) != 0) ||
           ((control & (1u << 12)) != 0 && (control & (1u << 20)) != 0);
}

bool kinetis_io_irq_asserted(const KinetisIo* io, uint8_t irq) {
    if (io == NULL)
        return false;
    uint8_t first_port = 0u;
    uint8_t last_port = 0u;
    bool port_irq = false;
    if (io->configuration.profile->id == KINETIS_PROFILE_MKV10Z1287 && (irq == 30u || irq == 31u)) {
        first_port = irq == 30u ? 0u : 1u;
        last_port = irq == 30u ? 0u : 4u;
        port_irq = true;
    } else if (irq >= 59u && irq <= 63u) {
        first_port = last_port = (uint8_t)(irq - 59u);
        port_irq = true;
    }
    if (port_irq) {
        for (uint8_t port = first_port; port <= last_port; port++) {
            uint32_t pending = io->port_isfr[port];
            while (pending != 0) {
                const uint8_t pin = kinetis_io_internal_first_set_bit(pending);
                const uint32_t interrupt_config = (io->port_pcr[port][pin] >> 16) & 15u;
                if (interrupt_config >= 8u && interrupt_config <= 12u)
                    return true;
                pending &= ~(1u << pin);
            }
        }
        return false;
    }
    if (irq == 53u)
        return (io->usb[KINETIS_USB_ISTAT] & io->usb[KINETIS_USB_INTEN]) != 0;
    if (irq == 75u)
        return (io->can[KINETIS_CAN_IFLAG1 / 4u] & io->can[KINETIS_CAN_IMASK1 / 4u]) != 0;
    if (irq == 28u)
        return i2s_irq_asserted(io->i2s[KINETIS_I2S_TCSR / 4u]);
    if (irq == 29u)
        return i2s_irq_asserted(io->i2s[KINETIS_I2S_RCSR / 4u]);
    return false;
}

static bool sysmpu_permission(uint32_t access_control, uint8_t master, bool supervisor,
                              KinetisSysMpuAccess access) {
    if (master < 4u) {
        const uint8_t shift = master * 6u;
        const uint8_t user = (uint8_t)(access_control >> shift) & 7u;
        if (!supervisor)
            return (user & (1u << (2u - access))) != 0;
        const uint8_t mode = (uint8_t)(access_control >> (shift + 3u)) & 3u;
        if (mode == 0)
            return true;
        if (mode == 1u)
            return access != KINETIS_SYSMPU_WRITE;
        if (mode == 2u)
            return access != KINETIS_SYSMPU_EXECUTE;
        return (user & (1u << (2u - access))) != 0;
    }
    if (access == KINETIS_SYSMPU_EXECUTE)
        return false;
    const uint8_t shift = (uint8_t)(24u + (master - 4u) * 2u);
    const uint8_t bit = access == KINETIS_SYSMPU_WRITE ? 0 : 1;
    return (access_control & (1u << (shift + bit))) != 0;
}

bool kinetis_io_sysmpu_access(KinetisIo* io, uint32_t address, uint8_t master, bool supervisor,
                              KinetisSysMpuAccess access) {
    if (io == NULL || master >= 8u || access > KINETIS_SYSMPU_EXECUTE ||
        !kinetis_profile_has_peripheral(io->configuration.profile, KINETIS_PERIPHERAL_SYSMPU) ||
        !kinetis_io_clock_enabled(io, KINETIS_PERIPHERAL_SYSMPU))
        return false;
    if ((io->sysmpu[0] & 1u) == 0)
        return true;
    for (uint8_t region = 0; region < 12; region++) {
        const uint32_t descriptor = 0x400u / 4u + (uint32_t)region * 4u;
        if ((io->sysmpu[descriptor + 3u] & 1u) == 0 || address < io->sysmpu[descriptor] ||
            address > io->sysmpu[descriptor + 1u])
            continue;
        if (sysmpu_permission(io->sysmpu[descriptor + 2u], master, supervisor, access))
            return true;
    }
    io->sysmpu[0] |= 1u << 27;
    io->sysmpu[0x10u / 4u] = address;
    io->sysmpu[0x14u / 4u] = (uint32_t)master << 24 | (uint32_t)access << 20;
    kinetis_io_internal_emit(io, KINETIS_IO_EVENT_ACCESS_ERROR, address, master, access);
    return false;
}

void kinetis_io_advance(KinetisIo* io, uint32_t cycles) {
    if (io == NULL || cycles == 0)
        return;
    for (uint8_t port = 0; port < KINETIS_IO_PORT_COUNT; port++) {
        uint32_t filtered_pins = io->port_dfer[port] & io->configuration.package_pin_mask[port];
        while (filtered_pins != 0) {
            const uint8_t pin = kinetis_io_internal_first_set_bit(filtered_pins);
            const uint32_t bit = 1u << pin;
            const bool target_level = (io->gpio_pending[port] & bit) != 0;
            const bool current_level = (io->gpio_filtered[port] & bit) != 0;
            if (target_level != current_level) {
                uint64_t age = (uint64_t)io->gpio_filter_age[port][pin] + cycles;
                const uint32_t threshold = (uint32_t)io->port_dfwr[port] + 1u;
                if (age >= threshold) {
                    kinetis_io_internal_commit_pin_level(io, port, pin, current_level,
                                                         target_level);
                    age = 0;
                }
                io->gpio_filter_age[port][pin] = age > UINT8_MAX ? UINT8_MAX : (uint8_t)age;
            }
            filtered_pins &= ~bit;
        }
    }
    if (kinetis_io_clock_enabled(io, KINETIS_PERIPHERAL_USB0) &&
        (io->usb[KINETIS_USB_CTL] & 1u) != 0) {
        const uint64_t elapsed = (uint64_t)io->usb_cycle_remainder + cycles;
        const uint32_t frames = (uint32_t)(elapsed / 1000u);
        io->usb_cycle_remainder = (uint32_t)(elapsed % 1000u);
        if (frames != 0) {
            uint16_t frame = (uint16_t)io->usb[KINETIS_USB_FRMNUML] |
                             ((uint16_t)io->usb[KINETIS_USB_FRMNUMH] << 8);
            frame = (uint16_t)((frame + frames) & 0x7ffu);
            io->usb[KINETIS_USB_FRMNUML] = (uint8_t)frame;
            io->usb[KINETIS_USB_FRMNUMH] = (uint8_t)(frame >> 8);
            io->usb[KINETIS_USB_ISTAT] |= 1u << 2;
            if ((io->usb[KINETIS_USB_INTEN] & (1u << 2)) != 0)
                kinetis_io_internal_emit(io, KINETIS_IO_EVENT_IRQ, 53u, 1u << 2, frame);
        }
    }
    if (kinetis_io_clock_enabled(io, KINETIS_PERIPHERAL_CAN0) &&
        (io->can[KINETIS_CAN_MCR / 4] & ((1u << 31) | (1u << 28))) == 0)
        io->can[KINETIS_CAN_TIMER / 4] += cycles;
}
