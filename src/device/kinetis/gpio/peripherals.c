#include "device/kinetis/gpio/internal.h"

static bool usb_offset_valid(uint32_t offset) {
    if (offset <= 0x1cu)
        return (offset & 3u) == 0;
    if (offset >= 0x80u && offset <= 0xbcu)
        return (offset & 3u) == 0;
    if (offset >= 0xc0u && offset <= 0xfcu)
        return (offset & 3u) == 0;
    return offset == 0x100u || offset == 0x104u || offset == 0x108u || offset == 0x10cu ||
           offset == 0x110u || offset == 0x114u || offset == 0x140u || offset == 0x144u ||
           offset == 0x148u || offset == 0x14cu || offset == 0x154u || offset == 0x158u ||
           offset == 0x15cu;
}

static bool read_usb(KinetisIo* io, KinetisPeripheralLocation location, uint8_t size,
                     uint32_t* output_value) {
    if (size != 1 || !usb_offset_valid(location.offset))
        return false;
    *output_value = io->usb[location.offset];
    return true;
}

static bool write_usb(KinetisIo* io, KinetisPeripheralLocation location, uint8_t size,
                      uint32_t write_value) {
    if (size != 1 || !usb_offset_valid(location.offset) || location.offset <= 0x0cu ||
        location.offset == KINETIS_USB_STAT || location.offset == KINETIS_USB_FRMNUML ||
        location.offset == KINETIS_USB_FRMNUMH)
        return false;
    if (location.offset == 0x10u || location.offset == KINETIS_USB_ISTAT ||
        location.offset == 0x88u)
        io->usb[location.offset] &= (uint8_t)~write_value;
    else if (location.offset == 0xd0u && (write_value & 0x80u) != 0) {
        const uint8_t ids[4] = {io->usb[0], io->usb[4], io->usb[8], io->usb[0x0c]};
        memset(io->usb, 0, sizeof(io->usb));
        io->usb[0] = ids[0];
        io->usb[4] = ids[1];
        io->usb[8] = ids[2];
        io->usb[0x0c] = ids[3];
    } else
        io->usb[location.offset] = (uint8_t)write_value;
    return true;
}

static bool read_words(const uint32_t* words, uint32_t word_count, uint32_t offset, uint8_t size,
                       uint32_t* output_value) {
    if (size != 4 || (offset & 3u) != 0 || offset >= word_count)
        return false;
    *output_value = words[offset / 4u];
    return true;
}

static bool can_offset_valid(uint32_t offset) {
    switch (offset) {
    case 0:
    case 4:
    case 8:
    case 0x10:
    case 0x14:
    case 0x18:
    case 0x1c:
    case 0x20:
    case 0x24:
    case 0x28:
    case 0x2c:
    case 0x30:
    case 0x34:
    case 0x38:
    case 0x44:
    case 0x48:
    case 0x4c:
        return true;
    default:
        return (offset >= 0x80u && offset < 0x180u) || (offset >= 0x880u && offset < 0x8c0u);
    }
}

static bool write_can(KinetisIo* io, uint32_t offset, uint8_t size, uint32_t write_value) {
    if (size != 4 || (offset & 3u) != 0 || !can_offset_valid(offset))
        return false;
    if (offset == KINETIS_CAN_TIMER || offset == 0x1cu || offset == 0x38u || offset == 0x44u ||
        offset == 0x4cu)
        return false;
    if (offset == KINETIS_CAN_IFLAG1) {
        io->can[offset / 4u] &= ~write_value;
        return true;
    }
    if (offset == KINETIS_CAN_MCR && (write_value & (1u << 25)) != 0) {
        const bool clock = io->clock_enabled[KINETIS_PERIPHERAL_CAN0];
        memset(io->can, 0, sizeof(io->can));
        io->can[KINETIS_CAN_MCR / 4] = 0xd890000fu;
        io->can[KINETIS_CAN_RXMGMASK / 4] = UINT32_MAX;
        io->can[KINETIS_CAN_RX14MASK / 4] = UINT32_MAX;
        io->can[KINETIS_CAN_RX15MASK / 4] = UINT32_MAX;
        io->clock_enabled[KINETIS_PERIPHERAL_CAN0] = clock;
        return true;
    }
    io->can[offset / 4u] = write_value;
    if (offset >= KINETIS_CAN_MB_BASE && offset < KINETIS_CAN_MB_BASE + 16u * 16u &&
        (offset & 15u) == 0) {
        const uint8_t mailbox_index = (uint8_t)((offset - KINETIS_CAN_MB_BASE) / 16u);
        const uint8_t mailbox_code = (uint8_t)(write_value >> 24) & 15u;
        if (mailbox_code >= 0xcu && mailbox_code <= 0xfu) {
            const uint32_t identifier = io->can[(offset + 4u) / 4u];
            kinetis_io_internal_emit_can(io, mailbox_index, identifier, write_value,
                                         io->can[(offset + 8u) / 4u], io->can[(offset + 12u) / 4u]);
            io->can[offset / 4u] = (write_value & ~(15u << 24)) | (8u << 24);
            io->can[KINETIS_CAN_IFLAG1 / 4] |= 1u << mailbox_index;
            if ((io->can[KINETIS_CAN_IMASK1 / 4] & (1u << mailbox_index)) != 0)
                kinetis_io_internal_emit(io, KINETIS_IO_EVENT_IRQ, 75u, 1u << mailbox_index, 0);
        }
    }
    return true;
}

bool kinetis_io_internal_fifo_push(uint32_t* fifo, uint8_t* write_index, uint8_t* entry_count,
                                   uint32_t entry_value) {
    if (*entry_count == KINETIS_IO_FIFO_CAPACITY)
        return false;
    fifo[*write_index] = entry_value;
    *write_index = (uint8_t)((*write_index + 1u) % KINETIS_IO_FIFO_CAPACITY);
    (*entry_count)++;
    return true;
}

bool kinetis_io_internal_fifo_pop(uint32_t* fifo, uint8_t* read_index, uint8_t* entry_count,
                                  uint32_t* output_value) {
    if (*entry_count == 0)
        return false;
    *output_value = fifo[*read_index];
    *read_index = (uint8_t)((*read_index + 1u) % KINETIS_IO_FIFO_CAPACITY);
    (*entry_count)--;
    return true;
}

void kinetis_io_internal_update_i2s_requests(KinetisIo* io) {
    uint32_t* transmit = &io->i2s[KINETIS_I2S_TCSR / 4];
    uint32_t* receive = &io->i2s[KINETIS_I2S_RCSR / 4];
    if (io->i2s_transmit_count < KINETIS_IO_FIFO_CAPACITY)
        *transmit |= KINETIS_I2S_REQUEST_FLAG;
    else
        *transmit &= ~KINETIS_I2S_REQUEST_FLAG;
    if (io->i2s_receive_count != 0)
        *receive |= KINETIS_I2S_REQUEST_FLAG;
    else
        *receive &= ~KINETIS_I2S_REQUEST_FLAG;
    if ((*transmit & KINETIS_I2S_REQUEST_FLAG) != 0) {
        if ((*transmit & 1u) != 0)
            kinetis_io_internal_emit(io, KINETIS_IO_EVENT_DMA, 0, io->i2s_transmit_count, 0);
        if ((*transmit & (1u << 8)) != 0)
            kinetis_io_internal_emit(io, KINETIS_IO_EVENT_IRQ, 28u, KINETIS_I2S_REQUEST_FLAG, 0);
    }
    if ((*receive & KINETIS_I2S_REQUEST_FLAG) != 0) {
        if ((*receive & 1u) != 0)
            kinetis_io_internal_emit(io, KINETIS_IO_EVENT_DMA, 1, io->i2s_receive_count, 0);
        if ((*receive & (1u << 8)) != 0)
            kinetis_io_internal_emit(io, KINETIS_IO_EVENT_IRQ, 29u, KINETIS_I2S_REQUEST_FLAG, 0);
    }
}

static bool i2s_offset_valid(uint32_t offset) {
    return offset <= 0x14u || offset == 0x20u || offset == 0x24u || offset == 0x40u ||
           offset == 0x44u || offset == 0x60u || (offset >= 0x80u && offset <= 0x94u) ||
           offset == 0xa0u || offset == 0xa4u || offset == 0xc0u || offset == 0xc4u ||
           offset == 0xe0u || offset == 0x100u || offset == 0x104u;
}

static bool read_i2s(KinetisIo* io, uint32_t offset, uint8_t size, uint32_t* output_value) {
    if (size != 4 || (offset & 3u) != 0 || !i2s_offset_valid(offset))
        return false;
    if (offset >= KINETIS_I2S_RDR0 && offset < KINETIS_I2S_RDR0 + 8u) {
        if (!kinetis_io_internal_fifo_pop(io->i2s_receive_fifo, &io->i2s_receive_read,
                                          &io->i2s_receive_count, output_value)) {
            io->i2s[KINETIS_I2S_RCSR / 4] |= KINETIS_I2S_FIFO_ERROR;
            *output_value = 0;
        }
        kinetis_io_internal_update_i2s_requests(io);
        return true;
    }
    *output_value = io->i2s[offset / 4u];
    return true;
}

static bool write_i2s(KinetisIo* io, uint32_t offset, uint8_t size, uint32_t write_value) {
    if (size != 4 || (offset & 3u) != 0 || !i2s_offset_valid(offset) || offset == 0x40u ||
        offset == 0x44u || offset == 0xc0u || offset == 0xc4u)
        return false;
    if (offset >= KINETIS_I2S_TDR0 && offset < KINETIS_I2S_TDR0 + 8u) {
        if (!kinetis_io_internal_fifo_push(io->i2s_transmit_fifo, &io->i2s_transmit_write,
                                           &io->i2s_transmit_count, write_value)) {
            io->i2s[KINETIS_I2S_TCSR / 4] |= KINETIS_I2S_FIFO_ERROR;
            return true;
        }
        kinetis_io_internal_emit(io, KINETIS_IO_EVENT_I2S_TRANSMIT,
                                 (offset - KINETIS_I2S_TDR0) / 4u, write_value,
                                 io->i2s_transmit_count);
        kinetis_io_internal_update_i2s_requests(io);
        return true;
    }
    if (offset == KINETIS_I2S_TCSR || offset == KINETIS_I2S_RCSR) {
        const uint32_t flags = KINETIS_I2S_REQUEST_FLAG | KINETIS_I2S_FIFO_ERROR | (1u << 17) |
                               (1u << 19) | (1u << 20);
        uint32_t previous_status = io->i2s[offset / 4u];
        previous_status &= ~(write_value & flags);
        io->i2s[offset / 4u] = (write_value & ~flags) | (previous_status & flags);
        if ((write_value & KINETIS_I2S_ENABLE) == 0) {
            if (offset == KINETIS_I2S_TCSR)
                io->i2s_transmit_count = io->i2s_transmit_read = io->i2s_transmit_write = 0;
            else
                io->i2s_receive_count = io->i2s_receive_read = io->i2s_receive_write = 0;
        }
        kinetis_io_internal_update_i2s_requests(io);
        return true;
    }
    io->i2s[offset / 4u] = write_value;
    return true;
}

static bool read_flash_configuration(KinetisIo* io, KinetisPeripheralLocation location,
                                     uint8_t size, uint32_t* output_value) {
    if (location.offset + size > sizeof(io->configuration.flash_configuration))
        return false;
    *output_value = kinetis_io_internal_load_bytes(
        io->configuration.flash_configuration + location.offset, size);
    return true;
}

static bool flexbus_offset_valid(uint32_t offset) {
    return (offset < 0x48u && (offset % 12u) <= 8u) || offset == 0x60u;
}

bool kinetis_io_flexbus_transfer(KinetisIo* io, uint32_t address, uint8_t access_size,
                                 bool is_write_access, uint32_t write_value) {
    if (io == NULL || !kinetis_io_clock_enabled(io, KINETIS_PERIPHERAL_FB) ||
        (access_size != 1u && access_size != 2u && access_size != 4u))
        return false;
    for (uint8_t chip_select = 0u; chip_select < 6u; chip_select++) {
        const uint32_t base = io->flexbus[chip_select * 3u] & 0xffff0000u;
        const uint32_t block = io->flexbus[chip_select * 3u + 1u];
        if ((block & 1u) == 0u)
            continue;
        const uint32_t comparison = ~(block & 0xffff0000u) & 0xffff0000u;
        if ((address & comparison) != (base & comparison))
            continue;
        kinetis_io_internal_emit(io, KINETIS_IO_EVENT_FLEXBUS_TRANSFER, chip_select, write_value,
                                 (uint32_t)access_size | (is_write_access ? 0x100u : 0u));
        return true;
    }
    return false;
}
static bool mcm_offset_valid(const KinetisIo* io, uint32_t offset) {
    if (io->configuration.profile->id == KINETIS_PROFILE_MKV10Z1287)
        return offset == 8u || offset == 0x0au || offset == 0x0cu || offset == 0x40u;
    if (offset == 0 || offset == 2u || offset == 4u || offset == 8u)
        return true;
    const bool large_profile = io->configuration.profile->id == KINETIS_PROFILE_MK22FN1M012 ||
                               io->configuration.profile->id == KINETIS_PROFILE_MK22FX51212;
    if (large_profile)
        return offset == 0x0cu || offset == 0x10u || offset == 0x14u || offset == 0x28u;
    return offset == 0x38u;
}

static bool sysmpu_offset_valid(uint32_t offset) {
    if (offset == 0)
        return true;
    if (offset >= 0x10u && offset <= 0x54u)
        return (offset & 0x0bu) == 0;
    return (offset >= 0x400u && offset < 0x4c0u) || (offset >= 0x800u && offset < 0x830u);
}

bool kinetis_io_internal_read_direct(KinetisIo* io, uint32_t address, uint8_t size,
                                     uint32_t* output_value) {
    KinetisPeripheralLocation location;
    if (!kinetis_profile_resolve_peripheral(io->configuration.profile, address, size, &location))
        return false;
    if (!kinetis_io_internal_module_clocked(io, location.id)) {
        kinetis_io_internal_emit(io, KINETIS_IO_EVENT_ACCESS_ERROR, address, size, 0);
        return false;
    }
    if (location.id == KINETIS_PERIPHERAL_FLASH_CONFIG)
        return read_flash_configuration(io, location, size, output_value);
    if (kinetis_io_internal_is_port(location.id))
        return kinetis_io_internal_read_port(io, location, size, output_value);
    if (kinetis_io_internal_is_gpio(location.id))
        return kinetis_io_internal_read_gpio(io, location, size, output_value);
    if (location.id == KINETIS_PERIPHERAL_USB0)
        return read_usb(io, location, size, output_value);
    if (location.id == KINETIS_PERIPHERAL_CAN0)
        return can_offset_valid(location.offset) &&
               read_words(io->can, sizeof(io->can), location.offset, size, output_value);
    if (location.id == KINETIS_PERIPHERAL_I2S0)
        return read_i2s(io, location.offset, size, output_value);
    if (location.id == KINETIS_PERIPHERAL_FB)
        return flexbus_offset_valid(location.offset) &&
               read_words(io->flexbus, sizeof(io->flexbus), location.offset, size, output_value);
    if (location.id == KINETIS_PERIPHERAL_MCM) {
        if (!mcm_offset_valid(io, location.offset))
            return false;
        if (size == 2u && (location.offset & 1u) == 0u) {
            const uint8_t* bytes = (const uint8_t*)io->mcm;
            *output_value = (uint32_t)bytes[location.offset] |
                            (uint32_t)bytes[location.offset + 1u] << 8u;
            return true;
        }
        if (location.offset == 0 && size == 4) {
            *output_value = io->mcm[0];
            return true;
        }
        if (location.offset == 0 || location.offset == 2u) {
            if (size != 2)
                return false;
            *output_value = (io->mcm[0] >> (location.offset * 8u)) & 0xffffu;
            return true;
        }
        return read_words(io->mcm, sizeof(io->mcm), location.offset, size, output_value);
    }
    if (location.id == KINETIS_PERIPHERAL_SYSMPU) {
        if (!sysmpu_offset_valid(location.offset))
            return false;
        if (size == 4 && location.offset >= 0x800u && location.offset < 0x830u) {
            const uint32_t region = (location.offset - 0x800u) / 4u;
            *output_value = io->sysmpu[0x400u / 4u + region * 4u + 2u];
            return true;
        }
        return read_words(io->sysmpu, sizeof(io->sysmpu), location.offset, size, output_value);
    }
    return false;
}

bool kinetis_io_internal_write_direct(KinetisIo* io, uint32_t address, uint8_t size,
                                      uint32_t write_value) {
    KinetisPeripheralLocation location;
    if (!kinetis_profile_resolve_peripheral(io->configuration.profile, address, size, &location))
        return false;
    if (!kinetis_io_internal_module_clocked(io, location.id)) {
        kinetis_io_internal_emit(io, KINETIS_IO_EVENT_ACCESS_ERROR, address, write_value, size);
        return false;
    }
    if (location.id == KINETIS_PERIPHERAL_FLASH_CONFIG)
        return false;
    if (kinetis_io_internal_is_port(location.id))
        return kinetis_io_internal_write_port(io, location, size, write_value);
    if (kinetis_io_internal_is_gpio(location.id))
        return kinetis_io_internal_write_gpio(io, location, size, write_value);
    if (location.id == KINETIS_PERIPHERAL_USB0)
        return write_usb(io, location, size, write_value);
    if (location.id == KINETIS_PERIPHERAL_CAN0)
        return write_can(io, location.offset, size, write_value);
    if (location.id == KINETIS_PERIPHERAL_I2S0)
        return write_i2s(io, location.offset, size, write_value);
    if (location.id == KINETIS_PERIPHERAL_FB) {
        if (size != 4 || (location.offset & 3u) != 0 || !flexbus_offset_valid(location.offset))
            return false;
        io->flexbus[location.offset / 4u] = write_value;
        if (location.offset < 0x48u && (location.offset % 12u) == 4u && (write_value & 1u) != 0)
            kinetis_io_internal_emit(io, KINETIS_IO_EVENT_FLEXBUS_TRANSFER, location.offset / 12u,
                                     write_value, 0);
        return true;
    }
    if (location.id == KINETIS_PERIPHERAL_MCM) {
        if (size != 4 || (location.offset & 3u) != 0 || location.offset == 0 ||
            !mcm_offset_valid(io, location.offset) || location.offset == 0x14u)
            return false;
        if (location.offset == 0x28u)
            write_value &= 0xffu;
        io->mcm[location.offset / 4u] = write_value;
        return true;
    }
    if (location.id == KINETIS_PERIPHERAL_SYSMPU) {
        if (size != 4 || (location.offset & 3u) != 0 || !sysmpu_offset_valid(location.offset))
            return false;
        if (location.offset >= 0x10u && location.offset < 0x70u)
            return false;
        if (location.offset == 0) {
            const uint32_t fixed = io->sysmpu[0] & 0x00ffff00u;
            const uint32_t errors = io->sysmpu[0] & 0xf8000000u & ~write_value;
            io->sysmpu[0] = fixed | errors | (write_value & 1u);
        } else if (location.offset >= 0x800u && location.offset < 0x830u) {
            const uint32_t region = (location.offset - 0x800u) / 4u;
            io->sysmpu[0x400u / 4u + region * 4u + 2u] = write_value;
            io->sysmpu[location.offset / 4u] = write_value;
        } else {
            io->sysmpu[location.offset / 4u] = write_value;
            if (location.offset >= 0x408u && location.offset < 0x4c0u &&
                (location.offset - 0x408u) % 16u == 0) {
                const uint32_t region = (location.offset - 0x408u) / 16u;
                io->sysmpu[0x800u / 4u + region] = write_value;
            }
        }
        return true;
    }
    return false;
}
