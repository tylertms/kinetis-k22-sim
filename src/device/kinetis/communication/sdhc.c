#include "device/kinetis/communication/sdhc.h"

#include <stdlib.h>
#include <string.h>

enum {
    SDHC_BASE = 0x400b1000u,
    SDHC_DSADDR = 0x00u,
    SDHC_BLKATTR = 0x04u,
    SDHC_CMDARG = 0x08u,
    SDHC_XFERTYP = 0x0cu,
    SDHC_CMDRSP0 = 0x10u,
    SDHC_DATPORT = 0x20u,
    SDHC_PRSSTAT = 0x24u,
    SDHC_PROCTL = 0x28u,
    SDHC_SYSCTL = 0x2cu,
    SDHC_IRQSTAT = 0x30u,
    SDHC_IRQSTATEN = 0x34u,
    SDHC_IRQSIGEN = 0x38u,
    SDHC_AC12ERR = 0x3cu,
    SDHC_HTCAPBLT = 0x40u,
    SDHC_WML = 0x44u,
    SDHC_FEVT = 0x50u,
    SDHC_ADMAES = 0x54u,
    SDHC_ADSADDR = 0x58u,
    SDHC_VENDOR = 0xc0u,
    SDHC_MMCBOOT = 0xc4u,
    SDHC_HOSTVER = 0xfcu,
    SDHC_IRQ_COMMAND_COMPLETE = 1u << 0,
    SDHC_IRQ_TRANSFER_COMPLETE = 1u << 1,
    SDHC_IRQ_DMA = 1u << 3,
    SDHC_IRQ_BUFFER_WRITE_READY = 1u << 4,
    SDHC_IRQ_BUFFER_READ_READY = 1u << 5,
    SDHC_IRQ_CARD_INSERTION = 1u << 6,
    SDHC_IRQ_CARD_REMOVAL = 1u << 7,
    SDHC_IRQ_COMMAND_TIMEOUT = 1u << 16,
    SDHC_IRQ_DATA_TIMEOUT = 1u << 20,
    SDHC_IRQ_DATA_CRC = 1u << 21,
};

static uint32_t* register_pointer(KinetisSdhc* sdhc, uint32_t register_offset) {
    return &sdhc->registers[register_offset / 4u];
}

static const uint32_t* const_register_pointer(const KinetisSdhc* sdhc, uint32_t register_offset) {
    return &sdhc->registers[register_offset / 4u];
}

static void set_interrupt_status(KinetisSdhc* sdhc, uint32_t status_flags) {
    *register_pointer(sdhc, SDHC_IRQSTAT) |= status_flags & *register_pointer(sdhc, SDHC_IRQSTATEN);
}

static void refresh_present_state(KinetisSdhc* sdhc) {
    uint32_t present_state = 1u << 3;
    if (sdhc->present)
        present_state |= (1u << 16) | (1u << 23) | (0xffu << 24);
    if (sdhc->transfer_remaining != 0u) {
        present_state |= 1u << 2;
        present_state |= sdhc->transfer_read ? (1u << 9) | (1u << 11) : (1u << 8) | (1u << 10);
    }
    *register_pointer(sdhc, SDHC_PRSSTAT) = present_state;
}

static void reset_registers(KinetisSdhc* sdhc) {
    memset(sdhc->registers, 0, sizeof(sdhc->registers));
    *register_pointer(sdhc, SDHC_PRSSTAT) = 1u << 3;
    *register_pointer(sdhc, SDHC_HTCAPBLT) = 0x07f30000u;
    *register_pointer(sdhc, SDHC_WML) = 0x00800080u;
    *register_pointer(sdhc, SDHC_HOSTVER) = 0x00001201u;
}

bool kinetis_sdhc_init(KinetisSdhc* sdhc, KinetisSdhcBus bus) {
    if (sdhc == NULL || bus.read == NULL || bus.write == NULL)
        return false;
    memset(sdhc, 0, sizeof(*sdhc));
    sdhc->bus = bus;
    sdhc->block_length = 512u;
    sdhc->relative_address = 1u << 16;
    reset_registers(sdhc);
    return true;
}

void kinetis_sdhc_destroy(KinetisSdhc* sdhc) {
    if (sdhc == NULL)
        return;
    free(sdhc->card);
    sdhc->card = NULL;
    sdhc->card_size = 0u;
    sdhc->present = false;
}

void kinetis_sdhc_reset(KinetisSdhc* sdhc) {
    if (sdhc == NULL)
        return;
    const bool present = sdhc->present;
    const bool write_protected = sdhc->write_protected;
    sdhc->transfer_address = 0u;
    sdhc->transfer_remaining = 0u;
    sdhc->block_length = 512u;
    sdhc->relative_address = 1u << 16;
    sdhc->clock_enabled = false;
    sdhc->application_command = false;
    sdhc->selected = false;
    sdhc->transfer_read = false;
    sdhc->transfer_multiple = false;
    sdhc->present = present;
    sdhc->write_protected = write_protected;
    reset_registers(sdhc);
    refresh_present_state(sdhc);
}

bool kinetis_sdhc_copy(KinetisSdhc* destination, const KinetisSdhc* source, KinetisSdhcBus bus) {
    if (destination == NULL || source == NULL || bus.read == NULL || bus.write == NULL)
        return false;
    uint8_t* card_copy = NULL;
    if (source->card_size != 0u) {
        card_copy = malloc(source->card_size);
        if (card_copy == NULL)
            return false;
        memcpy(card_copy, source->card, source->card_size);
    }
    free(destination->card);
    *destination = *source;
    destination->bus = bus;
    destination->card = card_copy;
    return true;
}

void kinetis_sdhc_set_clock(KinetisSdhc* sdhc, bool enabled) {
    if (sdhc != NULL)
        sdhc->clock_enabled = enabled;
}

bool kinetis_sdhc_state_insert(KinetisSdhc* sdhc, const void* card_data, size_t card_size,
                               bool write_protected) {
    if (sdhc == NULL || card_data == NULL || card_size == 0u || card_size % 512u != 0u)
        return false;
    uint8_t* card = malloc(card_size);
    if (card == NULL)
        return false;
    memcpy(card, card_data, card_size);
    free(sdhc->card);
    sdhc->card = card;
    sdhc->card_size = card_size;
    sdhc->present = true;
    sdhc->write_protected = write_protected;
    sdhc->selected = false;
    set_interrupt_status(sdhc, SDHC_IRQ_CARD_INSERTION);
    refresh_present_state(sdhc);
    return true;
}

void kinetis_sdhc_state_eject(KinetisSdhc* sdhc) {
    if (sdhc == NULL)
        return;
    free(sdhc->card);
    sdhc->card = NULL;
    sdhc->card_size = 0u;
    sdhc->present = false;
    sdhc->selected = false;
    sdhc->transfer_remaining = 0u;
    set_interrupt_status(sdhc, SDHC_IRQ_CARD_REMOVAL);
    refresh_present_state(sdhc);
}

bool kinetis_sdhc_state_read_card(const KinetisSdhc* sdhc, size_t card_offset, void* card_data,
                                  size_t byte_count) {
    if (sdhc == NULL || card_data == NULL || !sdhc->present || card_offset > sdhc->card_size ||
        byte_count > sdhc->card_size - card_offset)
        return false;
    memcpy(card_data, sdhc->card + card_offset, byte_count);
    return true;
}

static uint32_t transfer_block_count(const KinetisSdhc* sdhc) {
    const uint32_t block_count = *const_register_pointer(sdhc, SDHC_BLKATTR) >> 16;
    return block_count == 0u ? 1u : block_count;
}

static uint32_t transfer_block_size(const KinetisSdhc* sdhc) {
    const uint32_t block_size = *const_register_pointer(sdhc, SDHC_BLKATTR) & 0x1fffu;
    return block_size == 0u ? sdhc->block_length : block_size;
}

static bool transfer_bounds(const KinetisSdhc* sdhc, size_t card_offset, size_t byte_count) {
    return card_offset <= sdhc->card_size && byte_count <= sdhc->card_size - card_offset;
}

static void complete_transfer(KinetisSdhc* sdhc) {
    sdhc->transfer_remaining = 0u;
    set_interrupt_status(sdhc, SDHC_IRQ_TRANSFER_COMPLETE);
    refresh_present_state(sdhc);
}

static bool perform_dma_transfer(KinetisSdhc* sdhc) {
    uint32_t bus_address = *register_pointer(sdhc, SDHC_DSADDR) & 0xfffffffcu;
    while (sdhc->transfer_remaining != 0u) {
        const uint8_t byte_count = sdhc->transfer_remaining >= 4u ? 4u : 1u;
        uint32_t transfer_value = 0u;
        if (sdhc->transfer_read) {
            memcpy(&transfer_value, sdhc->card + sdhc->transfer_address, byte_count);
            if (!sdhc->bus.write(sdhc->bus.context, bus_address, byte_count, transfer_value))
                return false;
        } else {
            if (!sdhc->bus.read(sdhc->bus.context, bus_address, byte_count, &transfer_value))
                return false;
            memcpy(sdhc->card + sdhc->transfer_address, &transfer_value, byte_count);
        }

        bus_address += byte_count;
        sdhc->transfer_address += byte_count;
        sdhc->transfer_remaining -= byte_count;
    }
    *register_pointer(sdhc, SDHC_DSADDR) = bus_address;
    set_interrupt_status(sdhc, SDHC_IRQ_DMA | SDHC_IRQ_TRANSFER_COMPLETE);
    refresh_present_state(sdhc);
    return true;
}

static bool begin_transfer(KinetisSdhc* sdhc, bool read_transfer, bool multiple_blocks) {
    const size_t transfer_size = transfer_block_size(sdhc);
    const size_t block_count = multiple_blocks ? transfer_block_count(sdhc) : 1u;
    const size_t card_offset = (size_t)*register_pointer(sdhc, SDHC_CMDARG) * 512u;
    if (!sdhc->selected || transfer_size == 0u || transfer_size > 4096u ||
        block_count > SIZE_MAX / transfer_size ||
        !transfer_bounds(sdhc, card_offset, transfer_size * block_count)) {
        set_interrupt_status(sdhc, SDHC_IRQ_DATA_TIMEOUT);
        return false;
    }
    if (!read_transfer && sdhc->write_protected) {
        set_interrupt_status(sdhc, SDHC_IRQ_DATA_CRC);
        return false;
    }
    sdhc->transfer_address = card_offset;
    sdhc->transfer_remaining = transfer_size * block_count;
    sdhc->transfer_read = read_transfer;
    sdhc->transfer_multiple = multiple_blocks;
    refresh_present_state(sdhc);
    if ((*register_pointer(sdhc, SDHC_XFERTYP) & 1u) != 0u) {
        if (!perform_dma_transfer(sdhc)) {
            set_interrupt_status(sdhc, SDHC_IRQ_DATA_TIMEOUT);
            return false;
        }
    } else {
        set_interrupt_status(sdhc, read_transfer ? SDHC_IRQ_BUFFER_READ_READY
                                                 : SDHC_IRQ_BUFFER_WRITE_READY);
    }
    return true;
}

static void issue_command(KinetisSdhc* sdhc) {
    const uint8_t command_index = (uint8_t)(*register_pointer(sdhc, SDHC_XFERTYP) >> 24u) & 0x3fu;
    const uint32_t argument = *register_pointer(sdhc, SDHC_CMDARG);
    memset(register_pointer(sdhc, SDHC_CMDRSP0), 0, 4u * sizeof(uint32_t));
    if (command_index != 0u && !sdhc->present) {
        set_interrupt_status(sdhc, SDHC_IRQ_COMMAND_TIMEOUT);
        return;
    }
    if (sdhc->application_command && command_index == 41u) {
        *register_pointer(sdhc, SDHC_CMDRSP0) = 0xc0ff8000u;
        sdhc->application_command = false;
    } else {
        sdhc->application_command = false;
        switch (command_index) {
        case 0u:
            sdhc->selected = false;
            break;
        case 2u:
            *register_pointer(sdhc, SDHC_CMDRSP0) = 0x03534453u;
            *register_pointer(sdhc, SDHC_CMDRSP0 + 4u) = 0x44303447u;
            *register_pointer(sdhc, SDHC_CMDRSP0 + 8u) = 0x80123456u;
            *register_pointer(sdhc, SDHC_CMDRSP0 + 12u) = 0x78010001u;
            break;
        case 3u:
            *register_pointer(sdhc, SDHC_CMDRSP0) = sdhc->relative_address;
            break;
        case 7u:
            sdhc->selected = argument == sdhc->relative_address;
            break;
        case 8u:
            *register_pointer(sdhc, SDHC_CMDRSP0) = argument;
            break;
        case 9u:
            *register_pointer(sdhc, SDHC_CMDRSP0) = 0x400e0032u;
            *register_pointer(sdhc, SDHC_CMDRSP0 + 4u) = (uint32_t)(sdhc->card_size / 512u);
            break;
        case 12u:
            complete_transfer(sdhc);
            break;
        case 13u:
            *register_pointer(sdhc, SDHC_CMDRSP0) = sdhc->selected ? 4u << 9u : 3u << 9u;
            break;
        case 16u:
            if (argument == 0u || argument > 4096u)
                set_interrupt_status(sdhc, SDHC_IRQ_COMMAND_TIMEOUT);
            else
                sdhc->block_length = argument;
            break;
        case 17u:
            (void)begin_transfer(sdhc, true, false);
            break;
        case 18u:
            (void)begin_transfer(sdhc, true, true);
            break;
        case 24u:
            (void)begin_transfer(sdhc, false, false);
            break;
        case 25u:
            (void)begin_transfer(sdhc, false, true);
            break;
        case 55u:
            sdhc->application_command = true;
            *register_pointer(sdhc, SDHC_CMDRSP0) = 1u << 5u;
            break;
        default:
            set_interrupt_status(sdhc, SDHC_IRQ_COMMAND_TIMEOUT);
            break;
        }
    }
    set_interrupt_status(sdhc, SDHC_IRQ_COMMAND_COMPLETE);
}

static bool read_data(KinetisSdhc* sdhc, uint32_t* read_value) {
    if (!sdhc->transfer_read || sdhc->transfer_remaining == 0u)
        return false;
    const size_t byte_count = sdhc->transfer_remaining >= 4u ? 4u : sdhc->transfer_remaining;
    *read_value = 0u;
    memcpy(read_value, sdhc->card + sdhc->transfer_address, byte_count);
    sdhc->transfer_address += byte_count;
    sdhc->transfer_remaining -= byte_count;
    if (sdhc->transfer_remaining == 0u)
        complete_transfer(sdhc);
    return true;
}

static bool write_data(KinetisSdhc* sdhc, uint32_t write_value) {
    if (sdhc->transfer_read || sdhc->transfer_remaining == 0u)
        return false;
    const size_t byte_count = sdhc->transfer_remaining >= 4u ? 4u : sdhc->transfer_remaining;
    memcpy(sdhc->card + sdhc->transfer_address, &write_value, byte_count);
    sdhc->transfer_address += byte_count;
    sdhc->transfer_remaining -= byte_count;
    if (sdhc->transfer_remaining == 0u)
        complete_transfer(sdhc);
    return true;
}

static bool readable(uint32_t register_offset) {
    return register_offset <= SDHC_WML || register_offset == SDHC_ADMAES ||
           register_offset == SDHC_ADSADDR || register_offset == SDHC_VENDOR ||
           register_offset == SDHC_MMCBOOT || register_offset == SDHC_HOSTVER;
}

static bool writable(uint32_t register_offset) {
    return register_offset == SDHC_DSADDR || register_offset == SDHC_BLKATTR ||
           register_offset == SDHC_CMDARG || register_offset == SDHC_XFERTYP ||
           register_offset == SDHC_DATPORT || register_offset == SDHC_PROCTL ||
           register_offset == SDHC_SYSCTL || register_offset == SDHC_IRQSTAT ||
           register_offset == SDHC_IRQSTATEN || register_offset == SDHC_IRQSIGEN ||
           register_offset == SDHC_FEVT || register_offset == SDHC_ADSADDR ||
           register_offset == SDHC_VENDOR || register_offset == SDHC_MMCBOOT;
}

bool kinetis_sdhc_read(KinetisSdhc* sdhc, uint32_t register_address, uint8_t access_size,
                       uint32_t* read_value) {
    if (sdhc == NULL || read_value == NULL || !sdhc->clock_enabled || access_size != 4u ||
        register_address < SDHC_BASE || register_address - SDHC_BASE > SDHC_HOSTVER ||
        ((register_address - SDHC_BASE) & 3u) != 0u)
        return false;
    const uint32_t register_offset = register_address - SDHC_BASE;
    if (register_offset == SDHC_DATPORT)
        return read_data(sdhc, read_value);
    if (!readable(register_offset))
        return false;
    refresh_present_state(sdhc);
    *read_value = *register_pointer(sdhc, register_offset);
    return true;
}

bool kinetis_sdhc_write(KinetisSdhc* sdhc, uint32_t register_address, uint8_t access_size,
                        uint32_t write_value) {
    if (sdhc == NULL || !sdhc->clock_enabled || access_size != 4u || register_address < SDHC_BASE ||
        register_address - SDHC_BASE > SDHC_HOSTVER || ((register_address - SDHC_BASE) & 3u) != 0u)
        return false;
    const uint32_t register_offset = register_address - SDHC_BASE;
    if (!writable(register_offset))
        return false;
    if (register_offset == SDHC_DATPORT)
        return write_data(sdhc, write_value);
    if (register_offset == SDHC_IRQSTAT) {
        *register_pointer(sdhc, register_offset) &= ~write_value;
        return true;
    }
    if (register_offset == SDHC_FEVT) {
        set_interrupt_status(sdhc, write_value);
        return true;
    }
    if (register_offset == SDHC_SYSCTL && (write_value & 0x07000000u) != 0u) {
        const bool clock_enabled = sdhc->clock_enabled;
        kinetis_sdhc_reset(sdhc);
        sdhc->clock_enabled = clock_enabled;
        *register_pointer(sdhc, register_offset) = write_value & 0x000fffffu;
        return true;
    }
    *register_pointer(sdhc, register_offset) = write_value;
    if (register_offset == SDHC_XFERTYP)
        issue_command(sdhc);
    return true;
}

bool kinetis_sdhc_irq(const KinetisSdhc* sdhc) {
    if (sdhc == NULL || !sdhc->clock_enabled)
        return false;
    return (*const_register_pointer(sdhc, SDHC_IRQSTAT) &
            *const_register_pointer(sdhc, SDHC_IRQSIGEN)) != 0u;
}
