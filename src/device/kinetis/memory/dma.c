#include "internal.h"

static uint8_t dma_transfer_size(uint8_t transfer_width_code) {
    if (transfer_width_code <= 2u)
        return (uint8_t)(1u << transfer_width_code);
    return 0u;
}

uint32_t k22_data_internal_dma_priority_offset(uint8_t channel) {
    return 0x100u + (channel & 0xfcu) + 3u - (channel & 3u);
}

static uint8_t dma_priority_channel(uint32_t priority_offset) {
    const uint8_t channel_offset = (uint8_t)(priority_offset - 0x100u);
    return (uint8_t)((channel_offset & 0xfcu) + 3u - (channel_offset & 3u));
}

bool k22_data_internal_dma_priorities_valid(const K22Data* data) {
    uint16_t used_priorities = 0u;
    for (uint8_t channel = 0u; channel < data->dma_channel_count; channel++) {
        const uint8_t priority = data->dma[k22_data_internal_dma_priority_offset(channel)] & 15u;
        if ((used_priorities & (1u << priority)) != 0u)
            return false;
        used_priorities |= (uint16_t)(1u << priority);
    }
    return true;
}

uint8_t k22_data_internal_dma_select_channel(const K22Data* data) {
    if ((k22_data_internal_load_bytes(data->dma, 0u, 4u) & 4u) != 0u) {
        for (uint8_t channel_step = 1u; channel_step <= data->dma_channel_count; channel_step++) {
            const uint8_t channel =
                (uint8_t)((data->dma_last_channel + channel_step) % data->dma_channel_count);
            if ((data->dma_requests & (1u << channel)) != 0u)
                return channel;
        }
    } else {
        uint8_t selected_channel = UINT8_MAX;
        uint8_t selected_priority = 0u;
        for (uint8_t channel = 0u; channel < data->dma_channel_count; channel++) {
            if ((data->dma_requests & (1u << channel)) == 0u)
                continue;
            const uint8_t priority =
                data->dma[k22_data_internal_dma_priority_offset(channel)] & 15u;
            if (selected_channel == UINT8_MAX || priority > selected_priority) {
                selected_channel = channel;
                selected_priority = priority;
            }
        }
        return selected_channel;
    }
    return UINT8_MAX;
}

static uint16_t dma_iteration_count(uint16_t encoded_count) {
    return (encoded_count & 0x8000u) != 0u ? encoded_count & 0x01ffu : encoded_count & 0x7fffu;
}

static uint8_t dma_link_channel(uint16_t encoded_control) {
    return (uint8_t)((encoded_control >> 9) & 15u);
}

static uint32_t dma_advance_address(uint32_t address, int16_t address_delta, uint8_t modulo_bits) {
    const uint32_t advanced_address = (uint32_t)((int64_t)address + address_delta);
    if (modulo_bits == 0u || modulo_bits >= 31u)
        return advanced_address;
    const uint32_t modulo_mask = (1u << modulo_bits) - 1u;
    return (address & ~modulo_mask) | (advanced_address & modulo_mask);
}

static void dma_set_iteration_count(uint8_t* descriptor, uint32_t descriptor_offset,
                                    uint16_t iteration_count) {
    uint16_t encoded_count =
        (uint16_t)k22_data_internal_load_bytes(descriptor, descriptor_offset, 2u);
    const uint16_t iteration_mask = (encoded_count & 0x8000u) != 0u ? 0x01ffu : 0x7fffu;
    encoded_count =
        (uint16_t)((encoded_count & ~iteration_mask) | (iteration_count & iteration_mask));
    k22_data_internal_store_bytes(descriptor, descriptor_offset, 2u, encoded_count);
}

void k22_data_internal_dma_update_interrupts(K22Data* data) {
    const uint16_t pending = (uint16_t)k22_data_internal_load_bytes(data->dma, 0x24, 2);
    for (uint8_t channel = 0; channel < DMA_CHANNEL_COUNT; channel++)
        k22_data_internal_interrupt(data, (K22DataInterrupt)(K22_DATA_INTERRUPT_DMA0 + channel),
                                    (pending & (1u << channel)) != 0);
    const uint16_t errors = (uint16_t)k22_data_internal_load_bytes(data->dma, 0x2c, 2);
    const uint16_t enabled = (uint16_t)k22_data_internal_load_bytes(data->dma, 0x14, 2);
    k22_data_internal_interrupt(data, K22_DATA_INTERRUPT_DMA_ERROR, (errors & enabled) != 0);
}

void k22_data_internal_dma_error(K22Data* data, uint8_t channel, uint32_t error_reason) {
    uint32_t error_status = k22_data_internal_load_bytes(data->dma, 0x04, 4u);
    error_status |= 0x80000000u | ((uint32_t)channel << 8u) | error_reason;
    k22_data_internal_store_bytes(data->dma, 0x04, 4u, error_status);
    uint16_t error_flags = (uint16_t)k22_data_internal_load_bytes(data->dma, 0x2cu, 2u);
    error_flags |= (uint16_t)(1u << channel);
    k22_data_internal_store_bytes(data->dma, 0x2cu, 2u, error_flags);
    if ((k22_data_internal_load_bytes(data->dma, 0x14, 2) & (1u << channel)) != 0)
        k22_data_internal_interrupt(data, K22_DATA_INTERRUPT_DMA_ERROR, true);
    if ((k22_data_internal_load_bytes(data->dma, 0u, 4u) & 0x10u) != 0u)
        k22_data_internal_store_bytes(data->dma, 0u, 4u,
                                      k22_data_internal_load_bytes(data->dma, 0u, 4u) | 0x20u);
}

static bool dma_bus_read(K22Data* data, uint32_t address, uint8_t byte_count,
                         uint32_t* output_value) {
    if (data->bus.read == NULL)
        return false;
    return data->bus.read(data->bus.context, address, byte_count, output_value);
}

static bool dma_bus_write(K22Data* data, uint32_t address, uint8_t byte_count,
                          uint32_t write_value) {
    if (data->bus.write == NULL)
        return false;
    return data->bus.write(data->bus.context, address, byte_count, write_value);
}

static void dma_queue_channel(K22Data* data, uint8_t channel) {
    if (channel < data->dma_channel_count)
        data->dma_requests |= (uint16_t)(1u << channel);
}

bool k22_data_internal_dma_source_always_enabled(const K22Data* data, uint8_t request_source) {
    const bool has_extended_source_map = data->profile->id == K22_PROFILE_MK22FN1M012 ||
                                         data->profile->id == K22_PROFILE_MK22FX51212;
    return request_source >= (has_extended_source_map ? 54u : 60u);
}

static uint64_t dma_source_mask(const K22Data* data) {
    if (data->profile->id == K22_PROFILE_MKV30F12810)
        return UINT64_C(0xf03f2f00f3f4c03c);
    if (data->profile->id == K22_PROFILE_MK22FN1M012 ||
        data->profile->id == K22_PROFILE_MK22FX51212)
        return UINT64_C(0xfffffffffffffffc);
    if (data->profile->id == K22_PROFILE_MK22FN51212)
        return UINT64_C(0xfc3f6ffffffdf0fc);
    return UINT64_C(0xfc3f2f00fffdf0fc);
}

bool k22_data_internal_dma_source_valid(const K22Data* data, uint8_t request_source) {
    return request_source < 64u && (dma_source_mask(data) & (UINT64_C(1) << request_source)) != 0u;
}

void k22_data_internal_dma_queue_hardware_channel(K22Data* data, uint8_t channel, uint8_t source) {
    dma_queue_channel(data, channel);
    data->dma_hardware_requests |= (uint16_t)(1u << channel);
    data->dma_request_source[channel] = source;
}

void k22_data_internal_dma_queue_always_enabled(K22Data* data, uint8_t channel) {
    if (channel >= data->dma_channel_count)
        return;
    const uint8_t channel_configuration = data->dmamux[channel];
    const uint8_t request_source = channel_configuration & 0x3fu;
    const uint16_t enabled_channels = (uint16_t)k22_data_internal_load_bytes(data->dma, 0x0c, 2);
    if ((channel_configuration & 0xc0u) == 0x80u && (enabled_channels & (1u << channel)) != 0u &&
        k22_data_internal_dma_source_always_enabled(data, request_source))
        k22_data_internal_dma_queue_hardware_channel(data, channel, request_source);
}

static bool dma_copy_descriptor(K22Data* data, uint8_t* descriptor, uint32_t address) {
    for (uint8_t descriptor_offset = 0u; descriptor_offset < DMA_TCD_SIZE;
         descriptor_offset += 4u) {
        uint32_t descriptor_word = 0u;
        if (!dma_bus_read(data, address + descriptor_offset, 4u, &descriptor_word))
            return false;
        k22_data_internal_store_bytes(descriptor, descriptor_offset, 4u, descriptor_word);
    }
    return true;
}

static void dma_complete_major(K22Data* data, uint8_t channel, uint8_t* descriptor) {
    uint32_t source_address = k22_data_internal_load_bytes(descriptor, 0u, 4u);
    uint32_t destination_address = k22_data_internal_load_bytes(descriptor, 0x10u, 4u);
    source_address = (uint32_t)((int64_t)source_address +
                                (int32_t)k22_data_internal_load_bytes(descriptor, 0x0cu, 4u));
    destination_address = (uint32_t)((int64_t)destination_address +
                                     (int32_t)k22_data_internal_load_bytes(descriptor, 0x18u, 4u));
    k22_data_internal_store_bytes(descriptor, 0u, 4u, source_address);
    k22_data_internal_store_bytes(descriptor, 0x10u, 4u, destination_address);
    uint16_t control_flags = (uint16_t)k22_data_internal_load_bytes(descriptor, 0x1cu, 2u);
    const uint16_t initial_iteration_count =
        dma_iteration_count((uint16_t)k22_data_internal_load_bytes(descriptor, 0x1e, 2));
    dma_set_iteration_count(descriptor, 0x16u, initial_iteration_count);
    if ((control_flags & 0x10u) != 0u) {
        const uint32_t next_descriptor_address =
            k22_data_internal_load_bytes(descriptor, 0x18u, 4u);
        if (!dma_copy_descriptor(data, descriptor, next_descriptor_address)) {
            k22_data_internal_dma_error(data, channel, 1u << 2);
            return;
        }
        control_flags = (uint16_t)k22_data_internal_load_bytes(descriptor, 0x1cu, 2u);
    } else {
        control_flags |= 0x80u;
        k22_data_internal_store_bytes(descriptor, 0x1cu, 2u, control_flags);
    }
    if ((control_flags & 0x08u) != 0u) {
        uint16_t enabled_channels = (uint16_t)k22_data_internal_load_bytes(data->dma, 0x0cu, 2u);
        enabled_channels &= (uint16_t)~(1u << channel);
        k22_data_internal_store_bytes(data->dma, 0x0cu, 2u, enabled_channels);
    }
    if ((control_flags & 0x02u) != 0u) {
        uint16_t pending_interrupt_mask =
            (uint16_t)k22_data_internal_load_bytes(data->dma, 0x24u, 2u);
        pending_interrupt_mask |= (uint16_t)(1u << channel);
        k22_data_internal_store_bytes(data->dma, 0x24u, 2u, pending_interrupt_mask);
    }
    if ((control_flags & 0x20u) != 0u)
        dma_queue_channel(data, (uint8_t)((control_flags >> 8u) & 15u));
}

bool k22_data_internal_dma_service_channel(K22Data* data, uint8_t channel) {
    uint8_t* transfer_descriptor = data->dma + 0x1000u + (uint32_t)channel * DMA_TCD_SIZE;
    uint16_t encoded_iteration =
        (uint16_t)k22_data_internal_load_bytes(transfer_descriptor, 0x16u, 2u);
    uint16_t iteration_count = dma_iteration_count(encoded_iteration);
    const uint16_t initial_iteration_count =
        dma_iteration_count((uint16_t)k22_data_internal_load_bytes(transfer_descriptor, 0x1eu, 2u));
    const uint32_t minor_loop_config = k22_data_internal_load_bytes(transfer_descriptor, 8u, 4u);
    const bool minor_offset_mapping =
        (k22_data_internal_load_bytes(data->dma, 0u, 4u) & 0x80u) != 0u;
    const uint32_t transfer_byte_count =
        minor_offset_mapping ? minor_loop_config & 0x3ffu : minor_loop_config & 0x3fffffffu;
    int32_t minor_loop_offset = 0;
    if (minor_offset_mapping) {
        minor_loop_offset = (int32_t)((minor_loop_config >> 10u) & 0xfffffu);
        if ((minor_loop_offset & 0x80000) != 0)
            minor_loop_offset |= (int32_t)0xfff00000u;
    }
    const uint16_t transfer_attributes =
        (uint16_t)k22_data_internal_load_bytes(transfer_descriptor, 6u, 2u);
    const uint8_t source_transfer_size =
        dma_transfer_size((uint8_t)((transfer_attributes >> 8u) & 7u));
    const uint8_t destination_transfer_size =
        dma_transfer_size((uint8_t)(transfer_attributes & 7u));
    if (iteration_count == 0u || transfer_byte_count == 0u || source_transfer_size == 0u ||
        destination_transfer_size == 0u || transfer_byte_count % source_transfer_size != 0u ||
        transfer_byte_count % destination_transfer_size != 0u) {
        k22_data_internal_dma_error(data, channel, 1u << 4);
        return false;
    }
    uint32_t source_address = k22_data_internal_load_bytes(transfer_descriptor, 0u, 4u);
    uint32_t destination_address = k22_data_internal_load_bytes(transfer_descriptor, 0x10u, 4u);
    const int16_t source_address_delta =
        (int16_t)k22_data_internal_load_bytes(transfer_descriptor, 4u, 2u);
    const int16_t destination_address_delta =
        (int16_t)k22_data_internal_load_bytes(transfer_descriptor, 0x14u, 2u);
    const uint8_t source_modulo_bits = (uint8_t)((transfer_attributes >> 11u) & 31u);
    const uint8_t destination_modulo_bits = (uint8_t)((transfer_attributes >> 3u) & 31u);
    uint16_t running_control =
        (uint16_t)k22_data_internal_load_bytes(transfer_descriptor, 0x1cu, 2u);
    k22_data_internal_store_bytes(transfer_descriptor, 0x1cu, 2u, running_control | 0x40u);
    data->dma_active |= (uint16_t)(1u << channel);
    uint64_t transfer_buffer = 0;
    uint8_t buffered_bytes = 0;
    uint32_t source_bytes = 0;
    uint32_t destination_bytes = 0;
    while (destination_bytes < transfer_byte_count) {
        while (buffered_bytes < destination_transfer_size && source_bytes < transfer_byte_count) {
            uint32_t transfer_value = 0u;
            if (!dma_bus_read(data, source_address, source_transfer_size, &transfer_value)) {
                k22_data_internal_dma_error(data, channel, 1u << 2);
                data->dma_active &= (uint16_t)~(1u << channel);
                k22_data_internal_store_bytes(transfer_descriptor, 0x1cu, 2u, running_control);
                return false;
            }
            transfer_buffer |= (uint64_t)transfer_value << (buffered_bytes * 8u);
            buffered_bytes = (uint8_t)(buffered_bytes + source_transfer_size);
            source_bytes += source_transfer_size;
            source_address =
                dma_advance_address(source_address, source_address_delta, source_modulo_bits);
        }
        if (buffered_bytes < destination_transfer_size ||
            !dma_bus_write(data, destination_address, destination_transfer_size,
                           (uint32_t)transfer_buffer)) {
            k22_data_internal_dma_error(data, channel, 1u << 2);
            data->dma_active &= (uint16_t)~(1u << channel);
            k22_data_internal_store_bytes(transfer_descriptor, 0x1cu, 2u, running_control);
            return false;
        }
        destination_address = dma_advance_address(destination_address, destination_address_delta,
                                                  destination_modulo_bits);
        destination_bytes += destination_transfer_size;
        transfer_buffer >>= destination_transfer_size * 8u;
        buffered_bytes = (uint8_t)(buffered_bytes - destination_transfer_size);
    }
    if (minor_offset_mapping && (minor_loop_config & 0x80000000u) != 0u)
        source_address = (uint32_t)((int64_t)source_address + minor_loop_offset);
    if (minor_offset_mapping && (minor_loop_config & 0x40000000u) != 0u)
        destination_address = (uint32_t)((int64_t)destination_address + minor_loop_offset);
    data->dma_active &= (uint16_t)~(1u << channel);
    k22_data_internal_store_bytes(transfer_descriptor, 0x1cu, 2u, running_control);
    k22_data_internal_store_bytes(transfer_descriptor, 0u, 4u, source_address);
    k22_data_internal_store_bytes(transfer_descriptor, 0x10u, 4u, destination_address);
    iteration_count--;
    dma_set_iteration_count(transfer_descriptor, 0x16u, iteration_count);
    if ((encoded_iteration & 0x8000u) != 0u)
        dma_queue_channel(data, dma_link_channel(encoded_iteration));
    const uint16_t control_flags =
        (uint16_t)k22_data_internal_load_bytes(transfer_descriptor, 0x1cu, 2u);
    if (initial_iteration_count > 1u && iteration_count == initial_iteration_count / 2u &&
        (control_flags & 0x04u) != 0u && (data->dma_half & (1u << channel)) == 0) {
        data->dma_half |= (uint16_t)(1u << channel);
        uint16_t pending_interrupt_mask =
            (uint16_t)k22_data_internal_load_bytes(data->dma, 0x24u, 2u);
        k22_data_internal_store_bytes(data->dma, 0x24u, 2u,
                                      pending_interrupt_mask | (1u << channel));
    }
    if (iteration_count == 0u) {
        data->dma_half &= (uint16_t)~(1u << channel);
        dma_complete_major(data, channel, transfer_descriptor);
    }
    k22_data_internal_dma_update_interrupts(data);
    return true;
}

bool k22_data_internal_dma_read(K22Data* data, uint32_t address, uint8_t byte_count,
                                uint32_t* output_value) {
    const uint32_t register_offset = address - DMA_BASE;
    if (!k22_data_internal_valid_access(register_offset, byte_count, DMA_REGISTER_SIZE))
        return false;
    if (register_offset >= 0x100u && register_offset < 0x110u) {
        if (byte_count != 1u || dma_priority_channel(register_offset) >= data->dma_channel_count)
            return false;
    }
    if (register_offset >= 0x1000u &&
        (register_offset - 0x1000u) / DMA_TCD_SIZE >= data->dma_channel_count)
        return false;
    if (register_offset == 0x30u && (byte_count == 2u || byte_count == 4u)) {
        *output_value = data->dma_active;
        return true;
    }
    if (register_offset == 0x34u && (byte_count == 2u || byte_count == 4u)) {
        *output_value = data->dma_hardware_requests;
        return true;
    }
    *output_value = k22_data_internal_load_bytes(data->dma, register_offset, byte_count);
    return true;
}

static void dma_command(K22Data* data, uint32_t command_offset, uint8_t command_code) {
    const uint8_t channel = command_code & 15u;
    uint16_t command_value = 0u;
    if (command_offset >= 0x18u && command_offset <= 0x1fu) {
        uint32_t target_register_offset = 0u;
        bool set_bits = false;
        if (command_offset == 0x18u || command_offset == 0x19u)
            target_register_offset = 0x14u;
        else if (command_offset == 0x1au || command_offset == 0x1bu)
            target_register_offset = 0x0cu;
        else if (command_offset == 0x1fu)
            target_register_offset = 0x24u;
        else if (command_offset == 0x1eu)
            target_register_offset = 0x2cu;
        if (command_offset == 0x19u || command_offset == 0x1bu)
            set_bits = true;
        if (command_offset == 0x1du) {
            dma_queue_channel(data, channel);
            return;
        }
        if (command_offset == 0x1cu) {
            uint8_t* transfer_descriptor = data->dma + 0x1000u + (uint32_t)channel * DMA_TCD_SIZE;
            uint16_t control_flags =
                (uint16_t)k22_data_internal_load_bytes(transfer_descriptor, 0x1cu, 2u);
            k22_data_internal_store_bytes(transfer_descriptor, 0x1cu, 2u, control_flags & ~0x80u);
            return;
        }
        command_value =
            (uint16_t)k22_data_internal_load_bytes(data->dma, target_register_offset, 2u);
        if ((command_code & 0x40u) != 0u)
            command_value = set_bits ? 0xffffu : 0u;
        else if (set_bits)
            command_value |= (uint16_t)(1u << channel);
        else
            command_value &= (uint16_t)~(1u << channel);
        k22_data_internal_store_bytes(data->dma, target_register_offset, 2u, command_value);
        if (command_offset == 0x1eu && command_value == 0u)
            k22_data_internal_store_bytes(data->dma, 0x04u, 4u, 0u);
    }
}

bool k22_data_internal_dma_write(K22Data* data, uint32_t address, uint8_t byte_count,
                                 uint32_t write_value) {
    const uint32_t register_offset = address - DMA_BASE;
    if (!k22_data_internal_valid_access(register_offset, byte_count, DMA_REGISTER_SIZE))
        return false;
    if (register_offset >= 0x100u && register_offset < 0x110u) {
        if (byte_count != 1u || dma_priority_channel(register_offset) >= data->dma_channel_count)
            return false;
        data->dma[register_offset] = (uint8_t)write_value & 0xcfu;
        return true;
    }
    if (register_offset >= 0x1000u &&
        (register_offset - 0x1000u) / DMA_TCD_SIZE >= data->dma_channel_count)
        return false;
    if (register_offset >= 0x18u && register_offset <= 0x1fu && byte_count == 1u) {
        dma_command(data, register_offset, (uint8_t)write_value);
        for (uint8_t channel = 0u; channel < data->dma_channel_count; channel++)
            k22_data_internal_dma_queue_always_enabled(data, channel);
        k22_data_internal_dma_update_interrupts(data);
        return true;
    }
    if (register_offset == 0x04u || register_offset == 0x24u || register_offset == 0x28u ||
        register_offset == 0x2cu || register_offset == 0x30u || register_offset == 0x34u)
        return false;
    if (register_offset >= 0x1000u) {
        const uint32_t tcd_offset = (register_offset - 0x1000u) % DMA_TCD_SIZE;
        const uint8_t channel = (uint8_t)((register_offset - 0x1000u) / DMA_TCD_SIZE);
        if (tcd_offset == 0x1cu && (byte_count == 1u || byte_count == 2u)) {
            const uint16_t previous_control =
                (uint16_t)k22_data_internal_load_bytes(data->dma, register_offset, 2u);
            const uint16_t status_flags = previous_control & 0x00c0u;
            const uint16_t writable_control =
                byte_count == 1u ? (previous_control & 0xff00u) | (write_value & 0x3eu)
                                 : write_value & 0xcf3eu;
            k22_data_internal_store_bytes(data->dma, register_offset, 2u,
                                          writable_control | status_flags);
            if ((write_value & 1u) != 0u)
                dma_queue_channel(data, channel);
            return true;
        }
        if (tcd_offset == 0x1du && byte_count == 1u) {
            uint16_t control_flags =
                (uint16_t)k22_data_internal_load_bytes(data->dma, register_offset - 1u, 2u);
            control_flags = (uint16_t)((control_flags & 0x00ffu) | ((write_value & 0x7fu) << 8u));
            k22_data_internal_store_bytes(data->dma, register_offset - 1u, 2u, control_flags);
            return true;
        }
    }
    k22_data_internal_store_bytes(data->dma, register_offset, byte_count, write_value);
    if (register_offset <= 0x0du && register_offset + byte_count > 0x0cu)
        for (uint8_t channel = 0u; channel < data->dma_channel_count; channel++)
            k22_data_internal_dma_queue_always_enabled(data, channel);
    return true;
}

bool k22_data_internal_dmamux_read(K22Data* data, uint32_t address, uint8_t byte_count,
                                   uint32_t* output_value) {
    const uint32_t register_offset = address - DMAMUX_BASE;
    if (!k22_data_internal_valid_access(register_offset, byte_count, data->dmamux_count))
        return false;
    *output_value = k22_data_internal_load_bytes(data->dmamux, register_offset, byte_count);
    return true;
}

bool k22_data_internal_dmamux_write(K22Data* data, uint32_t address, uint8_t byte_count,
                                    uint32_t write_value) {
    const uint32_t register_offset = address - DMAMUX_BASE;
    if (!k22_data_internal_valid_access(register_offset, byte_count, data->dmamux_count))
        return false;
    k22_data_internal_store_bytes(data->dmamux, register_offset, byte_count, write_value);
    for (uint8_t channel = (uint8_t)register_offset; channel < register_offset + byte_count;
         channel++) {
        data->dma_trigger_waiting &= (uint16_t)~(1u << channel);
        k22_data_internal_dma_queue_always_enabled(data, channel);
    }
    return true;
}
