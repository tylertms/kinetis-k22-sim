#include "internal.h"

#include <stdlib.h>
#include <string.h>

uint32_t k22_data_internal_load_bytes(const uint8_t* input_bytes, uint32_t byte_offset,
                                      uint8_t byte_count) {
    uint32_t output_value = 0u;
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++)
        output_value |= (uint32_t)input_bytes[byte_offset + byte_index] << (8u * byte_index);
    return output_value;
}

void k22_data_internal_store_bytes(uint8_t* output_bytes, uint32_t byte_offset, uint8_t byte_count,
                                   uint32_t write_value) {
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++)
        output_bytes[byte_offset + byte_index] = (uint8_t)(write_value >> (8u * byte_index));
}

void k22_data_internal_adc_reset_registers(K22Adc* adc) {
    adc->registers[0] = 0x1fu;
    adc->registers[4] = 0x1fu;
    k22_data_internal_store_bytes(adc->registers, 0x28u, 4u, 0x0004u);
    k22_data_internal_store_bytes(adc->registers, 0x2cu, 4u, 0x8200u);
    k22_data_internal_store_bytes(adc->registers, 0x30u, 4u, 0x8200u);
    k22_data_internal_store_bytes(adc->registers, 0x34u, 4u, 0x000au);
    k22_data_internal_store_bytes(adc->registers, 0x38u, 4u, 0x0020u);
    k22_data_internal_store_bytes(adc->registers, 0x3cu, 4u, 0x0200u);
    k22_data_internal_store_bytes(adc->registers, 0x40u, 4u, 0x0100u);
    k22_data_internal_store_bytes(adc->registers, 0x44u, 4u, 0x0080u);
    k22_data_internal_store_bytes(adc->registers, 0x48u, 4u, 0x0040u);
    k22_data_internal_store_bytes(adc->registers, 0x4cu, 4u, 0x0020u);
    k22_data_internal_store_bytes(adc->registers, 0x54u, 4u, 0x000au);
    k22_data_internal_store_bytes(adc->registers, 0x58u, 4u, 0x0020u);
    k22_data_internal_store_bytes(adc->registers, 0x5cu, 4u, 0x0200u);
    k22_data_internal_store_bytes(adc->registers, 0x60u, 4u, 0x0100u);
    k22_data_internal_store_bytes(adc->registers, 0x64u, 4u, 0x0080u);
    k22_data_internal_store_bytes(adc->registers, 0x68u, 4u, 0x0040u);
    k22_data_internal_store_bytes(adc->registers, 0x6cu, 4u, 0x0020u);
}

bool k22_data_internal_valid_access(uint32_t byte_offset, uint8_t byte_count,
                                    uint32_t buffer_length) {
    return (byte_count == 1u || byte_count == 2u || byte_count == 4u) &&
           byte_offset < buffer_length && byte_count <= buffer_length - byte_offset;
}

void k22_data_internal_interrupt(K22Data* data, K22DataInterrupt line, bool asserted) {
    if (data->bus.interrupt != NULL)
        data->bus.interrupt(data->bus.context, line, asserted);
}

bool k22_data_internal_profile_block(const K22Data* data, K22PeripheralId id,
                                     uint32_t* block_address, uint32_t* block_size) {
    K22PeripheralBlock block;
    if (!k22_profile_peripheral_block(data->profile, id, &block))
        return false;
    if (block_address != NULL)
        *block_address = block.address;
    if (block_size != NULL)
        *block_size = block.size;
    return true;
}
