#include "internal.h"

static uint32_t reverse_bits(uint32_t input_value, uint8_t bit_count) {
    uint32_t reversed_value = 0u;
    for (uint8_t bit_index = 0u; bit_index < bit_count; bit_index++) {
        reversed_value <<= 1;
        reversed_value |= (input_value >> bit_index) & 1u;
    }
    return reversed_value;
}

static void crc_accumulate(K22Data* data, uint8_t input_byte) {
    const bool wide_crc = (data->crc_control & 0x01000000u) != 0u;
    const uint8_t crc_bit_count = wide_crc ? 32u : 16u;
    const uint32_t crc_mask = wide_crc ? UINT32_MAX : 0xffffu;
    uint32_t input_value = input_byte;
    data->crc_value &= crc_mask;
    data->crc_value ^= input_value << (crc_bit_count - 8u);
    for (uint8_t bit_index = 0u; bit_index < 8u; bit_index++) {
        const bool top_bit = (data->crc_value & (1u << (crc_bit_count - 1u))) != 0u;
        data->crc_value = (data->crc_value << 1) & crc_mask;
        if (top_bit)
            data->crc_value ^= data->crc_polynomial & crc_mask;
    }
}

static uint32_t crc_result(const K22Data* data) {
    const bool wide_crc = (data->crc_control & 0x01000000u) != 0u;
    const uint8_t crc_bit_count = wide_crc ? 32u : 16u;
    const uint32_t stored_high = wide_crc ? 0u : data->crc_value & 0xffff0000u;
    uint32_t crc_value = data->crc_value;
    const uint8_t transpose = (uint8_t)((data->crc_control >> 28) & 3u);
    uint32_t transformed = 0;
    const uint8_t byte_count = (uint8_t)(crc_bit_count / 8u);
    for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
        uint8_t output_byte = (uint8_t)(crc_value >> (byte_index * 8u));
        if (transpose == 1 || transpose == 2)
            output_byte = (uint8_t)reverse_bits(output_byte, 8u);
        const uint8_t target_index =
            transpose >= 2 ? (uint8_t)(byte_count - 1u - byte_index) : byte_index;
        transformed |= (uint32_t)output_byte << (target_index * 8u);
    }
    crc_value = transformed;
    if ((data->crc_control & 0x04000000u) != 0)
        crc_value = ~crc_value;
    return wide_crc ? crc_value : stored_high | (crc_value & 0xffffu);
}

bool k22_data_internal_crc_read(K22Data* data, uint32_t address, uint8_t byte_count,
                                uint32_t* output_value) {
    const uint32_t register_offset = address - CRC_BASE;
    if (!k22_data_internal_valid_access(register_offset, byte_count, 12u))
        return false;
    if (register_offset < 4u) {
        *output_value = crc_result(data) >> (register_offset * 8u);
        if (byte_count < 4u)
            *output_value &= (1u << (byte_count * 8u)) - 1u;
    } else if (register_offset < 8u)
        *output_value = data->crc_polynomial >> ((register_offset - 4u) * 8u);
    else
        *output_value = data->crc_control >> ((register_offset - 8u) * 8u);
    return true;
}

bool k22_data_internal_crc_write(K22Data* data, uint32_t address, uint8_t byte_count,
                                 uint32_t write_value) {
    const uint32_t register_offset = address - CRC_BASE;
    if (!k22_data_internal_valid_access(register_offset, byte_count, 12u))
        return false;
    if (register_offset < 4u) {
        if ((data->crc_control & 0x02000000u) != 0) {
            uint32_t mask = byte_count == 4u ? UINT32_MAX : (1u << (byte_count * 8u)) - 1u;
            data->crc_value = (data->crc_value & ~(mask << (register_offset * 8u))) |
                              ((write_value & mask) << (register_offset * 8u));
            return true;
        }
        const uint8_t transpose = (uint8_t)((data->crc_control >> 30) & 3u);
        for (uint8_t byte_index = 0u; byte_index < byte_count; byte_index++) {
            const uint8_t source_index =
                transpose >= 2 ? (uint8_t)(byte_count - 1u - byte_index) : byte_index;
            uint8_t input_byte = (uint8_t)(write_value >> (source_index * 8u));
            if (transpose == 1 || transpose == 2)
                input_byte = (uint8_t)reverse_bits(input_byte, 8u);
            crc_accumulate(data, input_byte);
        }
        return true;
    }
    if (register_offset < 8u) {
        uint8_t register_bytes[4];
        k22_data_internal_store_bytes(register_bytes, 0u, 4u, data->crc_polynomial);
        k22_data_internal_store_bytes(register_bytes, register_offset - 4u, byte_count,
                                      write_value);
        data->crc_polynomial = k22_data_internal_load_bytes(register_bytes, 0u, 4u);
    } else {
        uint8_t register_bytes[4];
        k22_data_internal_store_bytes(register_bytes, 0u, 4u, data->crc_control);
        k22_data_internal_store_bytes(register_bytes, register_offset - 8u, byte_count,
                                      write_value);
        data->crc_control = k22_data_internal_load_bytes(register_bytes, 0u, 4u) & 0xf7000000u;
    }
    return true;
}

uint32_t k22_data_internal_rng_next(uint32_t seed_value) {
    seed_value ^= seed_value << 13;
    seed_value ^= seed_value >> 17;
    seed_value ^= seed_value << 5;
    return seed_value == 0u ? 0x6d2b79f5u : seed_value;
}

bool k22_data_internal_rng_read(K22Data* data, uint32_t address, uint8_t byte_count,
                                uint32_t* output_value) {
    const uint32_t register_offset = address - RNG_BASE;
    if (!k22_data_internal_valid_access(register_offset, byte_count, 16u) || byte_count != 4u)
        return false;
    if (register_offset == 0u)
        *output_value = data->rng_control;
    else if (register_offset == 4u)
        *output_value = data->rng_status;
    else if (register_offset == 8u)
        *output_value = data->rng_error;
    else if (register_offset == 12u) {
        *output_value = data->rng_output;
        data->rng_status &= ~1u;
        k22_data_internal_interrupt(data, K22_DATA_INTERRUPT_RNG, false);
    } else
        return false;
    return true;
}

bool k22_data_internal_rng_write(K22Data* data, uint32_t address, uint8_t byte_count,
                                 uint32_t write_value) {
    const uint32_t register_offset = address - RNG_BASE;
    if (byte_count != 4u || !k22_data_internal_valid_access(register_offset, byte_count, 16u))
        return false;
    if (register_offset != 0u)
        return false;
    data->rng_control = write_value & 0x1fu;
    if ((write_value & 0x10u) != 0u) {
        data->rng_status = 0;
        data->rng_error = 0;
        k22_data_internal_interrupt(data, K22_DATA_INTERRUPT_RNG, false);
    }
    if ((write_value & 1u) != 0u)
        data->rng_cycles = 64u;
    return true;
}
