#include "internal.h"

static uint32_t adc_conversion_cycles(const K22Adc* adc) {
    const uint8_t configuration1 = adc->registers[8];
    const uint8_t conversion_mode = (configuration1 >> 2) & 3u;
    uint32_t cycle_count = conversion_mode == 0u   ? 10u
                           : conversion_mode == 1u ? 14u
                           : conversion_mode == 2u ? 12u
                                                   : 18u;
    if ((configuration1 & 0x10u) != 0u)
        cycle_count += (adc->registers[9] & 3u) == 0u ? 12u : 20u;
    if ((adc->registers[0x24] & 0x04u) != 0)
        cycle_count *= 1u << ((adc->registers[0x24] & 3u) + 2u);
    return cycle_count;
}

static void adc_schedule(K22Adc* adc, uint8_t conversion_slot, bool clear_complete_flag) {
    if ((adc->registers[conversion_slot * 4u] & 0x1fu) == 31u)
        return;
    if (clear_complete_flag)
        adc->registers[conversion_slot * 4u] &= 0x7fu;
    adc->active_slot = conversion_slot;
    adc->remaining_cycles = adc_conversion_cycles(adc);
    adc->converting = true;
}

void k22_data_internal_adc_start(K22Adc* adc, uint8_t slot) { adc_schedule(adc, slot, true); }

static bool adc_compare(const K22Adc* adc, uint16_t conversion_result) {
    const uint8_t sc2 = adc->registers[0x20];
    if ((sc2 & 0x20u) == 0)
        return true;
    const uint16_t compare_value_low =
        (uint16_t)k22_data_internal_load_bytes(adc->registers, 0x18u, 2u);
    const uint16_t compare_value_high =
        (uint16_t)k22_data_internal_load_bytes(adc->registers, 0x1cu, 2u);
    if ((sc2 & 0x08u) != 0) {
        const bool within_range =
            conversion_result >= compare_value_low && conversion_result <= compare_value_high;
        return (sc2 & 0x10u) != 0u ? !within_range : within_range;
    }
    return (sc2 & 0x10u) != 0u ? conversion_result > compare_value_low
                               : conversion_result < compare_value_low;
}

static uint16_t adc_result(const K22Adc* adc, uint8_t channel) {
    uint16_t sample_value = adc->inputs[channel];
    const uint8_t conversion_mode = (adc->registers[8] >> 2) & 3u;
    const uint8_t resolution_bits = conversion_mode == 0u   ? 8u
                                    : conversion_mode == 1u ? 12u
                                    : conversion_mode == 2u ? 10u
                                                            : 16u;
    if (resolution_bits < 16u)
        sample_value &= (uint16_t)((1u << resolution_bits) - 1u);
    return sample_value;
}

void k22_data_internal_adc_complete(K22Data* data, uint8_t instance) {
    K22Adc* adc = &data->adc[instance];
    const uint8_t conversion_slot = adc->active_slot;
    const uint8_t channel = adc->registers[conversion_slot * 4u] & 0x1fu;
    const uint16_t conversion_result = adc_result(adc, channel);
    adc->converting = false;
    if (adc_compare(adc, conversion_result)) {
        k22_data_internal_store_bytes(adc->registers, 0x10u + conversion_slot * 4u, 2u,
                                      conversion_result);
        adc->registers[conversion_slot * 4u] |= 0x80u;
        if ((adc->registers[conversion_slot * 4u] & 0x40u) != 0u)
            k22_data_internal_interrupt(
                data, (K22DataInterrupt)(K22_DATA_INTERRUPT_ADC0 + instance), true);
        if ((adc->registers[0x20] & 0x04u) != 0)
            k22_data_dma_request(data, (uint8_t)(40u + instance));
    }
    if ((adc->registers[0x24] & 0x08u) != 0)
        adc_schedule(adc, conversion_slot, false);
}

bool k22_data_internal_adc_read(K22Data* data, uint8_t instance, uint32_t address,
                                uint8_t byte_count, uint32_t* output_value) {
    K22Adc* adc = &data->adc[instance];
    const uint32_t register_offset = address - data->adc_base[instance];
    if (!k22_data_internal_valid_access(register_offset, byte_count, ADC_REGISTER_SIZE))
        return false;
    *output_value = k22_data_internal_load_bytes(adc->registers, register_offset, byte_count);
    if ((register_offset == 0x10u || register_offset == 0x14u) && byte_count <= 4u) {
        const uint8_t conversion_slot = (uint8_t)((register_offset - 0x10u) / 4u);
        adc->registers[conversion_slot * 4u] &= 0x7fu;
        k22_data_internal_interrupt(data, (K22DataInterrupt)(K22_DATA_INTERRUPT_ADC0 + instance),
                                    false);
    }
    return true;
}

bool k22_data_internal_adc_write(K22Data* data, uint8_t instance, uint32_t address,
                                 uint8_t byte_count, uint32_t write_value) {
    K22Adc* adc = &data->adc[instance];
    const uint32_t register_offset = address - data->adc_base[instance];
    if (!k22_data_internal_valid_access(register_offset, byte_count, ADC_REGISTER_SIZE))
        return false;
    if (register_offset == 0x10u || register_offset == 0x14u || register_offset == 0x0cu ||
        register_offset == 0x28u)
        return false;
    if (register_offset == 0u || register_offset == 4u) {
        const uint8_t conversion_slot = (uint8_t)(register_offset / 4u);
        k22_data_internal_store_bytes(adc->registers, register_offset, byte_count,
                                      write_value & 0x7fu);
        k22_data_internal_interrupt(data, (K22DataInterrupt)(K22_DATA_INTERRUPT_ADC0 + instance),
                                    false);
        if ((adc->registers[0x20] & 0x40u) == 0)
            k22_data_internal_adc_start(adc, conversion_slot);
        return true;
    }
    if (register_offset == 0x24u) {
        const uint8_t command_code = (uint8_t)write_value;
        adc->registers[0x24] = command_code & 0xcfu;
        if ((command_code & 0x80u) != 0u) {
            adc->registers[0x24] &= 0x3fu;
            adc->registers[0] |= 0x80u;
            k22_data_internal_store_bytes(adc->registers, 0x30, 2, 0x8200u);
            k22_data_internal_store_bytes(adc->registers, 0x34, 2, 0x8200u);
            k22_data_internal_store_bytes(adc->registers, 0x38, 2, 0x4000u);
        }
        return true;
    }
    k22_data_internal_store_bytes(adc->registers, register_offset, byte_count, write_value);
    return true;
}

void k22_data_internal_dac_update_output(K22Data* data, uint8_t instance) {
    K22Dac* dac = &data->dac[instance];
    if ((dac->registers[0x21] & 0x80u) == 0) {
        dac->output = 0;
        return;
    }
    uint8_t pointer = (dac->registers[0x23] >> 4) & 15u;
    dac->output =
        (uint16_t)k22_data_internal_load_bytes(dac->registers, (uint32_t)pointer * 2u, 2) & 0x0fffu;
}

void k22_data_internal_dac_flags(K22Data* data, uint8_t instance, uint8_t flags) {
    K22Dac* dac = &data->dac[instance];
    dac->registers[0x20] |= flags;
    const uint8_t enables = dac->registers[0x21] & 7u;
    k22_data_internal_interrupt(data, (K22DataInterrupt)(K22_DATA_INTERRUPT_DAC0 + instance),
                                (enables & flags) != 0);
    if ((dac->registers[0x22] & 0x80u) != 0 && flags != 0)
        k22_data_dma_request(data, (uint8_t)(45u + instance));
}

bool k22_data_internal_dac_read(K22Data* data, uint8_t instance, uint32_t address, uint8_t size,
                                uint32_t* value) {
    const uint32_t offset = address - data->dac_base[instance];
    if (!k22_data_internal_valid_access(offset, size, DAC_REGISTER_SIZE))
        return false;
    *value = k22_data_internal_load_bytes(data->dac[instance].registers, offset, size);
    return true;
}

bool k22_data_internal_dac_write(K22Data* data, uint8_t instance, uint32_t address, uint8_t size,
                                 uint32_t value) {
    K22Dac* dac = &data->dac[instance];
    const uint32_t offset = address - data->dac_base[instance];
    if (!k22_data_internal_valid_access(offset, size, DAC_REGISTER_SIZE))
        return false;
    if (offset == 0x20) {
        dac->registers[0x20] &= (uint8_t)~value;
        k22_data_internal_interrupt(data, (K22DataInterrupt)(K22_DATA_INTERRUPT_DAC0 + instance),
                                    false);
        return true;
    }
    k22_data_internal_store_bytes(dac->registers, offset, size, value);
    k22_data_internal_dac_update_output(data, instance);
    if (offset <= 0x1f && (dac->registers[0x22] & 0x80u) == 0)
        k22_data_internal_dac_update_output(data, instance);
    return true;
}

static uint8_t cmp_level(const K22Cmp* cmp, uint8_t selection) {
    if (selection == 7u && (cmp->registers[4] & 0x80u) != 0)
        return cmp->registers[4] & 0x3fu;
    return cmp->inputs[selection & 7u];
}

void k22_data_internal_cmp_evaluate(K22Data* data, uint8_t instance) {
    K22Cmp* cmp = &data->cmp[instance];
    const bool enabled = (cmp->registers[1] & 1u) != 0;
    const uint8_t positive = (cmp->registers[5] >> 3) & 7u;
    const uint8_t negative = cmp->registers[5] & 7u;
    bool output = enabled && cmp_level(cmp, positive) > cmp_level(cmp, negative);
    if ((cmp->registers[1] & 0x08u) != 0)
        output = !output;
    const bool previous = (cmp->registers[3] & 1u) != 0;
    if (output)
        cmp->registers[3] |= 1u;
    else
        cmp->registers[3] &= 0xfeu;
    if (!previous && output)
        cmp->registers[3] |= 4u;
    if (previous && !output)
        cmp->registers[3] |= 2u;
    const bool pending = ((cmp->registers[3] & 0x08u) != 0 && (cmp->registers[3] & 4u) != 0) ||
                         ((cmp->registers[3] & 0x10u) != 0 && (cmp->registers[3] & 2u) != 0);
    k22_data_internal_interrupt(data, (K22DataInterrupt)(K22_DATA_INTERRUPT_CMP0 + instance),
                                pending);
    if (pending && (cmp->registers[3] & 0x40u) != 0)
        k22_data_dma_request(data, (uint8_t)(42u + instance));
}

bool k22_data_internal_cmp_read(K22Data* data, uint8_t instance, uint32_t address, uint8_t size,
                                uint32_t* value) {
    const uint32_t offset = address - (CMP_BASE + (uint32_t)instance * 8u);
    if (!k22_data_internal_valid_access(offset, size, CMP_REGISTER_SIZE))
        return false;
    *value = k22_data_internal_load_bytes(data->cmp[instance].registers, offset, size);
    return true;
}

bool k22_data_internal_cmp_write(K22Data* data, uint8_t instance, uint32_t address, uint8_t size,
                                 uint32_t value) {
    K22Cmp* cmp = &data->cmp[instance];
    const uint32_t offset = address - (CMP_BASE + (uint32_t)instance * 8u);
    if (!k22_data_internal_valid_access(offset, size, CMP_REGISTER_SIZE))
        return false;
    if (offset == 3) {
        const uint8_t previous = cmp->registers[3];
        cmp->registers[3] =
            (uint8_t)((value & 0x78u) | (previous & 1u) | ((previous & 6u) & ~(value & 6u)));
    } else {
        k22_data_internal_store_bytes(cmp->registers, offset, size, value);
    }
    k22_data_internal_cmp_evaluate(data, instance);
    return true;
}
