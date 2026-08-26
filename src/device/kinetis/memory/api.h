#ifndef KINETIS_SIM_DATA_H
#define KINETIS_SIM_DATA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device/kinetis/variants/profile.h"

typedef struct KinetisData KinetisData;

typedef enum {
    KINETIS_DATA_INTERRUPT_DMA0,
    KINETIS_DATA_INTERRUPT_DMA1,
    KINETIS_DATA_INTERRUPT_DMA2,
    KINETIS_DATA_INTERRUPT_DMA3,
    KINETIS_DATA_INTERRUPT_DMA4,
    KINETIS_DATA_INTERRUPT_DMA5,
    KINETIS_DATA_INTERRUPT_DMA6,
    KINETIS_DATA_INTERRUPT_DMA7,
    KINETIS_DATA_INTERRUPT_DMA8,
    KINETIS_DATA_INTERRUPT_DMA9,
    KINETIS_DATA_INTERRUPT_DMA10,
    KINETIS_DATA_INTERRUPT_DMA11,
    KINETIS_DATA_INTERRUPT_DMA12,
    KINETIS_DATA_INTERRUPT_DMA13,
    KINETIS_DATA_INTERRUPT_DMA14,
    KINETIS_DATA_INTERRUPT_DMA15,
    KINETIS_DATA_INTERRUPT_DMA_ERROR,
    KINETIS_DATA_INTERRUPT_FTFA,
    KINETIS_DATA_INTERRUPT_FLASH_COLLISION,
    KINETIS_DATA_INTERRUPT_ADC0,
    KINETIS_DATA_INTERRUPT_ADC1,
    KINETIS_DATA_INTERRUPT_DAC0,
    KINETIS_DATA_INTERRUPT_DAC1,
    KINETIS_DATA_INTERRUPT_CMP0,
    KINETIS_DATA_INTERRUPT_CMP1,
    KINETIS_DATA_INTERRUPT_CMP2,
    KINETIS_DATA_INTERRUPT_RNG,
    KINETIS_DATA_INTERRUPT_COUNT,
} KinetisDataInterrupt;

typedef struct {
    void* context;
    bool (*read)(void* context, uint32_t address, uint8_t byte_count, uint32_t* output_value);
    bool (*write)(void* context, uint32_t address, uint8_t byte_count, uint32_t write_value);
    bool (*program)(void* context, uint32_t address, uint8_t byte_count, uint32_t write_value);
    void (*interrupt)(void* context, KinetisDataInterrupt interrupt, bool asserted);
    void (*dma_complete)(void* context, uint8_t request_source);
} KinetisDataBus;

KinetisData* kinetis_data_create(const KinetisDeviceProfile* profile, KinetisDataBus bus);
void kinetis_data_destroy(KinetisData* data);
void kinetis_data_reset(KinetisData* data);
bool kinetis_data_copy(KinetisData* destination, const KinetisData* source);
bool kinetis_data_read(KinetisData* data, uint32_t address, uint8_t byte_count,
                       uint32_t* output_value);
bool kinetis_data_write(KinetisData* data, uint32_t address, uint8_t byte_count,
                        uint32_t write_value);
void kinetis_data_advance(KinetisData* data, uint32_t cycle_count);
void kinetis_data_set_debug_halted(KinetisData* data, bool halted);
bool kinetis_data_dma_request(KinetisData* data, uint8_t request_source);
bool kinetis_data_dma_trigger(KinetisData* data, uint8_t channel);
void kinetis_data_adc_trigger(KinetisData* data, uint8_t instance);
void kinetis_data_adc_pretrigger(KinetisData* data, uint8_t instance, uint8_t pretrigger);
bool kinetis_data_set_adc_input(KinetisData* data, uint8_t instance, KinetisAdcMux mux,
                                uint8_t channel, uint16_t sample_value);
bool kinetis_data_set_cmp_input(KinetisData* data, uint8_t instance, uint8_t input_index,
                                uint8_t input_level);
bool kinetis_data_get_cmp_output(const KinetisData* data, uint8_t instance, bool* output_high);
bool kinetis_data_get_dac_output(const KinetisData* data, uint8_t instance, uint16_t* output_value);
void kinetis_data_dac_trigger(KinetisData* data, uint8_t instance);
void kinetis_data_rng_seed(KinetisData* data, uint32_t seed_value);
bool kinetis_data_set_flash_configuration(KinetisData* data, const uint8_t* bytes,
                                          size_t byte_count);
bool kinetis_data_flash_read(KinetisData* data, bool data_flash, uint32_t byte_offset,
                             uint8_t byte_count);
uint32_t kinetis_data_program_flash_address(const KinetisData* data, uint32_t address);

#endif
