#ifndef KINETIS_SIM_DATA_INTERNAL_H
#define KINETIS_SIM_DATA_INTERNAL_H

#include "device/kinetis/memory/api.h"

enum {
    DMA_BASE = 0x40008000u,
    DMA_TCD_BASE = 0x40009000u,
    DMAMUX_BASE = 0x40021000u,
    ADC0_BASE = 0x4003b000u,
    ADC1_BASE_STANDARD = 0x40027000u,
    ADC1_BASE_LARGE = 0x400bb000u,
    DAC0_BASE_STANDARD = 0x4003f000u,
    DAC0_BASE_LARGE = 0x400cc000u,
    DAC1_BASE = 0x40028000u,
    RNG_BASE = 0x40029000u,
    CRC_BASE = 0x40032000u,
    FLASH_BASE = 0x40020000u,
    CMP_BASE = 0x40073000u,
    VREF_BASE = 0x40074000u,
    DMA_CHANNEL_COUNT = 16,
    DMA_TCD_SIZE = 32,
    DMA_REGISTER_SIZE = 0x1200,
    ADC_REGISTER_SIZE = 0x70,
    DAC_REGISTER_SIZE = 0x24,
    CMP_REGISTER_SIZE = 6,
};

typedef struct {
    uint8_t registers[ADC_REGISTER_SIZE];
    uint16_t inputs[KINETIS_ADC_MUX_COUNT][32];
    uint64_t clock_remainder;
    uint32_t remaining_cycles;
    uint8_t active_slot;
    bool converting;
} KinetisAdc;

typedef struct {
    uint8_t registers[DAC_REGISTER_SIZE];
    uint16_t output;
} KinetisDac;

typedef struct {
    uint8_t registers[CMP_REGISTER_SIZE];
    uint8_t inputs[8];
} KinetisCmp;

struct KinetisData {
    const KinetisDeviceProfile* profile;
    KinetisDataBus bus;
    uint8_t dma[DMA_REGISTER_SIZE];
    uint16_t dma_requests;
    uint16_t dma_hardware_requests;
    uint16_t dma_trigger_waiting;
    uint16_t dma_active;
    uint16_t dma_half;
    uint8_t dma_request_source[DMA_CHANNEL_COUNT];
    uint8_t dma_channel_count;
    uint8_t dma_last_channel;
    uint32_t core_clock_hz;
    uint32_t bus_clock_hz;
    bool bus_clock_running;
    bool debug_halted;
    uint8_t dmamux[DMA_CHANNEL_COUNT];
    uint8_t dmamux_count;
    KinetisAdc adc[2];
    uint8_t adc_count;
    uint32_t adc_base[2];
    KinetisDac dac[2];
    uint8_t dac_count;
    uint32_t dac_base[2];
    KinetisCmp cmp[3];
    uint8_t cmp_count;
    uint8_t vref[2];
    uint32_t vref_cycles;
    uint32_t rng_control;
    uint32_t rng_status;
    uint32_t rng_error;
    uint32_t rng_output;
    uint32_t rng_state;
    uint32_t rng_cycles;
    uint32_t crc_value;
    uint32_t crc_polynomial;
    uint32_t crc_control;
    uint8_t flash[0x2c];
    uint8_t flash_config[0x10];
    uint8_t flash_program_ifr[1024];
    uint8_t flash_data_ifr[1024];
    uint32_t flash_cycles;
    uint8_t flash_busy_banks;
    uint32_t flash_busy_start;
    uint32_t flash_busy_length;
    uint32_t flash_swap_address;
    uint8_t flash_swap_mode;
    uint8_t flash_swap_current_block;
    uint8_t flash_swap_next_block;
    bool flash_key_blocked;
    bool flash_partitioned;
    bool flexram_eeprom;
    uint8_t* flexnvm;
    uint8_t* flexram;
    uint8_t* eeprom;
};

bool kinetis_data_internal_adc_read(KinetisData* data, uint8_t instance, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value);
bool kinetis_data_internal_adc_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value);
bool kinetis_data_internal_cmp_read(KinetisData* data, uint8_t instance, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value);
bool kinetis_data_internal_cmp_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value);
bool kinetis_data_internal_crc_read(KinetisData* data, uint32_t address, uint8_t byte_count,
                                    uint32_t* output_value);
bool kinetis_data_internal_crc_write(KinetisData* data, uint32_t address, uint8_t byte_count,
                                     uint32_t write_value);
bool kinetis_data_internal_dac_read(KinetisData* data, uint8_t instance, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value);
bool kinetis_data_internal_dac_write(KinetisData* data, uint8_t instance, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value);
bool kinetis_data_internal_dma_priorities_valid(const KinetisData* data);
bool kinetis_data_internal_dma_read(KinetisData* data, uint32_t address, uint8_t byte_count,
                                    uint32_t* output_value);
bool kinetis_data_internal_dma_service_channel(KinetisData* data, uint8_t channel);
bool kinetis_data_internal_dma_source_always_enabled(const KinetisData* data,
                                                     uint8_t request_source);
bool kinetis_data_internal_dma_source_valid(const KinetisData* data, uint8_t request_source);
bool kinetis_data_internal_dma_write(KinetisData* data, uint32_t address, uint8_t byte_count,
                                     uint32_t write_value);
bool kinetis_data_internal_dmamux_read(KinetisData* data, uint32_t address, uint8_t byte_count,
                                       uint32_t* output_value);
bool kinetis_data_internal_dmamux_write(KinetisData* data, uint32_t address, uint8_t byte_count,
                                        uint32_t write_value);
bool kinetis_data_internal_flash_read(KinetisData* data, uint32_t address, uint8_t byte_count,
                                      uint32_t* output_value);
bool kinetis_data_internal_flash_write(KinetisData* data, uint32_t address, uint8_t byte_count,
                                       uint32_t write_value);
bool kinetis_data_internal_profile_block(const KinetisData* data, KinetisPeripheralId id,
                                         uint32_t* block_address, uint32_t* block_size);
bool kinetis_data_internal_rng_read(KinetisData* data, uint32_t address, uint8_t byte_count,
                                    uint32_t* output_value);
bool kinetis_data_internal_rng_write(KinetisData* data, uint32_t address, uint8_t byte_count,
                                     uint32_t write_value);
bool kinetis_data_internal_valid_access(uint32_t byte_offset, uint8_t byte_count,
                                        uint32_t buffer_length);
uint32_t kinetis_data_internal_dma_priority_offset(uint8_t channel);
uint32_t kinetis_data_internal_load_bytes(const uint8_t* input_bytes, uint32_t byte_offset,
                                          uint8_t byte_count);
uint32_t kinetis_data_internal_rng_next(uint32_t seed_value);
uint8_t kinetis_data_internal_dma_select_channel(const KinetisData* data);
void kinetis_data_internal_adc_complete(KinetisData* data, uint8_t instance);
uint32_t kinetis_data_internal_adc_elapsed_cycles(KinetisData* data, KinetisAdc* adc,
                                                  uint32_t core_cycles);
void kinetis_data_internal_adc_reset_registers(KinetisAdc* adc);
void kinetis_data_internal_adc_start(KinetisAdc* adc, uint8_t slot);
void kinetis_data_internal_cmp_evaluate(KinetisData* data, uint8_t instance);
void kinetis_data_internal_dac_flags(KinetisData* data, uint8_t instance, uint8_t event_flags);
void kinetis_data_internal_dac_update_output(KinetisData* data, uint8_t instance);
void kinetis_data_internal_dma_error(KinetisData* data, uint8_t channel, uint32_t error_reason);
void kinetis_data_internal_dma_queue_always_enabled(KinetisData* data, uint8_t channel);
void kinetis_data_internal_dma_queue_hardware_channel(KinetisData* data, uint8_t channel,
                                                      uint8_t source);
void kinetis_data_internal_dma_update_interrupts(KinetisData* data);
void kinetis_data_internal_flash_update_interrupts(KinetisData* data);
void kinetis_data_internal_interrupt(KinetisData* data, KinetisDataInterrupt line, bool asserted);
void kinetis_data_internal_store_bytes(uint8_t* output_bytes, uint32_t byte_offset,
                                       uint8_t byte_count, uint32_t write_value);

#endif
