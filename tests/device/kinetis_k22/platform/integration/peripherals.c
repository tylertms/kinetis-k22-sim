#include "device/kinetis_k22/platform/integration/internal.h"

static uint32_t wait_states(void* context, uint32_t address, uint8_t access_size,
                            CortexM4Access access, bool write_access, bool sequential) {
    WaitFixture* fixture = context;
    fixture->call_count++;
    return (address & 1u) + access_size + access + (write_access ? 1u : 0u) +
           (sequential ? 1u : 0u);
}

static void expect_serial_dma_sources(TestState* state) {
    KinetisK22* device =
        k22_integration_test_create_f12_device(state, KINETIS_K22_PACKAGE_MD_144_MAPBGA);
    CortexM4* cpu = kinetis_k22_cpu(device);
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC7, 4u, 1u << 1u),
           "cortex_m4_write_memory(cpu, SIM_SCGC7, 4u, 1u << 1u)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, 1u << 1u),
           "cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, 1u << 1u)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC1, 4u, 3u << 10u),
           "cortex_m4_write_memory(cpu, SIM_SCGC1, 4u, 3u << 10u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_ERQ, 2u, 3u),
           "cortex_m4_write_memory(cpu, DMA_ERQ, 2u, 3u)");
    expect(state, cortex_m4_write_memory(cpu, DMAMUX_CHCFG0, 1u, 0x80u | 10u),
           "cortex_m4_write_memory(cpu, DMAMUX_CHCFG0, 1u, 0x80u | 10u)");
    expect(state, cortex_m4_write_memory(cpu, DMAMUX_CHCFG0 + 1u, 1u, 0x80u | 11u),
           "cortex_m4_write_memory(cpu, DMAMUX_CHCFG0 + 1u, 1u, 0x80u | 11u)");
    expect(state, cortex_m4_write_memory(cpu, 0x400ea003u, 1u, 0x04u),
           "cortex_m4_write_memory(cpu, 0x400ea003u, 1u, 0x04u)");
    expect(state, cortex_m4_write_memory(cpu, 0x400ea00bu, 1u, 0x20u),
           "cortex_m4_write_memory(cpu, 0x400ea00bu, 1u, 0x20u)");
    expect(state, cortex_m4_write_memory(cpu, 0x400eb003u, 1u, 0x04u),
           "cortex_m4_write_memory(cpu, 0x400eb003u, 1u, 0x04u)");
    expect(state, cortex_m4_write_memory(cpu, 0x400eb00bu, 1u, 0x20u),
           "cortex_m4_write_memory(cpu, 0x400eb00bu, 1u, 0x20u)");
    expect(state, kinetis_k22_serial_receive(device, KINETIS_K22_SERIAL_UART4, 0x44u, 0u),
           "kinetis_k22_serial_receive(device, KINETIS_K22_SERIAL_UART4, 0x44u, 0u)");
    uint16_t requests = 0u;
    expect(state, k22_integration_test_read16(device, DMA_HRS, &requests),
           "k22_integration_test_read16(device, DMA_HRS, &requests)");
    expect(state, requests == 1u, "requests == 1u");
    expect(state, kinetis_k22_serial_receive(device, KINETIS_K22_SERIAL_UART5, 0x55u, 0u),
           "kinetis_k22_serial_receive(device, KINETIS_K22_SERIAL_UART5, 0x55u, 0u)");
    expect(state, k22_integration_test_read16(device, DMA_HRS, &requests),
           "k22_integration_test_read16(device, DMA_HRS, &requests)");
    expect(state, requests == 1u, "requests == 1u");
    kinetis_k22_destroy(device);
}

static void expect_periodic_dma_trigger(TestState* state) {
    KinetisK22* device =
        k22_integration_test_create_device(state, KINETIS_K22_PACKAGE_DC_121_XFBGA);
    CortexM4* cpu = kinetis_k22_cpu(device);
    const uint32_t source_address = 0x20000080u;
    const uint32_t destination_address = 0x20000081u;
    expect(state, cortex_m4_write_memory(cpu, source_address, 1u, 0x5au),
           "cortex_m4_write_memory(cpu, source_address, 1u, 0x5au)");
    expect(state, cortex_m4_write_memory(cpu, destination_address, 1u, 0u),
           "cortex_m4_write_memory(cpu, destination_address, 1u, 0u)");
    uint32_t clock_gate_value = 0u;
    expect(state, k22_integration_test_read32(device, SIM_SCGC7, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC7, &clock_gate_value)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC7, 4u, clock_gate_value | 2u),
           "cortex_m4_write_memory(cpu, SIM_SCGC7, 4u, clock_gate_value | 2u)");
    expect(state, k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, clock_gate_value | 2u | (1u << 23u)),
           "cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, clock_gate_value | 2u | (1u << 23u))");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0, 4u, source_address),
           "cortex_m4_write_memory(cpu, DMA_TCD0, 4u, source_address)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 4u, 2u, 0u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 4u, 2u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 6u, 2u, 0u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 6u, 2u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 8u, 4u, 1u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 8u, 4u, 1u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 0x0cu, 4u, 0u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 0x0cu, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 0x10u, 4u, destination_address),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 0x10u, 4u, destination_address)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 0x14u, 2u, 0u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 0x14u, 2u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 0x16u, 2u, 1u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 0x16u, 2u, 1u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 0x18u, 4u, 0u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 0x18u, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 0x1cu, 2u, 8u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 0x1cu, 2u, 8u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_TCD0 + 0x1eu, 2u, 1u),
           "cortex_m4_write_memory(cpu, DMA_TCD0 + 0x1eu, 2u, 1u)");
    expect(state, cortex_m4_write_memory(cpu, DMAMUX_CHCFG0, 1u, 0xc0u | 60u),
           "cortex_m4_write_memory(cpu, DMAMUX_CHCFG0, 1u, 0xc0u | 60u)");
    expect(state, cortex_m4_write_memory(cpu, DMA_ERQ, 2u, 1u),
           "cortex_m4_write_memory(cpu, DMA_ERQ, 2u, 1u)");
    uint16_t requests = UINT16_MAX;
    expect(state, k22_integration_test_read16(device, DMA_HRS, &requests),
           "k22_integration_test_read16(device, DMA_HRS, &requests)");
    expect(state, (requests & 1u) == 0u, "(requests & 1u) == 0u");
    expect(state, cortex_m4_write_memory(cpu, PIT_MCR, 4u, 0u),
           "cortex_m4_write_memory(cpu, PIT_MCR, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_LDVAL0, 4u, 0u),
           "cortex_m4_write_memory(cpu, PIT_LDVAL0, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 1u),
           "cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 1u)");
    kinetis_k22_advance(device, k22_test_core_cycles_for_bus_cycles(device, 1u));
    uint32_t read_value = 0u;
    expect(state, cortex_m4_read_memory(cpu, destination_address, 1u, &read_value),
           "cortex_m4_read_memory(cpu, destination_address, 1u, &read_value)");
    expect(state, read_value == 0x5au, "read_value == 0x5au");
    expect(state, k22_integration_test_read16(device, DMA_ERQ, &requests),
           "k22_integration_test_read16(device, DMA_ERQ, &requests)");
    expect(state, (requests & 1u) == 0u, "(requests & 1u) == 0u");
    kinetis_k22_destroy(device);
}

static void expect_sdhc_integration(TestState* state) {
    uint8_t card[512];
    for (size_t card_index = 0u; card_index < sizeof(card); card_index++) {
        card[card_index] = (uint8_t)card_index;
    }

    KinetisK22* unsupported_device =
        k22_integration_test_create_f12_device(state, KINETIS_K22_PACKAGE_LH_64_LQFP);
    expect(state, !kinetis_k22_sdhc_insert(unsupported_device, card, sizeof(card), false),
           "!kinetis_k22_sdhc_insert(unsupported_device, card, sizeof(card), false)");
    kinetis_k22_destroy(unsupported_device);

    KinetisK22* sdhc_device =
        k22_integration_test_create_f12_device(state, KINETIS_K22_PACKAGE_LK_80_LQFP);
    expect(state, kinetis_k22_sdhc_insert(sdhc_device, card, sizeof(card), false),
           "kinetis_k22_sdhc_insert(sdhc_device, card, sizeof(card), false)");
    CortexM4* sdhc_cpu = kinetis_k22_cpu(sdhc_device);
    expect(state, cortex_m4_write_memory(sdhc_cpu, SIM_SCGC3, 4u, 1u << 17u),
           "cortex_m4_write_memory(sdhc_cpu, SIM_SCGC3, 4u, 1u << 17u)");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b1034u, 4u, UINT32_MAX),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b1034u, 4u, UINT32_MAX)");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b1038u, 4u, UINT32_MAX),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b1038u, 4u, UINT32_MAX)");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b1008u, 4u, 0u),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b1008u, 4u, 0u)");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b100cu, 4u, 3u << 24u),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b100cu, 4u, 3u << 24u)");
    uint32_t response_word = 0u;
    expect(state, k22_integration_test_read32(sdhc_device, 0x400b1010u, &response_word),
           "k22_integration_test_read32(sdhc_device, 0x400b1010u, &response_word)");
    expect(state, response_word == 0x00010000u, "response_word == 0x00010000u");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b1008u, 4u, 0x00010000u),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b1008u, 4u, 0x00010000u)");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b100cu, 4u, 7u << 24u),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b100cu, 4u, 7u << 24u)");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b1004u, 4u, 4u | (1u << 16u)),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b1004u, 4u, 4u | (1u << 16u))");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b1008u, 4u, 0u),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b1008u, 4u, 0u)");
    expect(state, cortex_m4_write_memory(sdhc_cpu, 0x400b100cu, 4u, 17u << 24u),
           "cortex_m4_write_memory(sdhc_cpu, 0x400b100cu, 4u, 17u << 24u)");
    expect(state, k22_integration_test_read32(sdhc_device, 0x400b1020u, &response_word),
           "k22_integration_test_read32(sdhc_device, 0x400b1020u, &response_word)");
    expect(state, response_word == 0x03020100u, "response_word == 0x03020100u");

    kinetis_k22_sdhc_eject(sdhc_device);
    expect(state, !kinetis_k22_sdhc_read_card(sdhc_device, 0u, &response_word, 1u),
           "!kinetis_k22_sdhc_read_card(sdhc_device, 0u, &response_word, 1u)");
    kinetis_k22_destroy(sdhc_device);
}

static void expect_endpoint_event_order(TestState* state, KinetisK22* device) {
    CortexM4* cpu = kinetis_k22_cpu(device);
    uint32_t clock_gate_value = 0u;
    expect(state, k22_integration_test_read32(device, SIM_SCGC4, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC4, &clock_gate_value)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC4, 4, clock_gate_value | 0xc0u),
           "cortex_m4_write_memory(cpu, SIM_SCGC4, 4, clock_gate_value | 0xc0u)");
    expect(state, cortex_m4_write_memory(cpu, I2C0_C1, 1, 0xf1u),
           "cortex_m4_write_memory(cpu, I2C0_C1, 1, 0xf1u)");
    expect(state, cortex_m4_write_memory(cpu, I2C1_C1, 1, 0xf1u),
           "cortex_m4_write_memory(cpu, I2C1_C1, 1, 0xf1u)");
    KinetisK22I2cTransfer i2c_transfer;
    expect(state, kinetis_k22_i2c_transfer(device, KINETIS_K22_SERIAL_I2C1, &i2c_transfer),
           "kinetis_k22_i2c_transfer(device, KINETIS_K22_SERIAL_I2C1, &i2c_transfer)");
    expect(state, i2c_transfer.type == KINETIS_K22_I2C_START,
           "i2c_transfer.type == KINETIS_K22_I2C_START");
    expect(state, kinetis_k22_i2c_transfer(device, KINETIS_K22_SERIAL_I2C0, &i2c_transfer),
           "kinetis_k22_i2c_transfer(device, KINETIS_K22_SERIAL_I2C0, &i2c_transfer)");
    expect(state, i2c_transfer.type == KINETIS_K22_I2C_START,
           "i2c_transfer.type == KINETIS_K22_I2C_START");
}

static void expect_pdb_data_triggers(TestState* state, KinetisK22* device) {
    CortexM4* cpu = kinetis_k22_cpu(device);
    uint32_t clock_gate_value = 0u;
    expect(state, k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value)");
    expect(
        state,
        cortex_m4_write_memory(cpu, SIM_SCGC6, 4,
                               clock_gate_value | (1u << 22u) | (1u << 27u) | (1u << 31u)),
        "cortex_m4_write_memory(cpu, SIM_SCGC6, 4, clock_gate_value | (1u << 22u) | (1u << 27u) | "
        "(1u << 31u))");
    expect(state, kinetis_k22_set_adc_channel(device, 0, 5, 0x345u),
           "kinetis_k22_set_adc_channel(device, 0, 5, 0x345u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_CFG1, 0x04u),
           "k22_integration_test_cpu_write8(device, ADC0_CFG1, 0x04u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u),
           "k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC2, 0x40u),
           "k22_integration_test_cpu_write8(device, ADC0_SC2, 0x40u)");

    expect(state, k22_integration_test_cpu_write8(device, DAC0_DAT0L, 0x11u),
           "k22_integration_test_cpu_write8(device, DAC0_DAT0L, 0x11u)");
    expect(state, k22_integration_test_cpu_write8(device, DAC0_DAT0L + 1u, 1u),
           "k22_integration_test_cpu_write8(device, DAC0_DAT0L + 1u, 1u)");
    expect(state, k22_integration_test_cpu_write8(device, DAC0_DAT1L, 0x22u),
           "k22_integration_test_cpu_write8(device, DAC0_DAT1L, 0x22u)");
    expect(state, k22_integration_test_cpu_write8(device, DAC0_DAT1L + 1u, 2u),
           "k22_integration_test_cpu_write8(device, DAC0_DAT1L + 1u, 2u)");
    expect(state, k22_integration_test_cpu_write8(device, DAC0_C0, 0x80u),
           "k22_integration_test_cpu_write8(device, DAC0_C0, 0x80u)");
    expect(state, k22_integration_test_cpu_write8(device, DAC0_C1, 0x80u),
           "k22_integration_test_cpu_write8(device, DAC0_C1, 0x80u)");
    expect(state, k22_integration_test_cpu_write8(device, DAC0_C2, 1u),
           "k22_integration_test_cpu_write8(device, DAC0_C2, 1u)");

    expect(state, cortex_m4_write_memory(cpu, PDB_MOD, 4, 31u),
           "cortex_m4_write_memory(cpu, PDB_MOD, 4, 31u)");
    expect(state, cortex_m4_write_memory(cpu, PDB_CH0C1, 4, 1u),
           "cortex_m4_write_memory(cpu, PDB_CH0C1, 4, 1u)");
    expect(state, cortex_m4_write_memory(cpu, PDB_CH0DLY0, 4, 1u),
           "cortex_m4_write_memory(cpu, PDB_CH0DLY0, 4, 1u)");
    expect(state, cortex_m4_write_memory(cpu, PDB_DACINT0, 4, 1u),
           "cortex_m4_write_memory(cpu, PDB_DACINT0, 4, 1u)");
    expect(state, cortex_m4_write_memory(cpu, PDB_DACINTC0, 4, 1u),
           "cortex_m4_write_memory(cpu, PDB_DACINTC0, 4, 1u)");
    expect(state, cortex_m4_write_memory(cpu, PDB_SC, 4, 3u),
           "cortex_m4_write_memory(cpu, PDB_SC, 4, 3u)");
    kinetis_k22_advance(device, 20u);

    uint16_t adc_result = 0u;
    uint16_t dac_result = 0u;
    expect(state, k22_integration_test_read16(device, ADC0_RA, &adc_result),
           "k22_integration_test_read16(device, ADC0_RA, &adc_result)");
    expect(state, adc_result == 0x345u, "adc_result == 0x345u");
    expect(state, kinetis_k22_get_dac_output(device, 0, &dac_result),
           "kinetis_k22_get_dac_output(device, 0, &dac_result)");
    expect(state, dac_result == 0x222u, "dac_result == 0x222u");
}

static void expect_adc_alternate_triggers(TestState* state) {
    KinetisK22* device =
        k22_integration_test_create_device(state, KINETIS_K22_PACKAGE_DC_121_XFBGA);
    CortexM4* cpu = kinetis_k22_cpu(device);
    uint32_t clock_gate_value = 0u;
    expect(state, k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value)");
    expect(state,
           cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, clock_gate_value | (1u << 23u) | (1u << 27u)),
           "cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, clock_gate_value | (1u << 23u) | "
           "(1u << 27u))");
    expect(state, kinetis_k22_set_adc_channel(device, 0u, 5u, 0x345u),
           "kinetis_k22_set_adc_channel(device, 0u, 5u, 0x345u)");
    expect(state, kinetis_k22_set_adc_channel(device, 0u, 6u, 0x456u),
           "kinetis_k22_set_adc_channel(device, 0u, 6u, 0x456u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_CFG1, 0x04u),
           "k22_integration_test_cpu_write8(device, ADC0_CFG1, 0x04u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC2, 0x40u),
           "k22_integration_test_cpu_write8(device, ADC0_SC2, 0x40u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u),
           "k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC1B, 6u),
           "k22_integration_test_cpu_write8(device, ADC0_SC1B, 6u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_MCR, 4u, 0u),
           "cortex_m4_write_memory(cpu, PIT_MCR, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_LDVAL0, 4u, 0u),
           "cortex_m4_write_memory(cpu, PIT_LDVAL0, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x85u),
           "cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x85u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 1u),
           "cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 1u)");
    kinetis_k22_advance(device, 20u);
    uint8_t status = 0u;
    expect(state, k22_integration_test_read8(device, ADC0_SC1A, &status),
           "k22_integration_test_read8(device, ADC0_SC1A, &status)");
    expect(state, (status & 0x80u) == 0u, "(status & 0x80u) == 0u");

    expect(state, cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x84u),
           "cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x84u)");
    kinetis_k22_advance(device, 20u);
    expect(state, k22_integration_test_read8(device, ADC0_SC1A, &status),
           "k22_integration_test_read8(device, ADC0_SC1A, &status)");
    expect(state, (status & 0x80u) != 0u, "(status & 0x80u) != 0u");
    uint16_t result = 0u;
    expect(state, k22_integration_test_read16(device, ADC0_RA, &result),
           "k22_integration_test_read16(device, ADC0_RA, &result)");
    expect(state, result == 0x345u, "result == 0x345u");

    expect(state, cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x94u),
           "cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x94u)");
    kinetis_k22_advance(device, 20u);
    expect(state, k22_integration_test_read8(device, ADC0_SC1B, &status),
           "k22_integration_test_read8(device, ADC0_SC1B, &status)");
    expect(state, (status & 0x80u) != 0u, "(status & 0x80u) != 0u");
    expect(state, k22_integration_test_read16(device, ADC0_RB, &result),
           "k22_integration_test_read16(device, ADC0_RB, &result)");
    expect(state, result == 0x456u, "result == 0x456u");
    expect(state, cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 0u),
           "cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_TFLG0, 4u, 1u),
           "cortex_m4_write_memory(cpu, PIT_TFLG0, 4u, 1u)");

    uint32_t data_gates = 0u;
    expect(state, k22_integration_test_read32(device, SIM_SCGC4, &data_gates),
           "k22_integration_test_read32(device, SIM_SCGC4, &data_gates)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC4, 4u, data_gates | (1u << 19u)),
           "cortex_m4_write_memory(cpu, SIM_SCGC4, 4u, data_gates | (1u << 19u))");
    expect(state, cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x81u),
           "cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x81u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u),
           "k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u)");
    expect(state, kinetis_k22_set_cmp_input(device, 0u, 1u, 10u),
           "kinetis_k22_set_cmp_input(device, 0u, 1u, 10u)");
    expect(state, kinetis_k22_set_cmp_input(device, 0u, 2u, 20u),
           "kinetis_k22_set_cmp_input(device, 0u, 2u, 20u)");
    expect(state, k22_integration_test_cpu_write8(device, CMP0_MUXCR, 0x0au),
           "k22_integration_test_cpu_write8(device, CMP0_MUXCR, 0x0au)");
    expect(state, k22_integration_test_cpu_write8(device, CMP0_CR1, 1u),
           "k22_integration_test_cpu_write8(device, CMP0_CR1, 1u)");
    expect(state, kinetis_k22_set_cmp_input(device, 0u, 1u, 30u),
           "kinetis_k22_set_cmp_input(device, 0u, 1u, 30u)");
    kinetis_k22_advance(device, 20u);
    expect(state, k22_integration_test_read16(device, ADC0_RA, &result),
           "k22_integration_test_read16(device, ADC0_RA, &result)");
    expect(state, result == 0x345u, "result == 0x345u");

    expect(state, k22_integration_test_read32(device, SIM_SCGC5, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC5, &clock_gate_value)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC5, 4u, clock_gate_value | 1u),
           "cortex_m4_write_memory(cpu, SIM_SCGC5, 4u, clock_gate_value | 1u)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x8eu),
           "cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x8eu)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC1A, 6u),
           "k22_integration_test_cpu_write8(device, ADC0_SC1A, 6u)");
    expect(state, kinetis_k22_set_lptmr_input(device, 0u, false),
           "kinetis_k22_set_lptmr_input(device, 0u, false)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_PSR, 4u, 4u),
           "cortex_m4_write_memory(cpu, LPTMR_PSR, 4u, 4u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_CMR, 4u, 0u),
           "cortex_m4_write_memory(cpu, LPTMR_CMR, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 3u),
           "cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 3u)");
    expect(state, kinetis_k22_set_lptmr_input(device, 0u, true),
           "kinetis_k22_set_lptmr_input(device, 0u, true)");
    kinetis_k22_advance(device, 20u);
    expect(state, k22_integration_test_read16(device, ADC0_RA, &result),
           "k22_integration_test_read16(device, ADC0_RA, &result)");
    expect(state, result == 0x456u, "result == 0x456u");

    expect(state, k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC6, &clock_gate_value)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, clock_gate_value | (1u << 24u)),
           "cortex_m4_write_memory(cpu, SIM_SCGC6, 4u, clock_gate_value | (1u << 24u))");
    expect(state, cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x88u),
           "cortex_m4_write_memory(cpu, SIM_SOPT7, 4u, 0x88u)");
    expect(state, k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u),
           "k22_integration_test_cpu_write8(device, ADC0_SC1A, 5u)");
    expect(state, cortex_m4_write_memory(cpu, FTM0_MOD, 4u, 3u),
           "cortex_m4_write_memory(cpu, FTM0_MOD, 4u, 3u)");
    expect(state, cortex_m4_write_memory(cpu, FTM0_C0V, 4u, 2u),
           "cortex_m4_write_memory(cpu, FTM0_C0V, 4u, 2u)");
    expect(state, cortex_m4_write_memory(cpu, FTM0_C0SC, 4u, 0x10u),
           "cortex_m4_write_memory(cpu, FTM0_C0SC, 4u, 0x10u)");
    expect(state, cortex_m4_write_memory(cpu, FTM0_EXTTRIG, 4u, 0x10u),
           "cortex_m4_write_memory(cpu, FTM0_EXTTRIG, 4u, 0x10u)");
    expect(state, cortex_m4_write_memory(cpu, FTM0_SC, 4u, 0x08u),
           "cortex_m4_write_memory(cpu, FTM0_SC, 4u, 0x08u)");
    kinetis_k22_advance(device, 20u);
    expect(state, k22_integration_test_read16(device, ADC0_RA, &result),
           "k22_integration_test_read16(device, ADC0_RA, &result)");
    expect(state, result == 0x345u, "result == 0x345u");

    expect(state, cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 0u),
           "cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_LDVAL0, 4u, 5u),
           "cortex_m4_write_memory(cpu, PIT_LDVAL0, 4u, 5u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_MCR, 4u, 1u),
           "cortex_m4_write_memory(cpu, PIT_MCR, 4u, 1u)");
    expect(state, cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 1u),
           "cortex_m4_write_memory(cpu, PIT_TCTRL0, 4u, 1u)");
    kinetis_k22_advance(device, 2u);
    uint32_t counter = 0u;
    expect(state, k22_integration_test_read32(device, PIT_LDVAL0 + 4u, &counter),
           "k22_integration_test_read32(device, PIT_LDVAL0 + 4u, &counter)");
    expect(state, counter == 3u, "counter == 3u");
    expect(state, cortex_m4_write_memory(cpu, (uint32_t)DHCSR, 4u, 0xa05f0003u),
           "cortex_m4_write_memory(cpu, (uint32_t)DHCSR, 4u, 0xa05f0003u)");
    kinetis_k22_advance(device, 10u);
    expect(state, k22_integration_test_read32(device, PIT_LDVAL0 + 4u, &counter),
           "k22_integration_test_read32(device, PIT_LDVAL0 + 4u, &counter)");
    expect(state, counter == 3u, "counter == 3u");
    expect(state, cortex_m4_write_memory(cpu, (uint32_t)DHCSR, 4u, 0xa05f0001u),
           "cortex_m4_write_memory(cpu, (uint32_t)DHCSR, 4u, 0xa05f0001u)");
    kinetis_k22_advance(device, 1u);
    expect(state, k22_integration_test_read32(device, PIT_LDVAL0 + 4u, &counter),
           "k22_integration_test_read32(device, PIT_LDVAL0 + 4u, &counter)");
    expect(state, counter == 2u, "counter == 2u");
    kinetis_k22_destroy(device);
}

static void expect_lptmr_pulse_input(TestState* state, KinetisK22* device) {
    CortexM4* cpu = kinetis_k22_cpu(device);
    uint32_t clock_gate_value = 0u;
    expect(state, k22_integration_test_read32(device, SIM_SCGC5, &clock_gate_value),
           "k22_integration_test_read32(device, SIM_SCGC5, &clock_gate_value)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC5, 4u, clock_gate_value | 1u),
           "cortex_m4_write_memory(cpu, SIM_SCGC5, 4u, clock_gate_value | 1u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_PSR, 4u, 4u),
           "cortex_m4_write_memory(cpu, LPTMR_PSR, 4u, 4u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_CMR, 4u, 0u),
           "cortex_m4_write_memory(cpu, LPTMR_CMR, 4u, 0u)");
    expect(state, kinetis_k22_set_lptmr_input(device, 1u, false),
           "kinetis_k22_set_lptmr_input(device, 1u, false)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 0x53u),
           "cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 0x53u)");
    expect(state, kinetis_k22_set_lptmr_input(device, 0u, true),
           "kinetis_k22_set_lptmr_input(device, 0u, true)");
    expect(state, kinetis_k22_set_lptmr_input(device, 1u, false),
           "kinetis_k22_set_lptmr_input(device, 1u, false)");
    expect(state, kinetis_k22_set_lptmr_input(device, 1u, true),
           "kinetis_k22_set_lptmr_input(device, 1u, true)");
    uint32_t read_value = 0u;
    expect(state, k22_integration_test_read32(device, LPTMR_CSR, &read_value),
           "k22_integration_test_read32(device, LPTMR_CSR, &read_value)");
    expect(state, read_value == 0xd3u, "read_value == 0xd3u");
    expect(state, cortex_m4_get_irq_pending(cpu, 58u), "cortex_m4_get_irq_pending(cpu, 58u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_CNR, 4u, 0u),
           "cortex_m4_write_memory(cpu, LPTMR_CNR, 4u, 0u)");
    expect(state, k22_integration_test_read32(device, LPTMR_CNR, &read_value),
           "k22_integration_test_read32(device, LPTMR_CNR, &read_value)");
    expect(state, read_value == 0u, "read_value == 0u");
    expect(state, !kinetis_k22_set_lptmr_input(device, 3u, false),
           "!kinetis_k22_set_lptmr_input(device, 3u, false)");

    expect(state, cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 0u),
           "cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 0u)");
    uint32_t data_gates = 0u;
    expect(state, k22_integration_test_read32(device, SIM_SCGC4, &data_gates),
           "k22_integration_test_read32(device, SIM_SCGC4, &data_gates)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC4, 4u, data_gates | (1u << 19u)),
           "cortex_m4_write_memory(cpu, SIM_SCGC4, 4u, data_gates | (1u << 19u))");
    expect(state, kinetis_k22_set_cmp_input(device, 0u, 1u, 10u),
           "kinetis_k22_set_cmp_input(device, 0u, 1u, 10u)");
    expect(state, kinetis_k22_set_cmp_input(device, 0u, 2u, 20u),
           "kinetis_k22_set_cmp_input(device, 0u, 2u, 20u)");
    expect(state, k22_integration_test_cpu_write8(device, CMP0_MUXCR, 0x0au),
           "k22_integration_test_cpu_write8(device, CMP0_MUXCR, 0x0au)");
    expect(state, k22_integration_test_cpu_write8(device, CMP0_CR1, 1u),
           "k22_integration_test_cpu_write8(device, CMP0_CR1, 1u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_PSR, 4u, 4u),
           "cortex_m4_write_memory(cpu, LPTMR_PSR, 4u, 4u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_CMR, 4u, 0u),
           "cortex_m4_write_memory(cpu, LPTMR_CMR, 4u, 0u)");
    expect(state, cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 0x43u),
           "cortex_m4_write_memory(cpu, LPTMR_CSR, 4u, 0x43u)");
    expect(state, kinetis_k22_set_cmp_input(device, 0u, 2u, 20u),
           "kinetis_k22_set_cmp_input(device, 0u, 2u, 20u)");
    expect(state, kinetis_k22_set_cmp_input(device, 0u, 1u, 30u),
           "kinetis_k22_set_cmp_input(device, 0u, 1u, 30u)");
    expect(state, k22_integration_test_read32(device, LPTMR_CSR, &read_value),
           "k22_integration_test_read32(device, LPTMR_CSR, &read_value)");
    expect(state, read_value == 0xc3u, "read_value == 0xc3u");
}

static void expect_flexbus_integration(TestState* state, KinetisK22* device) {
    uint8_t window_data[256];
    for (size_t byte_index = 0u; byte_index < sizeof(window_data); byte_index++) {
        window_data[byte_index] = (uint8_t)(0x80u + byte_index);
    }
    expect(state,
           kinetis_k22_flexbus_attach(device, 0x60000000u, window_data, sizeof(window_data), false),
           "kinetis_k22_flexbus_attach(device, 0x60000000u, window_data, sizeof(window_data), "
           "false)");
    CortexM4* cpu = kinetis_k22_cpu(device);
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC7, 4u, 1u),
           "cortex_m4_write_memory(cpu, SIM_SCGC7, 4u, 1u)");
    expect(state, cortex_m4_write_memory(cpu, 0x4000c000u, 4u, 0x60000000u),
           "cortex_m4_write_memory(cpu, 0x4000c000u, 4u, 0x60000000u)");
    expect(state, cortex_m4_write_memory(cpu, 0x4000c004u, 4u, 1u),
           "cortex_m4_write_memory(cpu, 0x4000c004u, 4u, 1u)");
    uint32_t read_value = 0u;
    expect(state, cortex_m4_read_memory(cpu, 0x60000004u, 4u, &read_value),
           "cortex_m4_read_memory(cpu, 0x60000004u, 4u, &read_value)");
    expect(state, read_value == 0x87868584u, "read_value == 0x87868584u");
    expect(state, cortex_m4_write_memory(cpu, 0x60000008u, 4u, 0x12345678u),
           "cortex_m4_write_memory(cpu, 0x60000008u, 4u, 0x12345678u)");
    expect(state, kinetis_k22_flexbus_read(device, 8u, &read_value, sizeof(read_value)),
           "kinetis_k22_flexbus_read(device, 8u, &read_value, sizeof(read_value))");
    expect(state, read_value == 0x12345678u, "read_value == 0x12345678u");
    KinetisK22Event event;
    bool read_event_seen = false;
    bool write_event_seen = false;
    while (kinetis_k22_next_event(device, &event)) {
        if (event.type == KINETIS_K22_EVENT_FLEXBUS_TRANSFER) {
            read_event_seen |= event.auxiliary == 4u;
            write_event_seen |= event.auxiliary == 0x104u && event.value == 0x12345678u;
        }
    }
    expect(state, read_event_seen, "read_event_seen");
    expect(state, write_event_seen, "write_event_seen");
}

static void expect_reset_domains(TestState* state, KinetisK22* device) {
    const uint32_t test_address = 0x20000040u;
    const uint32_t sentinel = 0x5aa53cc3u;
    expect(state, k22_integration_test_write32(device, test_address, sentinel),
           "k22_integration_test_write32(device, test_address, sentinel)");
    kinetis_k22_gpio_drive(device, 0, 0, true);
    expect(state, k22_integration_test_write16(device, WDOG_UNLOCK, 0xc520u),
           "k22_integration_test_write16(device, WDOG_UNLOCK, 0xc520u)");
    expect(state, k22_integration_test_write16(device, WDOG_UNLOCK, 0xd928u),
           "k22_integration_test_write16(device, WDOG_UNLOCK, 0xd928u)");
    expect(state, k22_integration_test_write16(device, WDOG_TOVALH, 0),
           "k22_integration_test_write16(device, WDOG_TOVALH, 0)");
    expect(state, k22_integration_test_write16(device, WDOG_TOVALL, 1),
           "k22_integration_test_write16(device, WDOG_TOVALL, 1)");
    expect(state, k22_integration_test_write16(device, WDOG_STCTRLH, 1),
           "k22_integration_test_write16(device, WDOG_STCTRLH, 1)");
    kinetis_k22_advance(device, k22_test_core_cycles_for_bus_cycles(device, 260u));
    kinetis_k22_watchdog_advance(device, 1);

    uint32_t read_value = 0u;
    uint8_t cause = 0;
    expect(state, k22_integration_test_read32(device, test_address, &read_value),
           "k22_integration_test_read32(device, test_address, &read_value)");
    expect(state, read_value == sentinel, "read_value == sentinel");
    expect(state, k22_integration_test_read8(device, RCM_SRS0, &cause),
           "k22_integration_test_read8(device, RCM_SRS0, &cause)");
    expect(state, cause == 0x20u, "cause == 0x20u");
    expect(state, k22_integration_test_read8(device, RCM_SSRS0, &cause),
           "k22_integration_test_read8(device, RCM_SSRS0, &cause)");
    expect(state, (cause & 0xa2u) == 0xa2u, "(cause & 0xa2u) == 0xa2u");
    expect(state, k22_integration_test_read32(device, GPIOA_PDIR, &read_value),
           "k22_integration_test_read32(device, GPIOA_PDIR, &read_value)");
    expect(state, (read_value & 1u) != 0, "(read_value & 1u) != 0");

    expect(state, k22_integration_test_write16(device, WDOG_UNLOCK, 0xc520u),
           "k22_integration_test_write16(device, WDOG_UNLOCK, 0xc520u)");
    expect(state, k22_integration_test_write16(device, WDOG_UNLOCK, 0xd928u),
           "k22_integration_test_write16(device, WDOG_UNLOCK, 0xd928u)");
    expect(state, k22_integration_test_write16(device, WDOG_TOVALH, 0u),
           "k22_integration_test_write16(device, WDOG_TOVALH, 0u)");
    expect(state, k22_integration_test_write16(device, WDOG_TOVALL, 1u),
           "k22_integration_test_write16(device, WDOG_TOVALL, 1u)");
    expect(state, k22_integration_test_write16(device, WDOG_STCTRLH, 5u),
           "k22_integration_test_write16(device, WDOG_STCTRLH, 5u)");
    kinetis_k22_advance(device, k22_test_core_cycles_for_bus_cycles(device, 260u));
    kinetis_k22_watchdog_advance(device, 1u);
    expect(state, cortex_m4_get_irq_pending(kinetis_k22_cpu(device), 22u),
           "cortex_m4_get_irq_pending(kinetis_k22_cpu(device), 22u)");
}

static void expect_copy(TestState* state, KinetisK22* source_device) {
    KinetisK22Configuration configuration = kinetis_k22_default_configuration();
    configuration.package = KINETIS_K22_PACKAGE_DC_121_XFBGA;
    KinetisK22* destination_device = kinetis_k22_create(configuration);
    expect(state, destination_device != NULL, "destination_device != NULL");
    WaitFixture source_wait = {0};
    WaitFixture destination_wait = {0};
    cortex_m4_set_wait_states(kinetis_k22_cpu(source_device), wait_states, &source_wait);
    cortex_m4_set_wait_states(kinetis_k22_cpu(destination_device), wait_states, &destination_wait);
    expect(state, kinetis_k22_copy(destination_device, source_device),
           "kinetis_k22_copy(destination_device, source_device)");
    uint32_t read_value = 0u;
    expect(state, k22_integration_test_read32(destination_device, 0x20000040u, &read_value),
           "k22_integration_test_read32(destination_device, 0x20000040u, &read_value)");
    expect(state, read_value == 0x5aa53cc3u, "read_value == 0x5aa53cc3u");
    expect(state,
           kinetis_k22_core_clock_hz(destination_device) ==
               kinetis_k22_core_clock_hz(source_device),
           "kinetis_k22_core_clock_hz(destination_device) == "
           "kinetis_k22_core_clock_hz(source_device)");
    expect(
        state,
        kinetis_k22_bus_clock_hz(destination_device) == kinetis_k22_bus_clock_hz(source_device),
        "kinetis_k22_bus_clock_hz(destination_device) == kinetis_k22_bus_clock_hz(source_device)");
    test_connect_debugger(state, kinetis_k22_cpu(destination_device));
    expect(state,
           cortex_m4_step(kinetis_k22_cpu(destination_device)).stop == CORTEX_M4_STOP_BREAKPOINT,
           "cortex_m4_step(kinetis_k22_cpu(destination_device)).stop == CORTEX_M4_STOP_BREAKPOINT");
    expect(state, destination_wait.call_count != 0u, "destination_wait.call_count != 0u");
    expect(state, source_wait.call_count == 0u, "source_wait.call_count == 0u");
    kinetis_k22_destroy(destination_device);
}

int main(void) {
    TestState state = {0};
    k22_integration_test_expect_package_selection(&state);
    k22_integration_test_expect_package_serial_extensions(&state);
    expect_serial_dma_sources(&state);
    expect_periodic_dma_trigger(&state);
    k22_integration_test_expect_can_irq_level(&state);
    expect_sdhc_integration(&state);
    k22_integration_test_expect_fmc_cache(&state);
    k22_integration_test_expect_fmc_invalidation_and_locking(&state);
    k22_integration_test_expect_integrated_flash_swap(&state);
    KinetisK22* device =
        k22_integration_test_create_device(&state, KINETIS_K22_PACKAGE_DC_121_XFBGA);
    expect(&state, kinetis_k22_reset(device), "kinetis_k22_reset(device)");
    expect(&state, kinetis_k22_core_clock_hz(device) == 20971520u,
           "kinetis_k22_core_clock_hz(device) == 20971520u");
    expect(&state, kinetis_k22_bus_clock_hz(device) == 20971520u,
           "kinetis_k22_bus_clock_hz(device) == 20971520u");
    k22_integration_test_expect_manifest_fallback(&state, device);
    k22_integration_test_expect_integrated_flash_command(&state, device);
    k22_integration_test_expect_io_irq_levels(&state, device);
    k22_integration_test_expect_clock_gates(&state, device);
    expect_endpoint_event_order(&state, device);
    expect_pdb_data_triggers(&state, device);
    expect_adc_alternate_triggers(&state);
    expect_lptmr_pulse_input(&state, device);
    expect_flexbus_integration(&state, device);
    expect_reset_domains(&state, device);
    expect_copy(&state, device);
    KinetisK22Event event;
    while (kinetis_k22_next_event(device, &event)) {
    }
    expect(&state, !kinetis_k22_next_event(device, &event),
           "!kinetis_k22_next_event(device, &event)");
    kinetis_k22_destroy(device);
    k22_integration_test_expect_memory_domains(&state);
    return test_finish(&state);
}
