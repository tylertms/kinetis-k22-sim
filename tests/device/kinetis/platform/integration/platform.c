#include "device/kinetis/platform/integration/internal.h"

bool kinetis_integration_test_read8(Kinetis* device, uint32_t address, uint8_t* read_value) {
    return kinetis_read(device, address, read_value, sizeof(*read_value));
}

bool kinetis_integration_test_read32(Kinetis* device, uint32_t address, uint32_t* read_value) {
    return kinetis_read(device, address, read_value, sizeof(*read_value));
}

bool kinetis_integration_test_write16(Kinetis* device, uint32_t address, uint16_t write_value) {
    return kinetis_write(device, address, &write_value, sizeof(write_value));
}

static bool write8(Kinetis* device, uint32_t address, uint8_t write_value) {
    return kinetis_write(device, address, &write_value, sizeof(write_value));
}

bool kinetis_integration_test_read16(Kinetis* device, uint32_t address, uint16_t* read_value) {
    return kinetis_read(device, address, read_value, sizeof(*read_value));
}

bool kinetis_integration_test_write32(Kinetis* device, uint32_t address, uint32_t write_value) {
    return kinetis_write(device, address, &write_value, sizeof(write_value));
}

bool kinetis_integration_test_cpu_write8(Kinetis* device, uint32_t address, uint8_t write_value) {
    return cortex_m4_write_memory(kinetis_cpu(device), address, 1u, write_value);
}

static bool irq_level(const Kinetis* device, uint8_t interrupt_number) {
    const CortexM4* cpu = kinetis_cpu_const(device);
    return (cpu->irq_level[interrupt_number / 32u] & (1u << (interrupt_number & 31u))) != 0u;
}

static uint32_t flash_fccob_address(uint8_t byte_index) {
    static const uint8_t offsets[12] = {7u, 6u, 5u, 4u, 11u, 10u, 9u, 8u, 15u, 14u, 13u, 12u};
    return FTFA_FCCOB3 - 4u + offsets[byte_index];
}

Kinetis* kinetis_integration_test_create_device(TestState* state, KinetisPackage package) {
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    configuration.package = package;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "device != NULL");
    const uint32_t vectors[2] = {0x20001000u, 0x00000101u};
    const uint16_t program = 0xbe00u;
    expect(state, kinetis_load(device, 0, vectors, sizeof(vectors)),
           "kinetis_load(device, 0, vectors, sizeof(vectors))");
    expect(state, kinetis_load(device, 0x100, &program, sizeof(program)),
           "kinetis_load(device, 0x100, &program, sizeof(program))");
    return device;
}

Kinetis* kinetis_integration_test_create_f12_device(TestState* state, KinetisPackage package) {
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    configuration.profile = KINETIS_PROFILE_MK22FN1M012;
    configuration.package = package;
    configuration.flash_size = 1024u * 1024u;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "device != NULL");
    return device;
}

void kinetis_integration_test_expect_package_selection(TestState* state) {
    KinetisConfiguration invalid_profile = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    invalid_profile.profile = KINETIS_PROFILE_COUNT;
    expect(state, kinetis_create(invalid_profile) == NULL,
           "kinetis_create(invalid_profile) == NULL");

    KinetisConfiguration invalid = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    invalid.package = KINETIS_PACKAGE_AH_64_WLCSP;
    expect(state, kinetis_create(invalid) == NULL, "kinetis_create(invalid) == NULL");

    Kinetis* small_device =
        kinetis_integration_test_create_device(state, KINETIS_PACKAGE_LH_64_LQFP);
    uint32_t device_id = 0u;
    expect(state, kinetis_integration_test_read32(small_device, SIM_SDID, &device_id),
           "kinetis_integration_test_read32(small_device, SIM_SDID, &device_id)");
    expect(state, (device_id & 15u) == 5u, "64-pin package reports PINID 5");
    uint16_t dac_value = 0u;
    uint8_t register_value = 0;
    expect(state, !kinetis_read(small_device, DAC1_DAT0L, &register_value, sizeof(register_value)),
           "!kinetis_read(small_device, DAC1_DAT0L, &register_value, sizeof(register_value))");
    expect(state, !kinetis_get_dac_output(small_device, 1, &dac_value),
           "!kinetis_get_dac_output(small_device, 1, &dac_value)");
    kinetis_gpio_drive(small_device, 4, 31, true);
    uint32_t pin_input = UINT32_MAX;
    expect(state,
           kinetis_integration_test_read32(small_device, GPIOA_PDIR + 4u * 0x40u, &pin_input),
           "kinetis_integration_test_read32(small_device, GPIOA_PDIR + 4u * 0x40u, &pin_input)");
    expect(state, pin_input == 0, "pin_input == 0");
    kinetis_destroy(small_device);

    Kinetis* large_device =
        kinetis_integration_test_create_device(state, KINETIS_PACKAGE_DC_121_XFBGA);
    expect(state, kinetis_integration_test_read32(large_device, SIM_SDID, &device_id),
           "kinetis_integration_test_read32(large_device, SIM_SDID, &device_id)");
    expect(state, (device_id & 15u) == 9u, "121-pin package reports PINID 9");
    expect(state, kinetis_read(large_device, DAC1_DAT0L, &register_value, sizeof(register_value)),
           "kinetis_read(large_device, DAC1_DAT0L, &register_value, sizeof(register_value))");
    kinetis_destroy(large_device);

    Kinetis* limited_device =
        kinetis_integration_test_create_device(state, KINETIS_PACKAGE_FX_88_HVQFN);
    expect(state,
           !kinetis_read(limited_device, DAC1_DAT0L, &register_value, sizeof(register_value)),
           "!kinetis_read(limited_device, DAC1_DAT0L, &register_value, sizeof(register_value))");
    expect(state, !kinetis_get_dac_output(limited_device, 1, &dac_value),
           "!kinetis_get_dac_output(limited_device, 1, &dac_value)");
    kinetis_destroy(limited_device);
}

void kinetis_integration_test_expect_integrated_flash_command(TestState* state, Kinetis* device) {
    const uint32_t target_address = 0x00001000u;
    uint32_t read_value = 0u;
    expect(state, kinetis_integration_test_read32(device, target_address, &read_value),
           "kinetis_integration_test_read32(device, target_address, &read_value)");
    expect(state, read_value == UINT32_MAX, "read_value == UINT32_MAX");
    const uint8_t command[8] = {0x06u, 0x00u, 0x10u, 0x00u, 0x12u, 0x34u, 0x56u, 0x78u};
    for (uint8_t command_index = 0u; command_index < sizeof(command); command_index++)
        expect(state,
               kinetis_integration_test_cpu_write8(device, flash_fccob_address(command_index),
                                                   command[command_index]),
               "kinetis_integration_test_cpu_write8(device, flash_fccob_address(command_index), "
               "command[command_index])");
    expect(state, kinetis_integration_test_cpu_write8(device, FTFA_FSTAT, 0x80u),
           "kinetis_integration_test_cpu_write8(device, FTFA_FSTAT, 0x80u)");
    expect(state, kinetis_integration_test_read32(device, target_address, &read_value),
           "kinetis_integration_test_read32(device, target_address, &read_value)");
    expect(state, read_value == UINT32_MAX, "read_value == UINT32_MAX");
    const uint32_t fstat_bit_band = 0x42000000u + (FTFA_FSTAT - 0x40000000u) * 32u + 6u * 4u;
    expect(state, kinetis_integration_test_write32(device, fstat_bit_band, 1u),
           "kinetis_integration_test_write32(device, fstat_bit_band, 1u)");
    kinetis_advance(device, 40u);
    expect(state, kinetis_integration_test_read32(device, target_address, &read_value),
           "kinetis_integration_test_read32(device, target_address, &read_value)");
    expect(state, read_value == 0x12345678u, "read_value == 0x12345678u");
}

static void launch_swap_command(TestState* state, Kinetis* device, uint32_t address,
                                uint8_t control) {
    const uint8_t command[5] = {0x46u, (uint8_t)(address >> 16u), (uint8_t)(address >> 8u),
                                (uint8_t)address, control};
    for (uint8_t command_index = 0u; command_index < sizeof(command); command_index++) {
        expect(state,
               kinetis_integration_test_cpu_write8(device, flash_fccob_address(command_index),
                                                   command[command_index]),
               "kinetis_integration_test_cpu_write8(device, flash_fccob_address(command_index), "
               "command[command_index])");
    }
    expect(state, kinetis_integration_test_cpu_write8(device, FTFA_FSTAT, 0x80u),
           "kinetis_integration_test_cpu_write8(device, FTFA_FSTAT, 0x80u)");
    kinetis_advance(device, 40u);
}

void kinetis_integration_test_expect_integrated_flash_swap(TestState* state) {
    Kinetis* device =
        kinetis_integration_test_create_f12_device(state, KINETIS_PACKAGE_MC_121_MAPBGA);
    uint8_t lower_bank_byte = 0x11u;
    uint8_t upper_bank_byte = 0x22u;
    expect(state, kinetis_write(device, 0x20u, &lower_bank_byte, sizeof(lower_bank_byte)),
           "kinetis_write(device, 0x20u, &lower_bank_byte, sizeof(lower_bank_byte))");
    expect(state, kinetis_write(device, 0x80020u, &upper_bank_byte, sizeof(upper_bank_byte)),
           "kinetis_write(device, 0x80020u, &upper_bank_byte, sizeof(upper_bank_byte))");
    launch_swap_command(state, device, 0x1000u, 1u);
    launch_swap_command(state, device, 0x1000u, 4u);
    expect(state, kinetis_reset(device), "kinetis_reset(device)");
    uint8_t read_byte = 0u;
    expect(state, kinetis_integration_test_read8(device, 0x20u, &read_byte),
           "kinetis_integration_test_read8(device, 0x20u, &read_byte)");
    expect(state, read_byte == upper_bank_byte, "read_byte == upper_bank_byte");
    expect(state, kinetis_integration_test_read8(device, 0x80020u, &read_byte),
           "kinetis_integration_test_read8(device, 0x80020u, &read_byte)");
    expect(state, read_byte == lower_bank_byte, "read_byte == lower_bank_byte");
    expect(state, kinetis_integration_test_read8(device, FTFA_FSTAT + 1u, &read_byte),
           "kinetis_integration_test_read8(device, FTFA_FSTAT + 1u, &read_byte)");
    expect(state, (read_byte & 8u) != 0u, "(read_byte & 8u) != 0u");
    kinetis_destroy(device);
}

void kinetis_integration_test_expect_io_irq_levels(TestState* state, Kinetis* device) {
    expect(state, kinetis_integration_test_write32(device, SIM_SCGC5, 1u << 12u),
           "kinetis_integration_test_write32(device, SIM_SCGC5, 1u << 12u)");
    expect(state, kinetis_integration_test_write32(device, PORTD_PCR0, 9u << 16u),
           "kinetis_integration_test_write32(device, PORTD_PCR0, 9u << 16u)");
    kinetis_gpio_drive(device, 3u, 0u, false);
    kinetis_gpio_drive(device, 3u, 0u, true);
    expect(state, irq_level(device, 62u), "irq_level(device, 62u)");
    expect(state, kinetis_integration_test_write32(device, PORTD_ISFR, 1u),
           "kinetis_integration_test_write32(device, PORTD_ISFR, 1u)");
    expect(state, !irq_level(device, 62u), "!irq_level(device, 62u)");

    expect(state, kinetis_integration_test_write32(device, SIM_SCGC4, 1u << 18u),
           "kinetis_integration_test_write32(device, SIM_SCGC4, 1u << 18u)");
    expect(state, write8(device, USB0_INTEN, 1u << 3u), "write8(device, USB0_INTEN, 1u << 3u)");
    expect(state, write8(device, USB0_ENDPT3, 1u), "write8(device, USB0_ENDPT3, 1u)");
    expect(state, kinetis_usb_token(device, 3u, 0x69u, false),
           "kinetis_usb_token(device, 3u, 0x69u, false)");
    expect(state, irq_level(device, 53u), "irq_level(device, 53u)");
    expect(state, write8(device, USB0_ISTAT, 1u << 3u), "write8(device, USB0_ISTAT, 1u << 3u)");
    expect(state, !irq_level(device, 53u), "!irq_level(device, 53u)");

    expect(state, kinetis_integration_test_write32(device, SIM_SCGC6, 1u << 15u),
           "kinetis_integration_test_write32(device, SIM_SCGC6, 1u << 15u)");
    expect(state, kinetis_integration_test_write32(device, I2S0_TCSR, UINT32_C(0x80000100)),
           "kinetis_integration_test_write32(device, I2S0_TCSR, UINT32_C(0x80000100))");
    expect(state, kinetis_integration_test_write32(device, I2S0_TDR0, 0x12345678u),
           "kinetis_integration_test_write32(device, I2S0_TDR0, 0x12345678u)");
    expect(state, irq_level(device, 28u), "irq_level(device, 28u)");
    expect(state, kinetis_integration_test_write32(device, I2S0_TCSR, UINT32_C(0x80000000)),
           "kinetis_integration_test_write32(device, I2S0_TCSR, UINT32_C(0x80000000))");
    expect(state, !irq_level(device, 28u), "!irq_level(device, 28u)");
    expect(state, kinetis_integration_test_write32(device, I2S0_RCSR, UINT32_C(0x80000100)),
           "kinetis_integration_test_write32(device, I2S0_RCSR, UINT32_C(0x80000100))");
    expect(state, kinetis_i2s_receive(device, 0x87654321u),
           "kinetis_i2s_receive(device, 0x87654321u)");
    expect(state, irq_level(device, 29u), "irq_level(device, 29u)");
    expect(state, kinetis_integration_test_write32(device, I2S0_RCSR, UINT32_C(0x80000000)),
           "kinetis_integration_test_write32(device, I2S0_RCSR, UINT32_C(0x80000000))");
    expect(state, !irq_level(device, 29u), "!irq_level(device, 29u)");
}

void kinetis_integration_test_expect_can_irq_level(TestState* state) {
    Kinetis* device =
        kinetis_integration_test_create_f12_device(state, KINETIS_PACKAGE_MD_144_MAPBGA);
    expect(state, kinetis_integration_test_write32(device, SIM_SCGC6, 1u << 4u),
           "kinetis_integration_test_write32(device, SIM_SCGC6, 1u << 4u)");
    expect(state, kinetis_integration_test_write32(device, CAN0_MCR, 0x0fu),
           "kinetis_integration_test_write32(device, CAN0_MCR, 0x0fu)");
    expect(state, kinetis_integration_test_write32(device, CAN0_CTRL1, 0u),
           "kinetis_integration_test_write32(device, CAN0_CTRL1, 0u)");
    expect(state, kinetis_integration_test_write32(device, CAN0_IMASK1, 1u),
           "kinetis_integration_test_write32(device, CAN0_IMASK1, 1u)");
    expect(state, kinetis_integration_test_write32(device, CAN0_MB0_CS, 4u << 24u),
           "kinetis_integration_test_write32(device, CAN0_MB0_CS, 4u << 24u)");
    const KinetisCanFrame receive_frame = {
        .identifier = 0x123u,
        .length = 8u,
        .data = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u},
    };
    expect(state, kinetis_can_receive(device, &receive_frame),
           "kinetis_can_receive(device, &receive_frame)");
    expect(state, irq_level(device, 75u), "irq_level(device, 75u)");
    expect(state, kinetis_integration_test_write32(device, CAN0_IFLAG1, 1u),
           "kinetis_integration_test_write32(device, CAN0_IFLAG1, 1u)");
    expect(state, !irq_level(device, 75u), "!irq_level(device, 75u)");
    kinetis_destroy(device);
}

void kinetis_integration_test_expect_memory_domains(TestState* state) {
    Kinetis* device =
        kinetis_integration_test_create_f12_device(state, KINETIS_PACKAGE_LQ_144_LQFP);
    CortexM4* cpu = kinetis_cpu(device);
    const CortexM4* constant_cpu = kinetis_cpu_const(device);
    expect(state, constant_cpu == cpu, "constant_cpu == cpu");
    expect(state, kinetis_cpu_const(NULL) == NULL, "kinetis_cpu_const(NULL) == NULL");

    const uint32_t flexram = 0x14000000u;
    uint32_t value = 0x12345678u;
    expect(state, kinetis_write(device, flexram, &value, sizeof(value)),
           "kinetis_write(device, flexram, &value, sizeof(value))");
    value = 0u;
    expect(state, cortex_m4_read_memory(cpu, flexram, 4u, &value),
           "cortex_m4_read_memory(cpu, flexram, 4u, &value)");
    expect(state, value == 0x12345678u, "value == 0x12345678u");
    expect(state, cortex_m4_write_memory(cpu, flexram + 4u, 4u, 0xa5a55a5au),
           "cortex_m4_write_memory(cpu, flexram + 4u, 4u, 0xa5a55a5au)");
    expect(state, kinetis_integration_test_read32(device, flexram + 4u, &value),
           "kinetis_integration_test_read32(device, flexram + 4u, &value)");
    expect(state, value == 0xa5a55a5au, "value == 0xa5a55a5au");

    const uint32_t flash = 0x100u;
    expect(state, !kinetis_memory_write(device, flash, 1u, CORTEX_M4_ACCESS_DATA, 0x5au),
           "!kinetis_memory_write(device, flash, 1u, CORTEX_M4_ACCESS_DATA, 0x5au)");
    uint8_t byte = 0x5au;
    expect(state, kinetis_write(device, flash, &byte, sizeof(byte)),
           "kinetis_write(device, flash, &byte, sizeof(byte))");
    byte = 0u;
    expect(state, kinetis_integration_test_read8(device, flash, &byte),
           "kinetis_integration_test_read8(device, flash, &byte)");
    expect(state, byte == 0x5au, "byte == 0x5au");
    kinetis_destroy(device);

    KinetisConfiguration small = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    small.flash_size = 8u;
    Kinetis* short_flash = kinetis_create(small);
    expect(state, short_flash != NULL, "short_flash != NULL");
    expect(state, kinetis_reset(short_flash), "kinetis_reset(short_flash)");
    kinetis_warm_reset(NULL, 0u, 0u);
    kinetis_destroy(short_flash);
}

void kinetis_integration_test_expect_manifest_fallback(TestState* state, Kinetis* device) {
    uint32_t value = 0;
    expect(state, kinetis_integration_test_read32(device, FMC_PFAPR, &value),
           "kinetis_integration_test_read32(device, FMC_PFAPR, &value)");
    expect(state, value == 0x00f8003fu, "value == 0x00f8003fu");
    expect(state, kinetis_integration_test_write32(device, FMC_PFAPR, UINT32_MAX),
           "kinetis_integration_test_write32(device, FMC_PFAPR, UINT32_MAX)");
    expect(state, kinetis_integration_test_read32(device, FMC_PFAPR, &value),
           "kinetis_integration_test_read32(device, FMC_PFAPR, &value)");
    expect(state, value == 0x00ffffffu, "value == 0x00ffffffu");
    expect(state, !kinetis_integration_test_read32(device, FMC_PFAPR + 0x0cu, &value),
           "!kinetis_integration_test_read32(device, FMC_PFAPR + 0x0cu, &value)");
    expect(state, !kinetis_integration_test_write32(device, FMC_PFAPR + 0x0cu, UINT32_MAX),
           "!kinetis_integration_test_write32(device, FMC_PFAPR + 0x0cu, UINT32_MAX)");
    expect(state,
           !kinetis_peripheral_write(device, FMC_PFAPR, 4u, CORTEX_M4_ACCESS_UNPRIVILEGED_DATA, 0u),
           "!kinetis_peripheral_write( device, FMC_PFAPR, 4u, "
           "CORTEX_M4_ACCESS_UNPRIVILEGED_DATA, 0u)");

    uint8_t flash = 0x5au;
    expect(state, kinetis_write(device, 0x100u, &flash, sizeof(flash)),
           "kinetis_write(device, 0x100u, &flash, sizeof(flash))");
    expect(state, kinetis_integration_test_write32(device, FMC_PFAPR, 0x10u),
           "kinetis_integration_test_write32(device, FMC_PFAPR, 0x10u)");
    expect(state, !kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value),
           "!kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value)");
    expect(state, kinetis_dma_read(device, 0x100u, 1u, &value),
           "kinetis_dma_read(device, 0x100u, 1u, &value)");
    expect(state, value == 0x5au, "value == 0x5au");
    expect(state, kinetis_integration_test_write32(device, FMC_PFAPR, 4u),
           "kinetis_integration_test_write32(device, FMC_PFAPR, 4u)");
    expect(state, !kinetis_dma_read(device, 0x100u, 1u, &value),
           "!kinetis_dma_read(device, 0x100u, 1u, &value)");
    expect(state, kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value),
           "kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value)");
    expect(state, value == 0x5au, "value == 0x5au");
    expect(state, kinetis_integration_test_write32(device, FMC_PFAPR, UINT32_MAX),
           "kinetis_integration_test_write32(device, FMC_PFAPR, UINT32_MAX)");
    expect(state, kinetis_integration_test_read32(device, FMC_PFB0CR, &value),
           "kinetis_integration_test_read32(device, FMC_PFB0CR, &value)");
    expect(state,
           kinetis_integration_test_write32(device, FMC_PFB0CR, (value & ~0x18u) | 0x00f00000u),
           "kinetis_integration_test_write32(device, FMC_PFB0CR, (value & ~0x18u) | 0x00f00000u)");

    const uint32_t alias = 0x42000000u + (FMC_TAGVDW0S0 - 0x40000000u) * 32u + 5u * 4u;
    expect(state, kinetis_integration_test_write32(device, alias, 1),
           "kinetis_integration_test_write32(device, alias, 1)");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value)");
    expect(state, value == 0x20u, "value == 0x20u");
    expect(state, kinetis_integration_test_read32(device, alias, &value),
           "kinetis_integration_test_read32(device, alias, &value)");
    expect(state, value == 1, "value == 1");
    expect(state, kinetis_integration_test_write32(device, FMC_TAGVDW0S0, 0x21u),
           "kinetis_integration_test_write32(device, FMC_TAGVDW0S0, 0x21u)");
    expect(state, kinetis_integration_test_write32(device, FMC_TAGVDW1S0, 0x41u),
           "kinetis_integration_test_write32(device, FMC_TAGVDW1S0, 0x41u)");
    expect(state, kinetis_integration_test_write32(device, FMC_DATAW0S0UM, 0x12345678u),
           "kinetis_integration_test_write32(device, FMC_DATAW0S0UM, 0x12345678u)");
    expect(state, kinetis_integration_test_write32(device, FMC_DATAW1S0UM, 0x89abcdefu),
           "kinetis_integration_test_write32(device, FMC_DATAW1S0UM, 0x89abcdefu)");
    expect(state, kinetis_integration_test_write32(device, FMC_PFB0CR, (1u << 19u) | (1u << 20u)),
           "kinetis_integration_test_write32(device, FMC_PFB0CR, (1u << 19u) | (1u << 20u))");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value)");
    expect(state, value == 0u, "value == 0u");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW1S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW1S0, &value)");
    expect(state, value == 0x41u, "value == 0x41u");
    expect(state, kinetis_integration_test_read32(device, FMC_DATAW0S0UM, &value),
           "kinetis_integration_test_read32(device, FMC_DATAW0S0UM, &value)");
    expect(state, value == 0u, "value == 0u");
    expect(state, kinetis_integration_test_read32(device, FMC_DATAW1S0UM, &value),
           "kinetis_integration_test_read32(device, FMC_DATAW1S0UM, &value)");
    expect(state, value == 0x89abcdefu, "value == 0x89abcdefu");
    expect(state, kinetis_integration_test_read32(device, FMC_PFB0CR, &value),
           "kinetis_integration_test_read32(device, FMC_PFB0CR, &value)");
    expect(state, (value & 0x00f80000u) == 0u, "(value & 0x00f80000u) == 0u");
    expect(state, kinetis_integration_test_write32(device, FMC_PFB0CR, 1u << 21u),
           "kinetis_integration_test_write32(device, FMC_PFB0CR, 1u << 21u)");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW1S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW1S0, &value)");
    expect(state, value == 0u, "value == 0u");
    expect(state, kinetis_integration_test_read32(device, FMC_DATAW1S0UM, &value),
           "kinetis_integration_test_read32(device, FMC_DATAW1S0UM, &value)");
    expect(state, value == 0u, "value == 0u");
    expect(state, !kinetis_integration_test_read32(device, 0x43ffffffu, &value),
           "!kinetis_integration_test_read32(device, 0x43ffffffu, &value)");

    expect(state, kinetis_integration_test_read32(device, MCM_PLASC, &value),
           "kinetis_integration_test_read32(device, MCM_PLASC, &value)");
    expect(state, value == 0x0017001fu, "value == 0x0017001fu");
    expect(state, !kinetis_integration_test_write32(device, MCM_PLASC, UINT32_MAX),
           "!kinetis_integration_test_write32(device, MCM_PLASC, UINT32_MAX)");
    expect(state, kinetis_integration_test_read32(device, MCM_PLASC, &value),
           "kinetis_integration_test_read32(device, MCM_PLASC, &value)");
    expect(state, value == 0x0017001fu, "value == 0x0017001fu");
}

void kinetis_integration_test_expect_fmc_cache(TestState* state) {
    Kinetis* device = kinetis_integration_test_create_device(state, KINETIS_PACKAGE_DC_121_XFBGA);
    expect(state, kinetis_reset(device), "kinetis_reset(device)");
    uint32_t value = 0u;
    expect(state, kinetis_integration_test_read32(device, FMC_PFB0CR, &value),
           "kinetis_integration_test_read32(device, FMC_PFB0CR, &value)");
    expect(state, kinetis_integration_test_write32(device, FMC_PFB0CR, value | 0x0ef00000u),
           "kinetis_integration_test_write32(device, FMC_PFB0CR, value | 0x0ef00000u)");
    uint8_t byte = 0x5au;
    expect(state, kinetis_write(device, 0x100u, &byte, sizeof(byte)),
           "kinetis_write(device, 0x100u, &byte, sizeof(byte))");
    expect(state, kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value),
           "kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value)");
    expect(state, value == 0x5au, "value == 0x5au");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value)");
    expect(state, (value & 1u) != 0u, "(value & 1u) != 0u");
    expect(state, kinetis_integration_test_read32(device, FMC_DATAW0S0LM, &value),
           "kinetis_integration_test_read32(device, FMC_DATAW0S0LM, &value)");
    expect(state, (value & 0xffu) == 0x5au, "(value & 0xffu) == 0x5au");
    expect(state, kinetis_flash_controller_write(device, 0x100u, 1u, 0xa5u),
           "kinetis_flash_controller_write(device, 0x100u, 1u, 0xa5u)");
    expect(state, kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value),
           "kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value)");
    expect(state, value == 0x5au, "value == 0x5au");
    expect(state, kinetis_integration_test_read32(device, FMC_PFB0CR, &value),
           "kinetis_integration_test_read32(device, FMC_PFB0CR, &value)");
    expect(state, kinetis_integration_test_write32(device, FMC_PFB0CR, value | (1u << 20u)),
           "kinetis_integration_test_write32(device, FMC_PFB0CR, value | (1u << 20u))");
    expect(state, kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value),
           "kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value)");
    expect(state, value == 0xa5u, "value == 0xa5u");
    kinetis_destroy(device);
}

void kinetis_integration_test_expect_fmc_invalidation_and_locking(TestState* state) {
    Kinetis* device = kinetis_integration_test_create_device(state, KINETIS_PACKAGE_DC_121_XFBGA);
    expect(state, kinetis_reset(device), "kinetis_reset(device)");
    uint8_t byte = 0x5au;
    uint32_t control = 0u;
    uint32_t tag = 0u;
    uint32_t value = 0u;
    expect(state, kinetis_write(device, 0x100u, &byte, sizeof(byte)),
           "kinetis_write(device, 0x100u, &byte, sizeof(byte))");
    expect(state, kinetis_integration_test_read32(device, FMC_PFB0CR, &control),
           "kinetis_integration_test_read32(device, FMC_PFB0CR, &control)");
    expect(state, kinetis_integration_test_write32(device, FMC_PFB0CR, control | 0x0ef00000u),
           "kinetis_integration_test_write32(device, FMC_PFB0CR, control | 0x0ef00000u)");
    expect(state, kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value),
           "kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value)");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &tag),
           "kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &tag)");
    expect(state, kinetis_integration_test_write32(device, FMC_TAGVDW1S0, tag),
           "kinetis_integration_test_write32(device, FMC_TAGVDW1S0, tag)");
    device->fmc_bank[1][0] = 0u;
    expect(state, kinetis_memory_write(device, 0x100u, 1u, CORTEX_M4_ACCESS_DEBUG, 0xa5u),
           "kinetis_memory_write(device, 0x100u, 1u, CORTEX_M4_ACCESS_DEBUG, 0xa5u)");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value)");
    expect(state, value == 0u, "value == 0u");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW1S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW1S0, &value)");
    expect(state, value == 0u, "value == 0u");

    expect(state, kinetis_integration_test_write32(device, FMC_PFB0CR, control | 0x0ff00000u),
           "kinetis_integration_test_write32(device, FMC_PFB0CR, control | 0x0ff00000u)");
    expect(state, kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value),
           "kinetis_memory_read(device, 0x100u, 1u, CORTEX_M4_ACCESS_DATA, &value)");
    expect(state, value == 0xa5u, "value == 0xa5u");
    expect(state, kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value),
           "kinetis_integration_test_read32(device, FMC_TAGVDW0S0, &value)");
    expect(state, value == 0u, "value == 0u");
    kinetis_destroy(device);
}

void kinetis_integration_test_expect_clock_gates(TestState* state, Kinetis* device) {
    uint32_t value = 0;
    uint8_t status = 0;
    CortexM4* cpu = kinetis_cpu(device);
    expect(state, kinetis_integration_test_read32(device, SIM_SCGC4, &value),
           "kinetis_integration_test_read32(device, SIM_SCGC4, &value)");
    expect(state, (value & (1u << 11)) == 0, "(value & (1u << 11)) == 0");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC4, 4, 0xf0100830u),
           "cortex_m4_write_memory(cpu, SIM_SCGC4, 4, 0xf0100830u)");
    expect(state, cortex_m4_write_memory(cpu, UART1_C2, 1, 0x24u),
           "cortex_m4_write_memory(cpu, UART1_C2, 1, 0x24u)");
    expect(state, kinetis_serial_receive(device, KINETIS_SERIAL_UART1, 0x5au, 0),
           "kinetis_serial_receive(device, KINETIS_SERIAL_UART1, 0x5au, 0)");
    expect(state, kinetis_integration_test_read8(device, UART1_S1, &status),
           "kinetis_integration_test_read8(device, UART1_S1, &status)");
    expect(state, (status & 0x20u) != 0, "(status & 0x20u) != 0");
    expect(state, cortex_m4_get_irq_pending(cpu, 33), "cortex_m4_get_irq_pending(cpu, 33)");
}

void kinetis_integration_test_expect_package_serial_extensions(TestState* state) {
    Kinetis* small = kinetis_integration_test_create_f12_device(state, KINETIS_PACKAGE_LH_64_LQFP);
    expect(state, !kinetis_serial_receive(small, KINETIS_SERIAL_UART3, 0x11u, 0u),
           "!kinetis_serial_receive(small, KINETIS_SERIAL_UART3, 0x11u, 0u)");
    expect(state, !kinetis_serial_receive(small, KINETIS_SERIAL_SPI1, 0x22u, 0u),
           "!kinetis_serial_receive(small, KINETIS_SERIAL_SPI1, 0x22u, 0u)");
    kinetis_destroy(small);

    Kinetis* large =
        kinetis_integration_test_create_f12_device(state, KINETIS_PACKAGE_MD_144_MAPBGA);
    CortexM4* cpu = kinetis_cpu(large);
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC1, 4u, 3u << 10u),
           "cortex_m4_write_memory(cpu, SIM_SCGC1, 4u, 3u << 10u)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC3, 4u, 1u << 12u),
           "cortex_m4_write_memory(cpu, SIM_SCGC3, 4u, 1u << 12u)");
    expect(state, cortex_m4_write_memory(cpu, SIM_SCGC4, 4u, 1u << 13u),
           "cortex_m4_write_memory(cpu, SIM_SCGC4, 4u, 1u << 13u)");
    expect(state, cortex_m4_write_memory(cpu, 0x4006d003u, 1u, 0x24u),
           "cortex_m4_write_memory(cpu, 0x4006d003u, 1u, 0x24u)");
    expect(state, cortex_m4_write_memory(cpu, 0x400ea003u, 1u, 0x24u),
           "cortex_m4_write_memory(cpu, 0x400ea003u, 1u, 0x24u)");
    expect(state, cortex_m4_write_memory(cpu, 0x400eb003u, 1u, 0x24u),
           "cortex_m4_write_memory(cpu, 0x400eb003u, 1u, 0x24u)");
    expect(state, kinetis_serial_receive(large, KINETIS_SERIAL_UART3, 0x33u, 0u),
           "kinetis_serial_receive(large, KINETIS_SERIAL_UART3, 0x33u, 0u)");
    expect(state, kinetis_serial_receive(large, KINETIS_SERIAL_UART4, 0x44u, 0u),
           "kinetis_serial_receive(large, KINETIS_SERIAL_UART4, 0x44u, 0u)");
    expect(state, kinetis_serial_receive(large, KINETIS_SERIAL_UART5, 0x55u, 0u),
           "kinetis_serial_receive(large, KINETIS_SERIAL_UART5, 0x55u, 0u)");
    expect(state, kinetis_serial_receive(large, KINETIS_SERIAL_SPI2, 0x66u, 0u),
           "kinetis_serial_receive(large, KINETIS_SERIAL_SPI2, 0x66u, 0u)");
    uint8_t value = 0u;
    expect(state, kinetis_integration_test_read8(large, 0x4006d007u, &value),
           "kinetis_integration_test_read8(large, 0x4006d007u, &value)");
    expect(state, value == 0x33u, "value == 0x33u");
    expect(state, kinetis_integration_test_read8(large, 0x400ea007u, &value),
           "kinetis_integration_test_read8(large, 0x400ea007u, &value)");
    expect(state, value == 0x44u, "value == 0x44u");
    expect(state, kinetis_integration_test_read8(large, 0x400eb007u, &value),
           "kinetis_integration_test_read8(large, 0x400eb007u, &value)");
    expect(state, value == 0x55u, "value == 0x55u");
    kinetis_destroy(large);
}
