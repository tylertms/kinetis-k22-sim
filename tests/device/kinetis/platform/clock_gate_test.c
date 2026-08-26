#include "kinetis.h"

#include <stdint.h>

#include "device/kinetis/internal.h"
#include "test.h"

enum {
    SIM_SCGC1 = 0x40048028u,
    SIM_SCGC2 = 0x4004802cu,
    SIM_SCGC3 = 0x40048030u,
    SIM_SCGC4 = 0x40048034u,
    SIM_SCGC5 = 0x40048038u,
    SIM_SCGC6 = 0x4004803cu,
    SIM_SCGC7 = 0x40048040u,
};

typedef struct {
    KinetisPeripheralId peripheral_id;
    uint32_t register_address;
    uint8_t gate_bit;
} GateCase;

typedef struct {
    uint32_t register_address;
    uint8_t access_size;
} RegisterAccess;

static const GateCase cases[] = {
    {KINETIS_PERIPHERAL_DMA, SIM_SCGC7, 1u},     {KINETIS_PERIPHERAL_FB, SIM_SCGC7, 0u},
    {KINETIS_PERIPHERAL_SYSMPU, SIM_SCGC7, 2u},  {KINETIS_PERIPHERAL_FTFE, SIM_SCGC6, 0u},
    {KINETIS_PERIPHERAL_DMAMUX, SIM_SCGC6, 1u},  {KINETIS_PERIPHERAL_CAN0, SIM_SCGC6, 4u},
    {KINETIS_PERIPHERAL_FTM0, SIM_SCGC6, 24u},   {KINETIS_PERIPHERAL_FTM1, SIM_SCGC6, 25u},
    {KINETIS_PERIPHERAL_FTM2, SIM_SCGC3, 24u},   {KINETIS_PERIPHERAL_FTM3, SIM_SCGC3, 25u},
    {KINETIS_PERIPHERAL_ADC0, SIM_SCGC6, 27u},   {KINETIS_PERIPHERAL_ADC1, SIM_SCGC3, 27u},
    {KINETIS_PERIPHERAL_DAC0, SIM_SCGC2, 12u},   {KINETIS_PERIPHERAL_DAC1, SIM_SCGC2, 13u},
    {KINETIS_PERIPHERAL_SPI0, SIM_SCGC6, 12u},   {KINETIS_PERIPHERAL_SPI1, SIM_SCGC6, 13u},
    {KINETIS_PERIPHERAL_SPI2, SIM_SCGC3, 12u},   {KINETIS_PERIPHERAL_SDHC, SIM_SCGC3, 17u},
    {KINETIS_PERIPHERAL_I2S0, SIM_SCGC6, 15u},   {KINETIS_PERIPHERAL_CRC, SIM_SCGC6, 18u},
    {KINETIS_PERIPHERAL_USBDCD, SIM_SCGC6, 21u}, {KINETIS_PERIPHERAL_PDB0, SIM_SCGC6, 22u},
    {KINETIS_PERIPHERAL_PIT, SIM_SCGC6, 23u},    {KINETIS_PERIPHERAL_RTC, SIM_SCGC6, 29u},
    {KINETIS_PERIPHERAL_LPTMR0, SIM_SCGC5, 0u},  {KINETIS_PERIPHERAL_CMT, SIM_SCGC4, 2u},
    {KINETIS_PERIPHERAL_I2C0, SIM_SCGC4, 6u},    {KINETIS_PERIPHERAL_I2C1, SIM_SCGC4, 7u},
    {KINETIS_PERIPHERAL_I2C2, SIM_SCGC1, 6u},    {KINETIS_PERIPHERAL_UART0, SIM_SCGC4, 10u},
    {KINETIS_PERIPHERAL_UART1, SIM_SCGC4, 11u},  {KINETIS_PERIPHERAL_UART2, SIM_SCGC4, 12u},
    {KINETIS_PERIPHERAL_UART3, SIM_SCGC4, 13u},  {KINETIS_PERIPHERAL_UART4, SIM_SCGC1, 10u},
    {KINETIS_PERIPHERAL_UART5, SIM_SCGC1, 11u},  {KINETIS_PERIPHERAL_USB0, SIM_SCGC4, 18u},
    {KINETIS_PERIPHERAL_CMP0, SIM_SCGC4, 19u},   {KINETIS_PERIPHERAL_CMP1, SIM_SCGC4, 19u},
    {KINETIS_PERIPHERAL_CMP2, SIM_SCGC4, 19u},   {KINETIS_PERIPHERAL_VREF, SIM_SCGC4, 20u},
    {KINETIS_PERIPHERAL_PORTA, SIM_SCGC5, 9u},   {KINETIS_PERIPHERAL_PORTB, SIM_SCGC5, 10u},
    {KINETIS_PERIPHERAL_PORTC, SIM_SCGC5, 11u},  {KINETIS_PERIPHERAL_PORTD, SIM_SCGC5, 12u},
    {KINETIS_PERIPHERAL_PORTE, SIM_SCGC5, 13u},  {KINETIS_PERIPHERAL_GPIOA, SIM_SCGC5, 9u},
    {KINETIS_PERIPHERAL_GPIOB, SIM_SCGC5, 10u},  {KINETIS_PERIPHERAL_GPIOC, SIM_SCGC5, 11u},
    {KINETIS_PERIPHERAL_GPIOD, SIM_SCGC5, 12u},  {KINETIS_PERIPHERAL_GPIOE, SIM_SCGC5, 13u},
};

static const GateCase secondary_profile_cases[] = {
    {KINETIS_PERIPHERAL_RNG, SIM_SCGC6, 9u},   {KINETIS_PERIPHERAL_LPUART0, SIM_SCGC6, 10u},
    {KINETIS_PERIPHERAL_FTM2, SIM_SCGC6, 26u}, {KINETIS_PERIPHERAL_FTM3, SIM_SCGC6, 6u},
    {KINETIS_PERIPHERAL_ADC1, SIM_SCGC6, 7u},  {KINETIS_PERIPHERAL_DAC1, SIM_SCGC6, 8u},
};

static Kinetis* create_device(TestState* state, KinetisProfile profile, KinetisPackage package) {
    KinetisConfiguration configuration = kinetis_default_configuration();
    configuration.profile = profile;
    configuration.package = package;
    configuration.flash_size = 4096u;
    configuration.sram_size = 65536u;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "device != NULL");
    return device;
}

static RegisterAccess readable_register(Kinetis* device, KinetisPeripheralId peripheral_id) {
    KinetisPeripheralBlock block;
    if (!kinetis_profile_peripheral_block(device->profile, peripheral_id, &block)) {
        return (RegisterAccess){0u, 0u};
    }
    for (size_t register_index = 0u; register_index < device->manifest->register_count;
         register_index++) {
        const KinetisRegisterDescriptor* descriptor = &device->manifest->registers[register_index];
        if (descriptor->address >= block.address &&
            descriptor->address < block.address + block.size &&
            (descriptor->access & KINETIS_REGISTER_ACCESS_READ) != 0u) {
            return (RegisterAccess){descriptor->address, (uint8_t)(descriptor->width / 8u)};
        }
    }
    const uint8_t sizes[] = {1u, 2u, 4u};
    for (size_t size_index = 0u; size_index < sizeof(sizes); size_index++) {
        uint32_t read_value = 0u;
        if (kinetis_peripheral_read(device, block.address, sizes[size_index],
                                    CORTEX_M4_ACCESS_DEBUG, &read_value)) {
            return (RegisterAccess){block.address, sizes[size_index]};
        }
    }
    return (RegisterAccess){0u, 0u};
}

static void clear_gates(TestState* state, Kinetis* device) {
    const uint32_t gate_registers[] = {SIM_SCGC1, SIM_SCGC2, SIM_SCGC3, SIM_SCGC4,
                                       SIM_SCGC5, SIM_SCGC6, SIM_SCGC7};
    for (size_t register_index = 0u;
         register_index < sizeof(gate_registers) / sizeof(gate_registers[0]); register_index++) {
        if (kinetis_register_manifest_lookup(device->profile->id, gate_registers[register_index],
                                             32u) != NULL) {
            expect(state,
                   kinetis_peripheral_write(device, gate_registers[register_index], 4u,
                                            CORTEX_M4_ACCESS_DEBUG, 0u),
                   "kinetis_peripheral_write(device, gate_registers[register_index], 4u, "
                   "CORTEX_M4_ACCESS_DEBUG, 0u)");
        }
    }
}

static void test_cases(TestState* state, Kinetis* device, const GateCase* gate_cases,
                       size_t gate_case_count) {
    clear_gates(state, device);
    for (size_t case_index = 0u; case_index < gate_case_count; case_index++) {
        const RegisterAccess register_access =
            readable_register(device, gate_cases[case_index].peripheral_id);
        expect(state, register_access.access_size != 0u, "register_access.access_size != 0u");
        if (register_access.access_size == 0u) {
            continue;
        }
        uint32_t read_value = 0u;
        expect(state,
               !kinetis_peripheral_read(device, register_access.register_address,
                                        register_access.access_size, CORTEX_M4_ACCESS_DATA,
                                        &read_value),
               "!kinetis_peripheral_read(device, register_access.register_address, "
               "register_access.access_size, CORTEX_M4_ACCESS_DATA, &read_value)");
        expect(state,
               kinetis_peripheral_write(device, gate_cases[case_index].register_address, 4u,
                                        CORTEX_M4_ACCESS_DEBUG,
                                        1u << gate_cases[case_index].gate_bit),
               "kinetis_peripheral_write(device, gate_cases[case_index].register_address, "
               "4u, CORTEX_M4_ACCESS_DEBUG, 1u << gate_cases[case_index].gate_bit)");
        const bool peripheral_enabled = kinetis_peripheral_read(
            device, register_access.register_address, register_access.access_size,
            CORTEX_M4_ACCESS_DATA, &read_value);
        expect(state, peripheral_enabled, "peripheral_enabled");
        expect(state,
               kinetis_peripheral_write(device, gate_cases[case_index].register_address, 4u,
                                        CORTEX_M4_ACCESS_DEBUG, 0u),
               "kinetis_peripheral_write(device, gate_cases[case_index].register_address, 4u, "
               "CORTEX_M4_ACCESS_DEBUG, 0u)");
    }
}

int main(void) {
    TestState state = {0};
    Kinetis* primary_profile_device =
        create_device(&state, KINETIS_PROFILE_MK22FN1M012, KINETIS_PACKAGE_LQ_144_LQFP);
    Kinetis* secondary_profile_device =
        create_device(&state, KINETIS_PROFILE_MK22FN51212, KINETIS_PACKAGE_DC_121_XFBGA);
    test_cases(&state, primary_profile_device, cases, sizeof(cases) / sizeof(cases[0]));
    test_cases(&state, secondary_profile_device, secondary_profile_cases,
               sizeof(secondary_profile_cases) / sizeof(secondary_profile_cases[0]));
    kinetis_destroy(primary_profile_device);
    kinetis_destroy(secondary_profile_device);
    return test_finish(&state);
}
