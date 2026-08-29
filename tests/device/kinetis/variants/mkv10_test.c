#include "kinetis.h"

#include <limits.h>
#include <stdint.h>

#include "device/kinetis/internal.h"
#include "device/kinetis/communication/serial/internal.h"
#include "device/kinetis/memory/internal.h"
#include "device/kinetis/timing/internal.h"
#include "kinetis_test.h"
#include "test.h"

static const uint32_t TEST_SIM_SCGC5 = 0x40048038u;
static const uint32_t TEST_SIM_SCGC6 = 0x4004803cu;
static const uint32_t TEST_SIM_CLKDIV1 = 0x40048044u;
static const uint32_t TEST_SIM_FCFG1 = 0x4004804cu;
static const uint32_t TEST_SIM_SOPT4 = 0x4004800cu;
static const uint32_t TEST_SIM_SOPT5 = 0x40048010u;
static const uint32_t TEST_SIM_SOPT6 = 0x40048014u;
static const uint32_t TEST_SIM_SOPT8 = 0x4004801cu;
static const uint32_t TEST_SIM_SOPT9 = 0x40048020u;
static const uint32_t PORTA_PCR0 = 0x40049000u;
static const uint32_t PORTB_PCR0 = 0x4004a000u;
static const uint32_t PORTE_PCR0 = 0x4004d000u;
static const uint32_t GPIOA_PDOR = 0x400ff000u;
static const uint32_t GPIOA_PCOR = 0x400ff008u;
static const uint32_t FGPIOA_PDOR = 0xf8000000u;
static const uint32_t FGPIOA_PSOR = 0xf8000004u;
static const uint32_t PDB0_SC = 0x40036000u;
static const uint32_t PDB0_MOD = 0x40036004u;
static const uint32_t PDB1_SC = 0x40031000u;
static const uint32_t PDB1_MOD = 0x40031004u;
static const uint32_t PDB0_DACINTC1 = 0x40036158u;
static const uint32_t PDB0_DACINT1 = 0x4003615cu;
static const uint32_t DAC0_DAT0L = 0x4003f000u;
static const uint32_t DAC0_DAT1L = 0x4003f002u;
static const uint32_t DAC0_SR = 0x4003f020u;
static const uint32_t DAC0_C0 = 0x4003f021u;
static const uint32_t DAC0_C1 = 0x4003f022u;
static const uint32_t DAC0_C2 = 0x4003f023u;
static const uint32_t CMP0_CR0 = 0x40073000u;
static const uint32_t CMP0_CR1 = 0x40073001u;
static const uint32_t CMP0_FPR = 0x40073002u;
static const uint32_t CMP0_SCR = 0x40073003u;
static const uint32_t CMP0_MUXCR = 0x40073005u;
static const uint32_t SMC_PMPROT = 0x4007e000u;
static const uint32_t SMC_PMCTRL = 0x4007e001u;
static const uint32_t SMC_STOPCTRL = 0x4007e002u;
static const uint32_t SMC_PMSTAT = 0x4007e003u;
static const uint32_t RCM_SRS0 = 0x4007f000u;
static const uint32_t RCM_RPFC = 0x4007f004u;
static const uint32_t RCM_RPFW = 0x4007f005u;
static const uint32_t DMAMUX_CHCFG0 = 0x40021000u;
static const uint32_t FTM0_CNT = 0x40038004u;
static const uint32_t FTM0_SC = 0x40038000u;
static const uint32_t FTM0_MOD = 0x40038008u;
static const uint32_t FTM0_EXTTRIG = 0x4003806cu;
static const uint32_t LPTMR0_CSR = 0x40040000u;
static const uint32_t LPTMR0_PSR = 0x40040004u;
static const uint32_t LPTMR0_CMR = 0x40040008u;
static const uint32_t MMDVSQ_DEND = 0xf0004000u;
static const uint32_t MMDVSQ_DSOR = 0xf0004004u;
static const uint32_t MMDVSQ_CSR = 0xf0004008u;
static const uint32_t MMDVSQ_RES = 0xf000400cu;
static const uint32_t MMDVSQ_RCND = 0xf0004010u;
static const uint32_t SCB_CPUID = 0xe000ed00u;
static const uint32_t SCB_CCR = 0xe000ed14u;
static const uint32_t SCB_CFSR = 0xe000ed28u;
static const uint32_t SCB_SHCSR = 0xe000ed24u;
static const uint32_t SCB_DFSR = 0xe000ed30u;
static const uint32_t SYST_CSR = 0xe000e010u;
static const uint32_t SYST_RVR = 0xe000e014u;
static const uint32_t SYST_CVR = 0xe000e018u;
static const uint32_t DWT_CTRL = 0xe0001000u;
static const uint32_t BPU_CTRL = 0xe0002000u;
static const uint32_t DHCSR = 0xe000edf0u;
static const uint32_t DEMCR = 0xe000edfcu;
static const uint32_t MTB_POSITION = 0xf0000000u;
static const uint32_t MTB_MASTER = 0xf0000004u;
static const uint32_t MTB_FLOW = 0xf0000008u;
static const uint32_t MTB_BASE = 0xf000000cu;
static const uint32_t MTBDWT_COMP0 = 0xf0001020u;
static const uint32_t MTBDWT_FCT0 = 0xf0001028u;
static const uint32_t MTBDWT_TBCTRL = 0xf0001200u;
static const uint32_t MCM_CPO = 0xf0003040u;
static const uint32_t MCG_C1 = 0x40064000u;
static const uint32_t MCG_C3 = 0x40064002u;
static const uint32_t MCG_C4 = 0x40064003u;
static const uint32_t MCG_SC = 0x40064008u;
static const uint32_t MCG_ATCVH = 0x4006400au;
static const uint32_t MCG_ATCVL = 0x4006400bu;
static const uint32_t FTFA = 0x40020000u;

static uint32_t read32(TestState* state, Kinetis* device, uint32_t address) {
    uint32_t value = 0u;
    expect(state, kinetis_read(device, address, &value, sizeof(value)),
           "kinetis_read(device, address, &value, sizeof(value))");
    return value;
}

static void write32(TestState* state, Kinetis* device, uint32_t address, uint32_t value) {
    expect(state, kinetis_write(device, address, &value, sizeof(value)),
           "kinetis_write(device, address, &value, sizeof(value))");
}

static void write8(TestState* state, Kinetis* device, uint32_t address, uint8_t value) {
    expect(state, kinetis_write(device, address, &value, sizeof(value)),
           "kinetis_write(device, address, &value, sizeof(value))");
}

static uint8_t read8(TestState* state, Kinetis* device, uint32_t address) {
    uint8_t value = 0u;
    expect(state, kinetis_read(device, address, &value, sizeof(value)),
           "kinetis_read(device, address, &value, sizeof(value))");
    return value;
}

static void data_write8(TestState* state, KinetisData* data, uint32_t address, uint8_t value) {
    expect(state, kinetis_data_write(data, address, 1u, value),
           "kinetis_data_write(data, address, 1u, value)");
}

static uint8_t data_read8(TestState* state, KinetisData* data, uint32_t address) {
    uint32_t value = 0u;
    expect(state, kinetis_data_read(data, address, 1u, &value),
           "kinetis_data_read(data, address, 1u, &value)");
    return (uint8_t)value;
}

static uint32_t flash_fccob_address(uint8_t index) {
    static const uint8_t offsets[12] = {7u, 6u, 5u, 4u, 11u, 10u, 9u, 8u, 15u, 14u, 13u, 12u};
    return FTFA + offsets[index];
}

static void write_fccob(TestState* state, KinetisData* data, uint8_t index, uint8_t value) {
    data_write8(state, data, flash_fccob_address(index), value);
}

static void execute_flash_command(TestState* state, KinetisData* data, uint8_t command,
                                  uint8_t parameter, uint32_t cycles) {
    data_write8(state, data, FTFA, 0x70u);
    write_fccob(state, data, 0u, command);
    write_fccob(state, data, 1u, parameter);
    data_write8(state, data, FTFA, 0x80u);
    kinetis_data_advance(data, cycles);
}

static Kinetis* create_device(TestState* state) {
    KinetisConfiguration configuration = kinetis_configuration(KINETIS_PROFILE_MKV10Z1287);
    configuration.package = KINETIS_PACKAGE_LH_64_LQFP;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "device != NULL");
    if (device == NULL)
        return NULL;
    const uint32_t vectors[] = {0x20002000u, 0x00000101u};
    const uint16_t nop = 0xbf00u;
    expect(state, kinetis_load(device, 0u, vectors, sizeof(vectors)),
           "kinetis_load(device, 0u, vectors, sizeof(vectors))");
    expect(state, kinetis_load(device, 0x100u, &nop, sizeof(nop)),
           "kinetis_load(device, 0x100u, &nop, sizeof(nop))");
    expect(state, kinetis_reset(device), "kinetis_reset(device)");
    return device;
}

static bool cpu_store32(TestState* state, Kinetis* device, uint32_t address, uint32_t value) {
    const uint16_t instruction = 0x6001u;
    if (!kinetis_load(device, 0x180u, &instruction, sizeof(instruction)))
        return false;
    cortex_m4_set_register(kinetis_cpu(device), 0u, address);
    cortex_m4_set_register(kinetis_cpu(device), 1u, value);
    cortex_m4_set_register(kinetis_cpu(device), 15u, 0x180u);
    const CortexM4Result result = cortex_m4_step(kinetis_cpu(device));
    expect(state, result.stop == CORTEX_M4_STOP_RUNNING, "CPU store completes");
    return result.stop == CORTEX_M4_STOP_RUNNING;
}

static bool cpu_load32(TestState* state, Kinetis* device, uint32_t address,
                       uint32_t* output_value) {
    const uint16_t instruction = 0x6801u;
    if (!kinetis_load(device, 0x180u, &instruction, sizeof(instruction)))
        return false;
    cortex_m4_set_register(kinetis_cpu(device), 0u, address);
    cortex_m4_set_register(kinetis_cpu(device), 15u, 0x180u);
    const CortexM4Result result = cortex_m4_step(kinetis_cpu(device));
    expect(state, result.stop == CORTEX_M4_STOP_RUNNING, "CPU load completes");
    if (result.stop != CORTEX_M4_STOP_RUNNING)
        return false;
    *output_value = cortex_m4_get_register(kinetis_cpu(device), 1u);
    return true;
}

static void test_core(TestState* state, Kinetis* device) {
    uint32_t value = 0u;
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SCB_CPUID, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 0x410cc601u,
           "CPUID identifies Cortex-M0+ r0p1");
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SCB_CPUID, 1u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "Cortex-M0+ rejects byte SCS reads");
    expect(state,
           cortex_m4_system_write(kinetis_cpu(device), SCB_CCR, 2u, CORTEX_M4_ACCESS_DEBUG,
                                  0u) == CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "Cortex-M0+ rejects DAP halfword SCS writes");
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SCB_CCR, 4u, CORTEX_M4_ACCESS_DATA, &value) ==
                   CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 0x208u,
           "CCR exposes the ARMv6-M fixed value");
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SCB_CFSR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "CFSR is not implemented by ARMv6-M");
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SCB_SHCSR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "processor cannot access the ARMv6-M SHCSR");
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SCB_DFSR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "processor cannot access the ARMv6-M DFSR");
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SCB_DFSR, 4u, CORTEX_M4_ACCESS_DEBUG,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "DAP can access the ARMv6-M DFSR");
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), DHCSR, 4u, CORTEX_M4_ACCESS_DATA, &value) ==
               CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "processor cannot access the ARMv6-M Debug Control Block");
    expect(state,
           cortex_m4_debug_read(kinetis_cpu(device), DWT_CTRL, 4u, &value) ==
                   CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 0x20000000u,
           "ARMv6-M DWT exposes two watchpoint comparators");
    expect(state,
           cortex_m4_debug_read(kinetis_cpu(device), BPU_CTRL, 4u, &value) ==
                   CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 0x20u,
           "ARMv6-M BPU exposes two breakpoint comparators");
    expect(state,
           cortex_m4_debug_read(kinetis_cpu(device), 0xe0000000u, 4u, &value) ==
               CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "MKV10 has no ITM");
    expect(state,
           cortex_m4_debug_read(kinetis_cpu(device), 0xe0040000u, 4u, &value) ==
               CORTEX_M4_SYSTEM_ACCESS_REJECTED,
           "MKV10 has no TPIU");
    expect(state,
           cortex_m4_debug_write(kinetis_cpu(device), DEMCR, 4u, UINT32_MAX) ==
               CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "DAP writes ARMv6-M DEMCR");
    expect(state,
           cortex_m4_debug_read(kinetis_cpu(device), DEMCR, 4u, &value) ==
                   CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 0x01000401u,
           "ARMv6-M DEMCR implements only DWT and vector catch controls");
    expect(state, (kinetis_cpu(device)->system_pending & (1u << 12u)) == 0u,
           "ARMv6-M has no DebugMonitor exception");
    expect(state,
           cortex_m4_debug_write(kinetis_cpu(device), DEMCR, 4u, 0u) ==
               CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "DAP clears ARMv6-M DEMCR");
    expect(state,
           cortex_m4_check_instruction_constraints(kinetis_cpu(device), 0xb100u, 0u, false) ==
               CORTEX_M4_INSTRUCTION_UNDEFINED,
           "ARMv6-M rejects CBZ");
    expect(state,
           cortex_m4_check_instruction_constraints(kinetis_cpu(device), 0xb900u, 0u, false) ==
               CORTEX_M4_INSTRUCTION_UNDEFINED,
           "ARMv6-M rejects CBNZ");
    const uint16_t move_pc = 0x469fu;
    expect(state, kinetis_load(device, 0x300u, &move_pc, sizeof(move_pc)),
           "MKV10 loads the official MOV PC jump-table instruction");
    kinetis_cpu(device)->cfsr = 0u;
    cortex_m4_set_register(kinetis_cpu(device), 3u, 0xcf2cu);
    cortex_m4_set_register(kinetis_cpu(device), 15u, 0x300u);
    expect(state,
           cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING &&
               cortex_m4_get_register(kinetis_cpu(device), 15u) == 0xcf2cu &&
               (kinetis_cpu(device)->cfsr & (1u << 17u)) == 0u,
           "ARMv6-M MOV PC accepts the official even Thumb target without interworking");
    expect(state,
           cortex_m4_system_write(kinetis_cpu(device), SYST_RVR, 4u, CORTEX_M4_ACCESS_DATA, 3u) ==
               CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "MKV10 writes the SysTick reload value");
    expect(state,
           cortex_m4_system_write(kinetis_cpu(device), SYST_CVR, 4u, CORTEX_M4_ACCESS_DATA, 0u) ==
               CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "MKV10 clears the SysTick current value");
    expect(state,
           cortex_m4_system_write(kinetis_cpu(device), SYST_CSR, 4u, CORTEX_M4_ACCESS_DATA, 1u) ==
               CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "MKV10 enables the SysTick external clock");
    cortex_m4_advance(kinetis_cpu(device), 15u);
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SYST_CVR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 0u,
           "MKV10 SysTick waits for the sixteenth core clock");
    cortex_m4_advance(kinetis_cpu(device), 1u);
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SYST_CVR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 3u,
           "MKV10 SysTick external source runs at core clock divided by sixteen");
    expect(state,
           cortex_m4_system_write(kinetis_cpu(device), SYST_CSR, 4u, CORTEX_M4_ACCESS_DATA, 5u) ==
               CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "MKV10 selects the SysTick processor clock");
    cortex_m4_advance(kinetis_cpu(device), 1u);
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SYST_CVR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 2u,
           "MKV10 SysTick processor source runs at the core clock");
    kinetis_cpu(device)->sleeping = true;
    cortex_m4_advance(kinetis_cpu(device), 32u);
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SYST_CVR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 2u,
           "MKV10 SysTick stops while the core clock is gated");
    kinetis_cpu(device)->sleeping = false;
    cortex_m4_advance(kinetis_cpu(device), 1u);
    expect(state,
           cortex_m4_system_read(kinetis_cpu(device), SYST_CVR, 4u, CORTEX_M4_ACCESS_DATA,
                                 &value) == CORTEX_M4_SYSTEM_ACCESS_ACCEPTED &&
               value == 1u,
           "MKV10 SysTick resumes from its preserved count after wakeup");
    expect(state,
           cortex_m4_system_write(kinetis_cpu(device), SYST_CSR, 4u, CORTEX_M4_ACCESS_DATA, 0u) ==
               CORTEX_M4_SYSTEM_ACCESS_ACCEPTED,
           "MKV10 disables SysTick");
    const uint32_t reset_core_clock = kinetis_core_clock_hz(device);
    expect(state, kinetis_bus_clock_hz(device) == reset_core_clock / 2u &&
                      device->timing.flash_clock_hz == reset_core_clock / 2u,
           "MKV10 reset bus and flash clocks use OUTDIV4 after OUTDIV1");
    write32(state, device, TEST_SIM_CLKDIV1, 0x10030000u);
    expect(state, kinetis_core_clock_hz(device) == reset_core_clock / 2u &&
                      kinetis_bus_clock_hz(device) == reset_core_clock / 8u &&
                      device->timing.flash_clock_hz == reset_core_clock / 8u,
           "MKV10 clock dividers cascade OUTDIV4 after OUTDIV1");
    write32(state, device, TEST_SIM_CLKDIV1, 0x00011000u);
    kinetis_cpu(device)->control = 1u;
    cortex_m4_system_set_pending(kinetis_cpu(device), 3u, false);
    expect(state, cpu_store32(state, device, TEST_SIM_CLKDIV1, 0u),
           "unprivileged MKV10 SIM store executes into fault handling");
    expect(state, (kinetis_cpu(device)->system_pending & (1u << 3u)) != 0u,
           "unprivileged MKV10 SIM writes escalate to HardFault");
    kinetis_cpu(device)->control = 0u;
    cortex_m4_system_set_pending(kinetis_cpu(device), 3u, false);
    expect(state, device->timing.ftm[4].quadrature_capable &&
                      device->timing.ftm[5].quadrature_capable,
           "MKV10 FTM4 and FTM5 expose quadrature decoding");
    kinetis_internal_data_interrupt(device, KINETIS_DATA_INTERRUPT_DMA0, true);
    kinetis_internal_data_interrupt(device, KINETIS_DATA_INTERRUPT_DMA4, false);
    expect(state, (device->cpu->irq_level[0] & 1u) != 0u,
           "MKV10 shared DMA0 DMA4 IRQ preserves either asserted channel");
    kinetis_internal_data_interrupt(device, KINETIS_DATA_INTERRUPT_DMA0, false);
    kinetis_internal_data_interrupt(device, KINETIS_DATA_INTERRUPT_FTFA, true);
    kinetis_internal_data_interrupt(device, KINETIS_DATA_INTERRUPT_FLASH_COLLISION, false);
    expect(state, (device->cpu->irq_level[0] & (1u << 5u)) != 0u,
           "MKV10 shared flash IRQ preserves either asserted source");
    kinetis_internal_data_interrupt(device, KINETIS_DATA_INTERRUPT_FTFA, false);
}

static void test_reset_pin_filter(TestState* state, Kinetis* device) {
    expect(state, kinetis_reset(device), "MKV10 reset-pin test resets the device");
    write8(state, device, RCM_RPFC, 1u);
    write8(state, device, RCM_RPFW, 31u);
    const uint64_t reset_generation = device->timing.reset_generation;
    expect(state, kinetis_set_reset_pin(device, false), "MKV10 reset pin asserts");
    kinetis_advance(device, 63u);
    expect(state,
           device->timing.reset_generation == reset_generation &&
               !device->timing.reset_pin_held,
           "MKV10 bus filter rejects fewer than thirty-two bus clocks");
    kinetis_advance(device, 1u);
    expect(state,
           device->timing.reset_generation == reset_generation + 1u &&
               device->timing.reset_pin_held && read8(state, device, RCM_SRS0) == 0x40u &&
               read8(state, device, RCM_RPFC) == 1u && read8(state, device, RCM_RPFW) == 31u,
           "MKV10 bus filter asserts and retains configuration across pin reset");
    const uint32_t held_pc = cortex_m4_get_register(kinetis_cpu(device), 15u);
    (void)cortex_m4_step(kinetis_cpu(device));
    expect(state, cortex_m4_get_register(kinetis_cpu(device), 15u) == held_pc,
           "MKV10 holds the processor at reset while RESET remains low");
    expect(state, kinetis_set_reset_pin(device, true), "MKV10 reset pin deasserts");
    kinetis_advance(device, 63u);
    expect(state, device->timing.reset_pin_held,
           "MKV10 applies the bus filter to reset deassertion");
    kinetis_advance(device, 1u);
    expect(state, !device->timing.reset_pin_held && device->timing.reset_pin_filtered_high,
           "MKV10 releases reset after qualified deassertion");
    expect(state, kinetis_test_disable_watchdog(device),
           "MKV10 reset-pin test disables the watchdog");

    device->timing.reset_pin_input_high = false;
    device->timing.reset_pin_filtered_high = false;
    write8(state, device, RCM_RPFC, 2u);
    expect(state, kinetis_set_reset_pin(device, true), "MKV10 LPO-filtered deassertion starts");
    const uint32_t lpo_filter_cycles =
        (uint32_t)(((uint64_t)kinetis_core_clock_hz(device) * 5u + 999u) / 1000u);
    kinetis_advance(device, lpo_filter_cycles - 1u);
    expect(state, !device->timing.reset_pin_filtered_high,
           "MKV10 LPO filter waits through synchronizer and three samples");
    kinetis_advance(device, 1u);
    expect(state, device->timing.reset_pin_filtered_high,
           "MKV10 LPO filter accepts either edge after five clocks");

    write8(state, device, RCM_RPFC, 4u);
    device->cpu->sleeping = true;
    device->cpu->scr |= 4u;
    kinetis_timing_set_cpu_sleeping(&device->timing, true, true);
    expect(state, kinetis_set_reset_pin(device, false),
           "MKV10 stop-mode RESET assertion starts");
    kinetis_advance(device, lpo_filter_cycles - 1u);
    expect(state, !device->timing.reset_pin_held,
           "MKV10 stop-mode LPO filter rejects fewer than five clocks");
    kinetis_advance(device, 1u);
    expect(state, device->timing.reset_pin_held,
           "MKV10 stop-mode RESET uses the LPO filter");
    expect(state, kinetis_set_reset_pin(device, true),
           "MKV10 stop-mode RESET deassertion starts");
    kinetis_advance(device, lpo_filter_cycles);
    expect(state, !device->timing.reset_pin_held,
           "MKV10 stop-mode RESET releases after five LPO clocks");

    expect(state, kinetis_reset(device), "MKV10 VLLS0 RESET-pin test resets the device");
    write8(state, device, SMC_PMPROT, 2u);
    write8(state, device, SMC_PMCTRL, 4u);
    write8(state, device, SMC_STOPCTRL, 0u);
    write8(state, device, RCM_RPFC, 4u);
    device->cpu->sleeping = true;
    device->cpu->scr |= 4u;
    kinetis_timing_set_cpu_sleeping(&device->timing, true, true);
    const uint64_t vlls0_reset_generation = device->timing.reset_generation;
    expect(state, kinetis_set_reset_pin(device, false) && device->timing.reset_pin_held &&
                      device->timing.reset_generation == vlls0_reset_generation + 1u,
           "MKV10 VLLS0 bypasses RESET filtering");
    expect(state, kinetis_set_reset_pin(device, true) && !device->timing.reset_pin_held,
           "MKV10 VLLS0 releases RESET without filtering");

    const uint8_t reset_pin_disabled = 0xf7u;
    expect(state, kinetis_load(device, 0x40du, &reset_pin_disabled, 1u) && kinetis_reset(device),
           "MKV10 latches disabled RESET pin option on POR");
    expect(state, !kinetis_set_reset_pin(device, false),
           "MKV10 ignores RESET input when FOPT disables the pin");
    const uint8_t reset_pin_enabled = 0xffu;
    expect(state, kinetis_load(device, 0x40du, &reset_pin_enabled, 1u) && kinetis_reset(device),
           "MKV10 restores the RESET pin option for remaining tests");
}

static void test_execute_never(TestState* state, Kinetis* device) {
    CortexM4* cpu = kinetis_cpu(device);
    const uint32_t addresses[] = {GPIOA_PDOR, 0x4400f000u, FGPIOA_PDOR};
    for (size_t index = 0u; index < sizeof(addresses) / sizeof(addresses[0]); index++) {
        cortex_m4_system_set_pending(cpu, 3u, false);
        cortex_m4_set_register(cpu, 15u, addresses[index]);
        const CortexM4Result result = cortex_m4_step(cpu);
        expect(state, result.stop == CORTEX_M4_STOP_RUNNING &&
                          (cpu->system_pending & (1u << 3u)) != 0u,
               "Cortex-M0+ execute-never fetch raises HardFault");
    }
    cortex_m4_system_set_pending(cpu, 3u, false);
    cortex_m4_set_register(cpu, 15u, 0x100u);
}

static void test_mtb(TestState* state, Kinetis* device) {
    const uint16_t program[] = {0xe000u, 0xbf00u, 0xbf00u};
    expect(state, kinetis_load(device, 0x100u, program, sizeof(program)),
           "load MTB branch fixture");
    write32(state, device, MTB_POSITION, 0x7000u);
    write32(state, device, MTB_MASTER, 0x80000008u);
    expect(state, read32(state, device, MTB_BASE) == 0x1ffff000u,
           "MTB BASE identifies the split SRAM trace anchor");
    expect(state, read32(state, device, 0xf0000fe0u) == 0x32u &&
                      read32(state, device, 0xf0000fe4u) == 0xb9u &&
                      read32(state, device, 0xf0000fe8u) == 0x1bu,
           "MTB CoreSight peripheral identifiers match the silicon integration");
    expect(state, read32(state, device, 0xf0002000u) == 0xffffe003u &&
                      read32(state, device, 0xf0002004u) == 0xfffff003u &&
                      read32(state, device, 0xf0002008u) == 0xf00fd003u,
           "MTB ROM table exposes all three component entries");
    expect(state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "MTB branch instruction executes");
    expect(state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "MTB branch destination executes");
    expect(state, read32(state, device, 0x1ffff000u) == 0x100u,
           "MTB packet stores the branch source in lower SRAM");
    expect(state, read32(state, device, 0x1ffff004u) == 0x105u,
           "MTB packet stores the branch destination and first-packet flag");
    expect(state, (read32(state, device, MTB_POSITION) & 0x7ff8u) == 0x7008u,
           "MTB advances POSITION after a complete packet");

    write32(state, device, MTB_MASTER, 0x28u);
    write32(state, device, MTBDWT_COMP0, 0x104u);
    write32(state, device, MTBDWT_FCT0, 4u);
    write32(state, device, MTBDWT_TBCTRL, 1u);
    device->mtb_previous_valid = false;
    cortex_m4_set_register(kinetis_cpu(device), 15u, 0x104u);
    expect(state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "MTBDWT fetch comparator instruction executes");
    expect(state, (read32(state, device, MTB_MASTER) & 0x80000000u) != 0u,
           "MTBDWT fetch match starts tracing");
    expect(state, (read32(state, device, MTBDWT_FCT0) & 0x01000000u) != 0u,
           "MTBDWT reports the comparator match");
    expect(state, (read32(state, device, MTBDWT_FCT0) & 0x01000000u) == 0u,
           "MTBDWT MATCHED clears on read");

    write32(state, device, MTB_POSITION, 0x7000u);
    write32(state, device, MTB_MASTER, 0x188u);
    uint32_t blocked_value = UINT32_MAX;
    expect(state,
           kinetis_memory_read(device, 0x1ffff000u, 4u, CORTEX_M4_ACCESS_UNPRIVILEGED_DATA,
                               &blocked_value) &&
               blocked_value == 0u,
           "MTB RAMPRIV makes trace RAM read-as-zero to unprivileged software");
    expect(state,
           kinetis_memory_write(device, MTB_POSITION, 4u,
                                CORTEX_M4_ACCESS_UNPRIVILEGED_DATA, 0x7010u),
           "unprivileged MTB SFR write completes when SFRWPRIV is set");
    expect(state, read32(state, device, MTB_POSITION) == 0x7000u,
           "MTB SFRWPRIV ignores the unprivileged SFR write");

    device->mtb_autostop_latched = true;
    write32(state, device, MTB_MASTER, 0x80000008u);
    expect(state, (read32(state, device, MTB_MASTER) & 0x80000000u) == 0u,
           "watermark autostop blocks direct trace re-enable");
    const uint32_t stopped_position = read32(state, device, MTB_POSITION);
    write32(state, device, MTB_POSITION, stopped_position);
    write32(state, device, MTB_MASTER, 0x80000008u);
    expect(state, (read32(state, device, MTB_MASTER) & 0x80000000u) == 0u,
           "same-pointer POSITION write does not release watermark autostop");
    write32(state, device, MTB_FLOW, 0u);
    write32(state, device, MTB_MASTER, 0x80000008u);
    expect(state, (read32(state, device, MTB_MASTER) & 0x80000000u) != 0u,
           "clearing FLOW AUTOSTOP permits trace restart");

    write32(state, device, MTB_MASTER, 0x20u);
    write32(state, device, MTBDWT_TBCTRL, 1u);
    write32(state, device, MTBDWT_COMP0, 0x00000055u);
    write32(state, device, MTBDWT_FCT0, 0x107u);
    kinetis_internal_mtbdwt_data_access(device, 0x20000000u, 1u, false, 0x55u);
    expect(state, (read32(state, device, MTBDWT_FCT0) & 0x01000000u) == 0u,
           "byte watchpoint rejects a non-replicated comparator value");
    write32(state, device, MTBDWT_COMP0, 0x55555555u);
    kinetis_internal_mtbdwt_data_access(device, 0x20000000u, 1u, false, 0x55u);
    expect(state, (read32(state, device, MTBDWT_FCT0) & 0x01000000u) != 0u,
           "byte watchpoint accepts a replicated comparator value");
    cortex_m4_set_register(kinetis_cpu(device), 15u, 0x100u);
}

static void test_fast_gpio(TestState* state, Kinetis* device) {
    write32(state, device, TEST_SIM_SCGC5, read32(state, device, TEST_SIM_SCGC5) | (1u << 9u));
    write32(state, device, FGPIOA_PSOR, 0x25u);
    expect(state, read32(state, device, GPIOA_PDOR) == 0x25u,
           "read32(state, device, GPIOA_PDOR) == 0x25u");
    write32(state, device, GPIOA_PCOR, 0x21u);
    expect(state, read32(state, device, FGPIOA_PDOR) == 4u,
           "read32(state, device, FGPIOA_PDOR) == 4u");
}

static void test_bme(TestState* state, Kinetis* device) {
    CortexM4* cpu = kinetis_cpu(device);
    uint32_t value = 0u;
    write32(state, device, GPIOA_PDOR, 0x0fu);
    expect(state, cpu_store32(state, device, 0x4400f000u, 0x0au),
           "BME logical AND store succeeds");
    expect(state, read32(state, device, GPIOA_PDOR) == 0x0au,
           "BME logical AND updates the GPIO slot alias");
    expect(state, cpu_store32(state, device, 0x4800f000u, 0x05u),
           "BME logical OR store succeeds");
    expect(state, cpu_store32(state, device, 0x4c00f000u, 0x03u),
           "BME logical XOR store succeeds");
    expect(state, read32(state, device, GPIOA_PDOR) == 0x0cu,
           "BME logical OR and XOR update the target atomically");
    expect(state, cpu_load32(state, device, 0x4840f000u, &value) && value == 1u,
           "BME LAC1 returns the previous bit");
    expect(state, read32(state, device, GPIOA_PDOR) == 0x08u,
           "BME LAC1 clears the selected bit");
    expect(state, cpu_load32(state, device, 0x4c20f000u, &value) && value == 0u,
           "BME LAS1 returns the previous bit");
    expect(state, read32(state, device, GPIOA_PDOR) == 0x0au,
           "BME LAS1 sets the selected bit");
    expect(state, cpu_store32(state, device, 0x5010f000u, 0x05u),
           "BME BFI store succeeds");
    expect(state, read32(state, device, GPIOA_PDOR) == 0x0du,
           "BME BFI inserts the positioned field");
    expect(state, cpu_load32(state, device, 0x5010f000u, &value) && value == 5u,
           "BME UBFX returns a right-justified field");
    expect(state, !cortex_m4_read_memory(cpu, 0x43fe0000u, 4u, &value),
           "Cortex-M0+ rejects the peripheral bit-band alias");
    expect(state, !cortex_m4_read_memory(cpu, 0x40100000u, 4u, &value),
           "MKV10 rejects the reserved region above AIPS");
    cortex_m4_set_register(cpu, 15u, 0x100u);
}

static void test_cpu_only(TestState* state, Kinetis* device) {
    uint32_t value = 0u;
    expect(state, kinetis_memory_write(device, MCM_CPO, 4u, CORTEX_M4_ACCESS_DATA, 1u),
           "CPU requests CPU-only mode");
    expect(state, kinetis_memory_read(device, MCM_CPO, 4u, CORTEX_M4_ACCESS_DATA, &value) &&
                      value == 1u,
           "CPU-only acknowledge waits for staged entry");
    kinetis_advance(device, 1u);
    expect(state, kinetis_memory_read(device, MCM_CPO, 4u, CORTEX_M4_ACCESS_DATA, &value) &&
                      value == 3u,
           "CPU-only entry acknowledges after a clock edge");
    expect(state, !kinetis_timing_system_clock_running(&device->timing) &&
                      !kinetis_timing_bus_clock_running(&device->timing),
           "CPU-only mode gates the system and bus clocks");
    expect(state,
           !kinetis_memory_read(device, GPIOA_PDOR, 4u, CORTEX_M4_ACCESS_DATA, &value),
           "CPU-only mode rejects AIPS accesses");
    expect(state,
           kinetis_memory_read(device, FGPIOA_PDOR, 4u, CORTEX_M4_ACCESS_DATA, &value),
           "CPU-only mode keeps IOPORT available");
    expect(state, kinetis_memory_write(device, 0x20000000u, 4u, CORTEX_M4_ACCESS_DATA,
                                       0xa5a55a5au) &&
                      kinetis_memory_read(device, 0x20000000u, 4u, CORTEX_M4_ACCESS_DATA,
                                          &value) &&
                      value == 0xa5a55a5au,
           "CPU-only mode keeps SRAM available");
    expect(state, kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_DATA, &value),
           "CPU-only mode keeps flash reads available");
    expect(state, kinetis_memory_write(device, MCM_CPO, 4u, CORTEX_M4_ACCESS_DATA, 0u),
           "CPU clears the CPU-only request");
    expect(state, kinetis_memory_read(device, MCM_CPO, 4u, CORTEX_M4_ACCESS_DATA, &value) &&
                      value == 0u &&
                      kinetis_timing_bus_clock_running(&device->timing),
           "clearing CPU-only mode restores the bus clock immediately");
    expect(state, kinetis_memory_write(device, MCM_CPO, 4u, CORTEX_M4_ACCESS_DATA, 5u),
           "CPU-only mode enables wake on interrupt");
    kinetis_advance(device, 1u);
    kinetis_internal_exception_vector(device);
    expect(state, kinetis_memory_read(device, MCM_CPO, 4u, CORTEX_M4_ACCESS_DATA, &value) &&
                      value == 4u,
           "exception vectoring clears the CPU-only request and acknowledge");
    expect(state, !kinetis_read(device, MCM_CPO, &value, sizeof(value)) &&
                      !kinetis_write(device, MCM_CPO, &value, sizeof(value)),
           "DAP cannot access the CPU-only control register");
}

static void test_port_irqs(TestState* state, Kinetis* device) {
    write32(state, device, TEST_SIM_SCGC5,
            read32(state, device, TEST_SIM_SCGC5) | (1u << 9u) | (1u << 10u) | (1u << 13u));
    write32(state, device, PORTA_PCR0, 9u << 16u);
    write32(state, device, PORTB_PCR0, 9u << 16u);
    expect(state, kinetis_gpio_drive(device, 0u, 0u, true),
           "kinetis_gpio_drive(device, 0u, 0u, true)");
    expect(state, (device->cpu->irq_level[0] & (1u << 30u)) != 0u, "PORTA drives IRQ 30");
    expect(state, (device->cpu->irq_level[0] & (1u << 31u)) == 0u,
           "PORTA does not drive the shared PORTB through PORTE IRQ");
    expect(state, kinetis_gpio_drive(device, 1u, 0u, true),
           "kinetis_gpio_drive(device, 1u, 0u, true)");
    expect(state, (device->cpu->irq_level[0] & (1u << 31u)) != 0u,
           "PORTB drives the shared PORTB through PORTE IRQ 31");
    write32(state, device, PORTA_PCR0 + 20u * 4u, 1u << 8u);
    write32(state, device, PORTE_PCR0 + 20u * 4u, 1u << 8u);
    write32(state, device, PORTE_PCR0 + 24u * 4u, 1u << 8u);
    write32(state, device, PORTE_PCR0 + 29u * 4u, 1u << 8u);
    expect(state, read32(state, device, PORTE_PCR0 + 29u * 4u) == 1u << 8u,
           "LH package exposes the motor firmware's high PORTE pins");
}

static void test_pdb_instances(TestState* state, Kinetis* device) {
    write32(state, device, TEST_SIM_SCGC6,
            read32(state, device, TEST_SIM_SCGC6) | (1u << 17u) | (1u << 22u));
    write32(state, device, PDB0_MOD, 0x1234u);
    write32(state, device, PDB1_MOD, 0x5678u);
    write32(state, device, PDB0_SC, 0x10f81u);
    write32(state, device, PDB1_SC, 0x10fa1u);
    expect(state, read32(state, device, PDB0_MOD) == 0x1234u,
           "read32(state, device, PDB0_MOD) == 0x1234u");
    expect(state, read32(state, device, PDB1_MOD) == 0x5678u,
           "read32(state, device, PDB1_MOD) == 0x5678u");
    expect(state, read32(state, device, PDB0_SC) == 0xf80u,
           "read32(state, device, PDB0_SC) == 0xf80u");
    expect(state, read32(state, device, PDB1_SC) == 0xfa0u,
           "read32(state, device, PDB1_SC) == 0xfa0u");

    device->timing.pdb_sc[0] = 0x60u;
    device->timing.pdb_sc[1] = 0x60u;
    kinetis_timing_internal_refresh_pdb_irq(&device->timing);
    expect(state, (device->cpu->irq_level[0] & (1u << 29u)) != 0u, "PDB0 and PDB1 share IRQ 29");
    write32(state, device, PDB0_SC, 0x21u);
    expect(state, (device->cpu->irq_level[0] & (1u << 29u)) != 0u,
           "clearing PDB0 preserves an asserted PDB1 IRQ");
    write32(state, device, PDB1_SC, 0x21u);
    expect(state, (device->cpu->irq_level[0] & (1u << 29u)) == 0u,
           "clearing both PDB flags releases IRQ 29");

    write32(state, device, TEST_SIM_SCGC6, read32(state, device, TEST_SIM_SCGC6) | (1u << 24u));
    write32(state, device, PDB0_MOD, 9u);
    write32(state, device, PDB0_SC, 0x881u);
    write32(state, device, FTM0_EXTTRIG, 1u << 6u);
    write32(state, device, FTM0_CNT, 0u);
    expect(state, device->timing.pdb_running[0], "FTM0 initialization trigger starts PDB input 8");
    uint32_t value = 0u;
    expect(state, !kinetis_read(device, PDB0_DACINTC1, &value, sizeof(value)),
           "MKV10 rejects reserved second PDB DAC control");
    expect(state, !kinetis_write(device, PDB0_DACINT1, &value, sizeof(value)),
           "MKV10 rejects reserved second PDB DAC interval");
}

static bool pdb_starts_adc(Kinetis* device, uint8_t selection, uint8_t pdb_instance) {
    device->timing.sim_sopt7 = selection;
    device->data->adc[0].registers[0] = 0u;
    device->data->adc[0].registers[0x20] = 0x40u;
    device->data->adc[0].converting = false;
    kinetis_internal_timing_trigger(device, KINETIS_TIMING_TRIGGER_PDB_ADC, 0u, 0u, pdb_instance);
    return device->data->adc[0].converting;
}

static void test_pdb_adc_routing(TestState* state, Kinetis* device) {
    expect(state, pdb_starts_adc(device, 0x00u, 0u) && !pdb_starts_adc(device, 0x00u, 1u),
           "SOPT7 mode zero routes only PDB0 to ADC0");
    expect(state, !pdb_starts_adc(device, 0x40u, 0u) && pdb_starts_adc(device, 0x40u, 1u),
           "SOPT7 mode one routes only PDB1 to ADC0");
    expect(state, !pdb_starts_adc(device, 0x80u, 0u) && !pdb_starts_adc(device, 0x80u, 1u),
           "SOPT7 mode two reserves ADC0 for the alternate trigger");
    expect(state, pdb_starts_adc(device, 0xc0u, 0u) && pdb_starts_adc(device, 0xc0u, 1u),
           "SOPT7 mode three routes both PDB instances to ADC0");
    device->timing.sim_sopt7 = 0u;
    device->data->adc[0].registers[0x20] = 0u;
    device->data->adc[0].converting = false;
}

static void test_ftm_system_clock(TestState* state, Kinetis* device) {
    write32(state, device, TEST_SIM_SCGC6, read32(state, device, TEST_SIM_SCGC6) | (1u << 24u));
    write32(state, device, FTM0_SC, 0u);
    write32(state, device, FTM0_CNT, 0u);
    write32(state, device, FTM0_MOD, UINT16_MAX);
    write32(state, device, FTM0_SC, 1u << 3u);
    kinetis_advance(device, 1u);
    expect(state, read32(state, device, FTM0_CNT) == 1u,
           "MKV10 FTM advances from the system clock instead of the divided bus clock");
    write32(state, device, FTM0_SC, 0u);
}

static void test_ftm_fixed_clock_mux(TestState* state, Kinetis* device) {
    const uint32_t saved_sopt2 = device->timing.sim_sopt2;
    const uint8_t saved_c1 = device->timing.mcg[0];
    const uint8_t saved_c2 = device->timing.mcg[1];
    const uint8_t saved_osc = device->timing.osc_cr;
    device->timing.mcg[0] |= 2u;
    device->timing.mcg[1] &= 0xfeu;
    device->timing.sim_sopt2 = 1u << 24u;
    expect(state, kinetis_timing_internal_fixed_clock_hz(&device->timing) ==
                      device->timing.slow_irc_hz,
           "SOPT2 selects MCGIRCLK for the FTM fixed clock");
    device->timing.osc_cr |= 0x80u;
    device->timing.sim_sopt2 = 2u << 24u;
    expect(state, kinetis_timing_internal_fixed_clock_hz(&device->timing) ==
                      device->timing.external_oscillator_hz,
           "SOPT2 selects OSCERCLK for the FTM fixed clock");
    device->timing.sim_sopt2 = 3u << 24u;
    const uint32_t selection3 = kinetis_timing_internal_fixed_clock_hz(&device->timing);
    device->timing.sim_sopt2 = 0u;
    expect(state, kinetis_timing_internal_fixed_clock_hz(&device->timing) == selection3,
           "SOPT2 selections zero and three both select MCGFFCLK");
    device->timing.sim_sopt2 = saved_sopt2;
    device->timing.mcg[0] = saved_c1;
    device->timing.mcg[1] = saved_c2;
    device->timing.osc_cr = saved_osc;
}

static void test_ftm_external_clock_mux(TestState* state, Kinetis* device) {
    static const uint32_t bases[6] = {0x40038000u, 0x40039000u, 0x4003a000u,
                                      0x40026000u, 0x40027000u, 0x40028000u};
    write32(state, device, TEST_SIM_SCGC6,
            read32(state, device, TEST_SIM_SCGC6) | 0x070001c0u);
    write32(state, device, TEST_SIM_SOPT4, 0x24000000u);
    write32(state, device, TEST_SIM_SOPT6, 0x24000000u);
    for (uint8_t instance = 0u; instance < 6u; instance++) {
        write32(state, device, bases[instance], 0u);
        write32(state, device, bases[instance] + 4u, 0u);
        write32(state, device, bases[instance] + 8u, UINT16_MAX);
        write32(state, device, bases[instance], 3u << 3u);
    }
    for (uint8_t input = 0u; input < 3u; input++) {
        expect(state, kinetis_set_ftm_clock_input(device, input, true),
               "MKV10 accepts all three FTM external clock inputs");
        expect(state, kinetis_set_ftm_clock_input(device, input, false),
               "MKV10 accepts falling FTM external clock edges");
    }
    for (uint8_t instance = 0u; instance < 6u; instance++)
        expect(state, read32(state, device, bases[instance] + 4u) == 1u,
               "each MKV10 FTM uses its two-bit SOPT4 or SOPT6 clock selection");
    for (uint8_t instance = 0u; instance < 6u; instance++)
        write32(state, device, bases[instance], 0u);
    write32(state, device, TEST_SIM_SOPT4, 0u);
    write32(state, device, TEST_SIM_SOPT6, 0u);
}

static void test_ftm_crossbar(TestState* state, Kinetis* device) {
    write32(state, device, TEST_SIM_SOPT4, 1u << 22u);
    expect(state, kinetis_set_ftm_input(device, 1u, 1u, true), "set FTM1 channel 1 pin");
    expect(state, kinetis_set_ftm_input(device, 2u, 0u, true), "set FTM2 channel 0 pin");
    expect(state, kinetis_set_ftm_input(device, 2u, 1u, true), "set FTM2 channel 1 pin");
    expect(state, !device->timing.ftm[1].channel_input[1] &&
                      device->timing.ftm[2].channel_input[1],
           "FTM2 channel 1 XOR route consumes all three pins");
    expect(state, kinetis_set_ftm_input(device, 2u, 0u, false), "clear FTM2 channel 0 pin");
    expect(state, !device->timing.ftm[2].channel_input[1],
           "FTM2 channel 1 XOR route follows every source pin");

    device->comparator_output[0] = true;
    device->comparator_output[1] = true;
    write32(state, device, TEST_SIM_SOPT4, (1u << 2u) | (1u << 18u));
    expect(state, device->timing.ftm[1].fault_input[0] &&
                      device->timing.ftm[1].channel_input[0],
           "SOPT4 routes comparator 0 to FTM1 fault and capture inputs");
    device->comparator_output[0] = false;
    kinetis_sync_clock_gates(device);
    expect(state, !device->timing.ftm[1].fault_input[0] &&
                      !device->timing.ftm[1].channel_input[0],
           "internal FTM routes follow comparator levels");

    for (uint8_t instance = 0u; instance < 3u; instance++)
        device->timing.ftm[instance].hardware_trigger_pending_mask = 0u;
    write32(state, device, TEST_SIM_SOPT4, 1u << 8u);
    kinetis_internal_timing_trigger(device, KINETIS_TIMING_TRIGGER_PDB_FTM, 0u, 1u, 0u);
    expect(state, (device->timing.ftm[0].hardware_trigger_pending_mask & 2u) == 0u &&
                      (device->timing.ftm[1].hardware_trigger_pending_mask & 2u) != 0u &&
                      (device->timing.ftm[2].hardware_trigger_pending_mask & 2u) != 0u,
           "SOPT4 selects PDB or FTM for each hardware trigger 1 input");
    kinetis_internal_timing_trigger(device, KINETIS_TIMING_TRIGGER_FTM_OUTPUT, 2u, 0u, 2u);
    expect(state, (device->timing.ftm[0].hardware_trigger_pending_mask & 2u) != 0u,
           "FTM2 channel match routes to FTM0 hardware trigger 1");
    write32(state, device, TEST_SIM_SOPT4, 1u << 7u);
    kinetis_internal_timing_trigger(device, KINETIS_TIMING_TRIGGER_FTM_OUTPUT, 1u, 0u, 1u);
    expect(state, (device->timing.ftm[0].hardware_trigger_pending_mask & 1u) != 0u,
           "FTM1 channel match routes to FTM0 hardware trigger 0");

    KinetisFtmState* ftm = &device->timing.ftm[0];
    ftm->registers[1] = 1u << 4u;
    ftm->initial_buffer = 7u;
    ftm->initial_pending = true;
    ftm->modulo_buffer = 31u;
    ftm->modulo_pending = true;
    ftm->channel_value_buffer[0] = 13u;
    ftm->channel_value_pending[0] = true;
    write32(state, device, TEST_SIM_SOPT8, 1u);
    expect(state, ftm->initial == 7u && ftm->modulo == 31u && ftm->channel_value[0] == 13u &&
                      ftm->counter == 7u,
           "SOPT8 sync bit refreshes CNTIN and every FTM buffer on a zero-to-one edge");
    ftm->hardware_trigger_pending_mask = 0u;
    expect(state, kinetis_timing_trigger_ftm_hardware(&device->timing, 0u, 0u) &&
                      ftm->hardware_trigger_pending_mask == 0u,
           "an asserted SOPT8 sync bit masks FTM hardware trigger 0");
    write32(state, device, TEST_SIM_SOPT8, 0u);
    expect(state, kinetis_timing_trigger_ftm_hardware(&device->timing, 0u, 0u) &&
                      ftm->hardware_trigger_pending_mask == 1u,
           "clearing the SOPT8 sync bit restores FTM hardware trigger 0");

    bool output = true;
    device->timing.ftm[0].channel_output[0] = true;
    device->timing.ftm[1].channel_output[1] = false;
    write32(state, device, TEST_SIM_SOPT8, 1u << 16u);
    expect(state, kinetis_timing_get_ftm_output(&device->timing, 0u, 0u, &output) && !output,
           "SOPT8 carrier selection modulates the FTM0 output");
    device->timing.ftm[1].channel_output[1] = true;
    expect(state, kinetis_timing_get_ftm_output(&device->timing, 0u, 0u, &output) && output,
           "FTM carrier modulation follows its selected source");
    write32(state, device, TEST_SIM_SOPT9, 0x00ff0300u);
    expect(state, read32(state, device, TEST_SIM_SOPT9) == 0x00ff0300u,
           "SOPT9 stores all implemented carrier routing fields");
    write32(state, device, TEST_SIM_SOPT8, 0u);
    write32(state, device, TEST_SIM_SOPT9, 0u);
    write32(state, device, TEST_SIM_SOPT4, 0u);
}

static void test_llwu_layout(TestState* state, Kinetis* device) {
    device->timing.smc[3] = 0x20u;
    write8(state, device, LLWU_BASE + 5u, 0xffu);
    write8(state, device, LLWU_BASE + 6u, 0xffu);
    expect(state, (device->cpu->irq_level[0] & (1u << 7u)) == 0u,
           "programming KV10 PE6 and PE7 does not assert LLWU");
    write8(state, device, LLWU_BASE + 5u, 4u);
    expect(state, kinetis_timing_set_llwu_pin(&device->timing, 21u, true),
           "KV10 accepts routed LLWU pin 21");
    expect(state, (device->timing.llwu[11] & 0x20u) != 0u &&
                      (device->cpu->irq_level[0] & (1u << 7u)) != 0u,
           "LLWU pin 21 sets PF3 and IRQ7");
    write8(state, device, LLWU_BASE + 11u, 0x20u);
    expect(state, (device->cpu->irq_level[0] & (1u << 7u)) == 0u,
           "PF3 clears by writing one");
    expect(state, !kinetis_timing_set_llwu_pin(&device->timing, 1u, true) &&
                      !kinetis_timing_set_llwu_pin(&device->timing, 28u, true),
           "KV10 rejects reserved and unrouted LLWU pins");
    write8(state, device, LLWU_BASE + 8u, 2u);
    expect(state, kinetis_timing_trigger_llwu_module(&device->timing, 1u) &&
                      device->timing.llwu[13] == 2u,
           "KV10 ME and MF5 use their extended register offsets");
    write8(state, device, LLWU_BASE + 13u, 0xffu);
    expect(state, device->timing.llwu[13] == 2u, "MF5 is read-only");
    device->timing.llwu[13] = 0u;
    kinetis_timing_internal_update_llwu_irq(&device->timing);
    device->timing.smc[3] = 1u;
}

static void test_dmamux(TestState* state, Kinetis* device) {
    for (uint8_t channel = 0u; channel < 8u; channel++) {
        write8(state, device, DMAMUX_CHCFG0 + channel, 0xffu);
        expect(state, read8(state, device, DMAMUX_CHCFG0 + channel) == 0xbfu,
               "MKV10 DMAMUX periodic trigger bit is reserved");
    }
    expect(state, !kinetis_data_dma_trigger(device->data, 0u),
           "MKV10 rejects periodic DMA triggers");
}

static void test_lptmr(TestState* state) {
    Kinetis* device = create_device(state);
    if (device == NULL)
        return;
    write32(state, device, TEST_SIM_SCGC5, read32(state, device, TEST_SIM_SCGC5) | 1u);
    write32(state, device, LPTMR0_CSR, 0u);
    write32(state, device, LPTMR0_PSR, 5u);
    write32(state, device, LPTMR0_CMR, 0xffffu);
    write32(state, device, LPTMR0_CSR, 1u);
    device->timing.wdog[0] &= 0xfffeu;
    device->timing.wdog_initial_unlock_required = false;
    device->timing.wdog_update_open = false;
    device->timing.wdog_reset_pending = false;
    expect(state, (device->timing.sim_scgc5 & 1u) != 0u, "LPTMR clock gate is enabled");
    expect(state, device->timing.lptmr_psr == 5u, "LPTMR bypass selects the LPO clock");
    expect(state, device->timing.lptmr_csr == 1u, "LPTMR time counter is enabled");
    expect(state, device->timing.core_clock_hz != 0u, "LPTMR has a core timing reference");
    expect(state, device->timing.lpo_hz == 1000u, "LPTMR has its LPO clock source");
    expect(state,
           kinetis_timing_internal_has(&device->timing, KINETIS_PERIPHERAL_LPTMR0),
           "MKV10 provides LPTMR0");
    const uint32_t half_period = device->timing.core_clock_hz / 2000u + 1u;
    const bool first = device->timing.lptmr_prescaler_output;
    kinetis_timing_advance(&device->timing, half_period);
    expect(state, (device->timing.sim_scgc5 & 1u) != 0u,
           "LPTMR clock gate remains enabled while advancing");
    expect(state, device->timing.lptmr_csr == 1u, "LPTMR remains enabled while advancing");
    expect(state, device->timing.lptmr_prescaler_remainder != 0u,
           "LPTMR tracks subcycle prescaler phase");
    const bool second = device->timing.lptmr_prescaler_output;
    expect(state, second != first,
           "LPTMR prescaler carrier changes after a half period");
    kinetis_timing_advance(&device->timing, half_period);
    const bool third = device->timing.lptmr_prescaler_output;
    expect(state, third != second,
           "LPTMR prescaler carrier changes independently of CNR");
    expect(state, device->timing.lptmr_counter == 0u,
           "MKV10 LPTMR waits for two synchronization clocks");
    kinetis_timing_advance(&device->timing, device->timing.core_clock_hz / 1000u + 1u);
    expect(state, device->timing.lptmr_counter == 0u,
           "MKV10 LPTMR waits for the second synchronization clock");
    kinetis_timing_advance(&device->timing, device->timing.core_clock_hz / 1000u + 1u);
    expect(state, device->timing.lptmr_counter == 1u,
           "MKV10 LPTMR increments after synchronization");
    device->timing.debug_halted = true;
    kinetis_timing_advance(&device->timing, device->timing.core_clock_hz / 1000u + 1u);
    expect(state, device->timing.lptmr_counter == 1u,
           "MKV10 LPTMR time counter stops while the core is halted");
    device->timing.debug_halted = false;
    write32(state, device, LPTMR0_CSR, 0u);
    kinetis_destroy(device);
}

static void test_cmp_modes(TestState* state, Kinetis* device) {
    KinetisData* data = device->data;
    kinetis_data_set_clocks(data, 100u, 50u, 0u, 0u, true, KINETIS_DATA_STOP_NONE);
    expect(state, kinetis_data_set_cmp_input(data, 0u, 1u, 30u),
           "set CMP positive input");
    expect(state, kinetis_data_set_cmp_input(data, 0u, 2u, 10u),
           "set CMP negative input");
    data_write8(state, data, CMP0_MUXCR, 0x0au);
    data_write8(state, data, CMP0_CR0, 0x30u);
    data_write8(state, data, CMP0_FPR, 2u);
    data_write8(state, data, CMP0_SCR, 0x40u);
    data->dmamux[0] = 0x80u | 42u;
    kinetis_data_internal_store_bytes(data->dma, 0x0cu, 2u, 1u);
    data_write8(state, data, CMP0_CR1, 1u);
    kinetis_data_advance(data, 4u);
    expect(state, (data_read8(state, data, CMP0_SCR) & 1u) == 0u,
           "CMP waits for consecutive internal filter samples");
    kinetis_data_advance(data, 8u);
    expect(state, (data_read8(state, data, CMP0_SCR) & 5u) == 5u,
           "CMP commits after three samples spaced by FPR bus clocks");
    expect(state, !device->data_interrupt[KINETIS_DATA_INTERRUPT_CMP0],
           "CMP DMA does not require an interrupt enable");
    expect(state, (data->dma_requests & 1u) != 0u,
           "CMP edge flag requests DMA independently of IER and IEF");

    data->dma_requests = 0u;
    data_write8(state, data, CMP0_SCR, 0x46u);
    data_write8(state, data, CMP0_CR0, 0x10u);
    data_write8(state, data, CMP0_CR1, 0x41u);
    kinetis_data_internal_cmp_set_sample(data, 0u, true);
    kinetis_data_advance(data, 4u);
    expect(state, (data_read8(state, data, CMP0_SCR) & 1u) != 0u,
           "windowed CMP samples while WINDOW is high");
    kinetis_data_internal_cmp_set_sample(data, 0u, false);
    expect(state, kinetis_data_set_cmp_input(data, 0u, 1u, 0u),
           "change CMP input while WINDOW is low");
    kinetis_data_advance(data, 8u);
    expect(state, (data_read8(state, data, CMP0_SCR) & 1u) != 0u,
           "windowed CMP holds its latch while WINDOW is low");
    kinetis_data_internal_cmp_set_sample(data, 0u, true);
    kinetis_data_advance(data, 4u);
    expect(state, (data_read8(state, data, CMP0_SCR) & 1u) == 0u,
           "windowed CMP resamples after WINDOW rises");
}

static void test_cmp_trigger_mode(TestState* state) {
    Kinetis* device = create_device(state);
    if (device == NULL)
        return;
    KinetisData* data = device->data;
    KinetisTiming* timing = &device->timing;
    kinetis_data_set_clocks(data, timing->core_clock_hz, timing->bus_clock_hz, 0u, 0u, true,
                            KINETIS_DATA_STOP_NONE);
    expect(state, kinetis_data_set_cmp_input(data, 0u, 1u, 30u),
           "set trigger-mode CMP positive input");
    expect(state, kinetis_data_set_cmp_input(data, 0u, 2u, 10u),
           "set trigger-mode CMP negative input");
    data_write8(state, data, CMP0_MUXCR, 0x0au);
    data_write8(state, data, CMP0_CR1, 0x21u);
    timing->wdog[0] &= 0xfffeu;
    timing->wdog_initial_unlock_required = false;
    timing->sim_scgc5 |= 1u;
    timing->lptmr_psr = 5u;
    timing->lptmr_cmr = 0u;
    timing->lptmr_csr = 1u;
    timing->lptmr_sync_ticks = 0u;
    const uint32_t bypass_delay = kinetis_timing_lptmr_trigger_delay_cycles(timing);
    kinetis_timing_advance(timing, (timing->core_clock_hz + 999u) / 1000u);
    expect(state, data->cmp[0].trigger_pending && data->cmp[0].trigger_cycles == bypass_delay,
           "LPTMR TCF starts the first CMP trigger stage");
    kinetis_data_advance(data, bypass_delay - 1u);
    expect(state, (data_read8(state, data, CMP0_SCR) & 1u) == 0u,
           "CMP stays in standby before the bypass half-clock delay");
    kinetis_data_advance(data, 1u);
    expect(state, (data_read8(state, data, CMP0_SCR) & 1u) != 0u,
           "CMP captures at the second bypass trigger stage");

    data_write8(state, data, CMP0_CR1, 0u);
    data_write8(state, data, CMP0_SCR, 6u);
    data_write8(state, data, CMP0_CR1, 0x21u);
    timing->lptmr_psr = 1u;
    timing->lptmr_remainder = 0u;
    timing->lptmr_counter = 0u;
    const uint32_t prescaled_delay = kinetis_timing_lptmr_trigger_delay_cycles(timing);
    kinetis_timing_advance(timing, (timing->core_clock_hz * 2u + 999u) / 1000u);
    expect(state, prescaled_delay == bypass_delay * 2u &&
                      data->cmp[0].trigger_cycles == prescaled_delay,
           "prescaled CMP trigger uses half the prescaler-output period");
    kinetis_destroy(device);
}

static void test_dac_modes(TestState* state, Kinetis* device) {
    KinetisData* data = device->data;
    data_write8(state, data, DAC0_DAT0L, 0x11u);
    data_write8(state, data, DAC0_DAT0L + 1u, 1u);
    data_write8(state, data, DAC0_DAT1L, 0x22u);
    data_write8(state, data, DAC0_DAT1L + 1u, 2u);
    data_write8(state, data, DAC0_C0, 0x80u);
    data_write8(state, data, DAC0_C1, 1u);
    data_write8(state, data, DAC0_C2, 1u);
    uint16_t output = 0u;
    expect(state, kinetis_data_get_dac_output(data, 0u, &output) && output == 0x111u,
           "MKV10 DAC begins at buffer word zero");
    kinetis_data_dac_trigger(data, 0u);
    expect(state, kinetis_data_get_dac_output(data, 0u, &output) && output == 0x222u,
           "MKV10 DAC normal mode advances to its upper word");
    expect(state, (data_read8(state, data, DAC0_SR) & 1u) != 0u,
           "DAC bottom-position flag means RP equals UP");
    kinetis_data_dac_trigger(data, 0u);
    expect(state, kinetis_data_get_dac_output(data, 0u, &output) && output == 0x111u,
           "MKV10 DAC normal mode wraps only after reaching UP");
    expect(state, (data_read8(state, data, DAC0_SR) & 2u) != 0u,
           "DAC top-position flag means RP is zero");
    data_write8(state, data, DAC0_SR, 0xfeu);
    expect(state, (data_read8(state, data, DAC0_SR) & 3u) == 2u,
           "MKV10 DAC status flags clear by writing zero");

    data_write8(state, data, DAC0_C1, 5u);
    data_write8(state, data, DAC0_C2, 1u);
    kinetis_data_dac_trigger(data, 0u);
    kinetis_data_dac_trigger(data, 0u);
    expect(state, (data_read8(state, data, DAC0_C2) & 0x10u) != 0u,
           "MKV10 DAC one-time mode stops at UP");
    data_write8(state, data, DAC0_C1, 1u);
    data_write8(state, data, DAC0_C2, 1u);
    data_write8(state, data, DAC0_C0, 0xb0u);
    expect(state, (data_read8(state, data, DAC0_C0) & 0x10u) == 0u &&
                      (data_read8(state, data, DAC0_C2) & 0x10u) != 0u,
           "DAC software trigger is write-only and advances with DACTRGSEL");
    kinetis_data_dac_trigger(data, 0u);
    expect(state, (data_read8(state, data, DAC0_C2) & 0x10u) != 0u,
           "DAC ignores hardware triggers while software trigger is selected");

    data_write8(state, data, DAC0_SR, 0u);
    data_write8(state, data, DAC0_C0, 0x84u);
    data_write8(state, data, DAC0_DAT0L, 0x33u);
    expect(state, (data_read8(state, data, DAC0_SR) & 4u) != 0u,
           "writing the two-word DAC buffer reaches its one-word watermark");

    data_write8(state, data, DAC0_SR, 0u);
    data_write8(state, data, DAC0_C0, 0x81u);
    data_write8(state, data, DAC0_C1, 0x81u);
    data_write8(state, data, DAC0_C2, 1u);
    data->dmamux[1] = 0x80u | 45u;
    kinetis_data_internal_store_bytes(data->dma, 0x0cu, 2u, 2u);
    kinetis_data_dac_trigger(data, 0u);
    expect(state, (data->dma_requests & 2u) != 0u &&
                      !device->data_interrupt[KINETIS_DATA_INTERRUPT_DAC0],
           "DAC DMA replaces the module interrupt");
    kinetis_data_internal_dac_dma_complete(data, 45u);
    expect(state, (data_read8(state, data, DAC0_SR) & 7u) == 0u,
           "DAC DMA completion automatically clears status flags");

    data_write8(state, data, DAC0_C0, 0x80u);
    data_write8(state, data, DAC0_C1, 1u);
    data_write8(state, data, DAC0_C2, 1u);
    kinetis_internal_timing_trigger(device, KINETIS_TIMING_TRIGGER_PDB_DAC, 1u, 0u, 0u);
    expect(state, (data_read8(state, data, DAC0_C2) & 0x10u) != 0u,
           "PDB1 interval trigger zero routes to the sole DAC0");
}

static void test_power_modes(TestState* state) {
    Kinetis* device = create_device(state);
    if (device == NULL)
        return;
    KinetisData* data = device->data;
    data->adc[0].registers[0x10] = 0x5au;
    data->adc[0].registers[8] = 0u;
    data->adc[0].converting = true;
    data->adc[0].remaining_cycles = 10u;
    kinetis_data_set_clocks(data, 100u, 50u, 0u, 0u, false, KINETIS_DATA_STOP_NORMAL);
    expect(state, !data->adc[0].converting && data->adc[0].registers[0x10] == 0x5au,
           "normal Stop aborts a bus-clocked ADC and preserves Rn");
    data->adc[0].registers[0x10] = 0x5au;
    data->adc[0].registers[8] = 3u;
    data->adc[0].converting = true;
    data->adc[0].remaining_cycles = 10u;
    kinetis_data_set_clocks(data, 100u, 50u, 0u, 0u, false, KINETIS_DATA_STOP_VLPS);
    expect(state, !data->adc[0].converting && data->adc[0].registers[0x10] == 0u,
           "low-power Stop aborts every ADC source and resets Rn");

    data->stop_mode = KINETIS_DATA_STOP_NONE;
    data_write8(state, data, DAC0_DAT0L, 0x55u);
    data_write8(state, data, DAC0_C0, 0x80u);
    uint16_t output = 0u;
    expect(state, kinetis_data_get_dac_output(data, 0u, &output) && output == 0x55u,
           "DAC produces its configured output before Stop");
    kinetis_data_set_clocks(data, 100u, 50u, 0u, 0u, false, KINETIS_DATA_STOP_VLPS);
    expect(state, kinetis_data_get_dac_output(data, 0u, &output) && output == 0x55u,
           "DAC output and registers remain static in low-power Stop");

    write8(state, device, SMC_PMCTRL, 2u);
    expect(state, read8(state, device, SMC_PMCTRL) == 0u,
           "MKV10 blocks VLPS selection without AVLP");
    device->timing.smc[1] = 2u;
    write8(state, device, SMC_PMPROT, 0x20u);
    expect(state, (read8(state, device, SMC_PMCTRL) & 7u) == 0u,
           "the first PMPROT write clears STOPM");
    write8(state, device, SMC_PMPROT, 2u);
    expect(state, read8(state, device, SMC_PMPROT) == 0x20u,
           "MKV10 PMPROT accepts only its first write");
    write8(state, device, SMC_PMCTRL, 2u);
    expect(state, read8(state, device, SMC_PMCTRL) == 2u,
           "MKV10 accepts VLPS after AVLP is enabled");
    write8(state, device, SMC_PMCTRL, 4u);
    expect(state, read8(state, device, SMC_PMCTRL) == 2u,
           "MKV10 blocks VLLS selection without AVLLS");

    device->timing.debug_halted = true;
    kinetis_timing_set_cpu_sleeping(&device->timing, true, true);
    expect(state, read8(state, device, SMC_PMSTAT) == 1u &&
                      kinetis_timing_power_status(&device->timing) == 0x10u,
           "debug Stop leaves PMSTAT at RUN while entering VLPS");
    kinetis_timing_set_cpu_sleeping(&device->timing, false, false);
    device->timing.debug_halted = false;
    write8(state, device, SMC_PMCTRL, 0u);
    write8(state, device, SMC_STOPCTRL, 0x40u);
    kinetis_timing_set_cpu_sleeping(&device->timing, true, true);
    expect(state, read8(state, device, SMC_PMSTAT) == 1u &&
                      kinetis_timing_power_status(&device->timing) == 2u,
           "partial Stop leaves PMSTAT at RUN while entering Stop");
    kinetis_timing_set_cpu_sleeping(&device->timing, false, false);

    write8(state, device, SMC_PMCTRL, 2u);
    write8(state, device, SMC_STOPCTRL, 1u);
    kinetis_timing_warm_reset(&device->timing, 0u, 4u);
    expect(state, read8(state, device, SMC_PMPROT) == 0u &&
                      read8(state, device, SMC_PMCTRL) == 2u &&
                      read8(state, device, SMC_STOPCTRL) == 1u,
           "warm reset preserves PMCTRL and STOPCTRL but resets PMPROT");

    write8(state, device, SMC_PMPROT, 2u);
    write8(state, device, SMC_PMCTRL, 4u);
    write8(state, device, SMC_STOPCTRL, 3u);
    kinetis_timing_set_cpu_sleeping(&device->timing, true, true);
    kinetis_timing_warm_reset(&device->timing, 1u, 0u);
    expect(state, read8(state, device, SMC_PMPROT) == 2u &&
                      read8(state, device, SMC_PMCTRL) == 4u &&
                      read8(state, device, SMC_STOPCTRL) == 3u &&
                      read8(state, device, SMC_PMSTAT) == 1u,
           "VLLS reset retains SMC protection and control registers");
    kinetis_destroy(device);
}

static void test_sim_access_controls(TestState* state) {
    Kinetis* device = create_device(state);
    if (device == NULL)
        return;
    const struct {
        uint32_t address;
        uint8_t size;
    } restricted[] = {
        {0x40064000u, 1u}, {0x4007f004u, 1u}, {TEST_SIM_CLKDIV1, 4u},
        {0x4007e001u, 1u}, {0x4007c000u, 1u}, {0x4007d002u, 1u},
    };
    for (size_t index = 0u; index < sizeof(restricted) / sizeof(restricted[0]); index++) {
        uint32_t value = 0u;
        expect(state,
               kinetis_memory_read(device, restricted[index].address, restricted[index].size,
                                   CORTEX_M4_ACCESS_UNPRIVILEGED_DATA, &value),
               "unprivileged software can read a protected system block");
        expect(state,
               !kinetis_memory_write(device, restricted[index].address, restricted[index].size,
                                     CORTEX_M4_ACCESS_UNPRIVILEGED_DATA, UINT32_MAX),
               "unprivileged software cannot write a protected system block");
    }

    write32(state, device, TEST_SIM_CLKDIV1, 0x10030000u);
    device->timing.smc[3] = 4u;
    write32(state, device, TEST_SIM_CLKDIV1, 0u);
    expect(state, read32(state, device, TEST_SIM_CLKDIV1) == 0x10030000u,
           "MKV10 ignores CLKDIV1 writes in VLPR");
    device->timing.smc[3] = 1u;

    write32(state, device, TEST_SIM_FCFG1, UINT32_MAX);
    expect(state, read32(state, device, TEST_SIM_FCFG1) == 0x07000003u,
           "MKV10 FCFG1 exposes PFSIZE, FLASHDOZE, and FLASHDIS");
    uint32_t value = 0u;
    expect(state,
           !kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_DATA, &value),
           "FLASHDIS rejects processor Flash reads");
    expect(state, kinetis_read(device, 0x100u, &value, 2u),
           "debug access remains available while FLASHDIS is set");

    write32(state, device, TEST_SIM_FCFG1, 2u);
    expect(state,
           kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_DATA, &value),
           "FLASHDOZE leaves Flash enabled in Run");
    kinetis_timing_set_cpu_sleeping(&device->timing, true, false);
    expect(state,
           !kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_DATA, &value),
           "FLASHDOZE rejects Flash reads in Wait");
    kinetis_timing_set_cpu_sleeping(&device->timing, false, false);
    device->timing.smc[3] = 4u;
    device->timing.smc_run_status = 4u;
    kinetis_timing_set_cpu_sleeping(&device->timing, true, false);
    expect(state,
           kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_DATA, &value),
           "FLASHDOZE does not disable Flash in VLPW");
    kinetis_destroy(device);
}

static void test_watchdog_sources(TestState* state) {
    Kinetis* device = create_device(state);
    if (device == NULL)
        return;
    KinetisTiming* timing = &device->timing;
    timing->wdog_initial_unlock_required = false;
    timing->wdog[0] = 1u;
    timing->wdog[2] = 0xffffu;
    timing->wdog[3] = 0xffffu;
    timing->wdog[11] = 0u;
    timing->wdog_counter = 0u;
    kinetis_timing_advance(timing, (timing->core_clock_hz + 999u) / 1000u);
    expect(state, timing->wdog_counter == 1u,
           "MKV10 WDOG default clock uses the LPO");
    timing->wdog_counter = 0u;
    timing->wdog_remainder = 0u;
    timing->wdog[0] = 0x2001u;
    kinetis_timing_advance(timing, (timing->core_clock_hz + 999u) / 1000u);
    expect(state, timing->wdog_counter == 1u,
           "WDOG BYTESEL does not select the alternate clock");
    timing->wdog_counter = 0u;
    timing->wdog_remainder = 0u;
    write32(state, device, SIM_WDOGC, 2u);
    timing->mcg[0] |= 2u;
    const uint32_t mcgir_hz = kinetis_timing_internal_mcgir_clock_hz(timing);
    kinetis_timing_advance(timing, (timing->core_clock_hz + mcgir_hz - 1u) / mcgir_hz);
    expect(state, timing->wdog_counter == 1u,
           "SIM_WDOGC selects MCGIRCLK for the WDOG default source");
    timing->wdog_counter = 0u;
    timing->wdog_remainder = 0u;
    timing->wdog[0] = 3u;
    kinetis_timing_advance(timing,
                           (timing->core_clock_hz + timing->bus_clock_hz - 1u) /
                               timing->bus_clock_hz);
    expect(state, timing->wdog_counter == 1u,
           "WDOG CLKSRC selects the alternate bus clock");
    kinetis_destroy(device);
}

static void test_flash_access_control(TestState* state) {
    Kinetis* device = create_device(state);
    if (device == NULL)
        return;
    KinetisData* data = device->data;
    for (uint32_t offset = 0x1cu; offset <= 0x1fu; offset++)
        expect(state, data_read8(state, data, FTFA + offset) == 0xffu,
               "MKV10 XACC reset value comes from erased IFR records");
    for (uint32_t offset = 0x24u; offset <= 0x27u; offset++)
        expect(state, data_read8(state, data, FTFA + offset) == 0xffu,
               "MKV10 SACC reset value comes from erased IFR records");
    expect(state, data_read8(state, data, FTFA + 0x28u) == 4u,
           "MKV10 FACSS reports four kilobytes");
    expect(state, data_read8(state, data, FTFA + 0x2bu) == 0x20u,
           "MKV10 FACSN reports 32 segments");

    data->flash[0x1fu] &= 0xfeu;
    uint32_t value = 0u;
    expect(state, kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_INSTRUCTION, &value),
           "execute-only Flash permits instruction fetches");
    expect(state, !kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_DATA, &value),
           "execute-only Flash rejects data reads");
    data->flash[0x1fu] |= 1u;
    data->flash[0x27u] &= 0xfeu;
    expect(state,
           !kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_UNPRIVILEGED_DATA,
                                &value),
           "supervisor-only Flash rejects unprivileged data reads");
    cortex_m4_set_control(device->cpu, CORTEX_M4_CONTROL_NPRIV);
    expect(state,
           !kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_INSTRUCTION, &value),
           "supervisor-only Flash rejects unprivileged instruction fetches");
    cortex_m4_set_control(device->cpu, 0u);
    expect(state, kinetis_memory_read(device, 0x100u, 2u, CORTEX_M4_ACCESS_DATA, &value),
           "supervisor-only Flash permits privileged reads");
    data->flash[0x27u] |= 1u;

    data->flash[0x1fu] &= 0xfeu;
    execute_flash_command(state, data, 0x4au, 0u, 40u);
    expect(state, (data_read8(state, data, FTFA) & 1u) != 0u &&
                      !data->flash_access_control_disabled,
           "Read 1s All Execute-only reports a programmed execute-only segment");
    data->flash[0x1fu] |= 1u;
    data->flash[0x1fu] &= 0xfdu;
    bool segment_erased = true;
    for (uint32_t offset = 0x1000u; offset < 0x2000u; offset++)
        segment_erased &= device->flash[offset] == 0xffu;
    expect(state, segment_erased, "selected execute-only segment starts erased");
    execute_flash_command(state, data, 0x4au, 0u, 40u);
    expect(state, (data_read8(state, data, FTFA) & 1u) == 0u,
           "Read 1s All Execute-only verifies an erased execute-only segment");
    expect(state, data->flash_access_control_disabled,
           "Read 1s All Execute-only releases access control after successful verification");

    write_fccob(state, data, 2u, 0x10u);
    write_fccob(state, data, 3u, 0u);
    write_fccob(state, data, 4u, 0u);
    write_fccob(state, data, 5u, 0u);
    write_fccob(state, data, 6u, 0u);
    write_fccob(state, data, 7u, 0u);
    execute_flash_command(state, data, 0x06u, 0u, 40u);
    expect(state, data->flash_execute_access_programmed,
           "programming a released execute-only segment arms access restoration");
    execute_flash_command(state, data, 0x4au, 0u, 40u);
    expect(state, !data->flash_access_control_disabled,
           "failed execute-only verification restores access control after programming");

    data->flash[0x10u] &= 0xfdu;
    execute_flash_command(state, data, 0x4bu, 0u, 2000u);
    expect(state, (data_read8(state, data, FTFA) & 0x10u) != 0u,
           "Erase All Execute-only rejects protected execute-only segments");
    data->flash[0x10u] |= 2u;
    execute_flash_command(state, data, 0x4bu, 0u, 2000u);
    expect(state, device->flash[0x1000u] == 0xffu,
           "Erase All Execute-only erases selected segments");
    expect(state, data->flash_access_control_disabled,
           "Erase All Execute-only releases access control");

    const uint8_t record_four[4] = {0x11u, 0x22u, 0x33u, 0x44u};
    for (uint8_t index = 0u; index < sizeof(record_four); index++)
        write_fccob(state, data, (uint8_t)(4u + index), record_four[index]);
    execute_flash_command(state, data, 0x43u, 0x0fu, 40u);
    expect(state, data->flash_program_ifr[0xfcu] == 0x44u &&
                      data->flash_program_ifr[0xffu] == 0x11u,
           "Program Once stores four-byte records in FCCOB byte order");
    const uint8_t record_eight[8] = {0x10u, 0x21u, 0x32u, 0x43u,
                                     0x54u, 0x65u, 0x76u, 0x87u};
    for (uint8_t index = 0u; index < sizeof(record_eight); index++)
        write_fccob(state, data, (uint8_t)(4u + index), record_eight[index]);
    execute_flash_command(state, data, 0x43u, 0x10u, 40u);
    expect(state, data->flash_program_ifr[0xa0u] == 0x87u &&
                      data->flash_program_ifr[0xa7u] == 0x10u,
           "Program Once stores eight-byte access records at IFR 0xA0");
    execute_flash_command(state, data, 0x41u, 0x10u, 40u);
    for (uint8_t index = 0u; index < sizeof(record_eight); index++)
        expect(state, data_read8(state, data, flash_fccob_address((uint8_t)(4u + index))) ==
                          record_eight[index],
               "Read Once returns access records in FCCOB byte order");

    execute_flash_command(state, data, 0x4au, 3u, 40u);
    expect(state, (data_read8(state, data, FTFA) & 0x20u) != 0u,
           "Read 1s All Execute-only rejects invalid margin levels");
    data_write8(state, data, FTFA, 0x70u);
    write_fccob(state, data, 0u, 0x41u);
    write_fccob(state, data, 1u, 0u);
    kinetis_data_set_vlp_mode(data, true);
    data_write8(state, data, FTFA, 0x80u);
    expect(state, (data_read8(state, data, FTFA) & 0x80u) != 0u && data->flash_cycles == 0u,
           "MKV10 ignores Flash command launches in VLP modes");
    kinetis_data_set_vlp_mode(data, false);
    data_write8(state, data, FTFA, 0x80u);
    expect(state, (data_read8(state, data, FTFA) & 0x80u) == 0u,
           "MKV10 clears CCIF when a Flash command launches after VLP");
    expect(state, data->flash_cycles != 0u,
           "MKV10 schedules a Flash command after leaving VLP modes");
    kinetis_data_advance(data, 40u);
    kinetis_destroy(device);
}

static void test_mcg_auto_trim(TestState* state) {
    Kinetis* device = create_device(state);
    if (device == NULL)
        return;
    device->timing.wdog[0] = 0u;
    device->timing.wdog_initial_unlock_required = false;
    device->timing.external_oscillator_hz = 8000000u;
    write32(state, device, TEST_SIM_CLKDIV1, 0u);
    write8(state, device, MCG_C1, 0x80u);
    write8(state, device, MCG_C3, 0x55u);
    write8(state, device, MCG_C4, 0x0au);
    write8(state, device, MCG_ATCVH, 0x14u);
    write8(state, device, MCG_ATCVL, 0x04u);
    write8(state, device, MCG_SC, 0xa2u);
    expect(state, device->timing.core_clock_hz == 8000000u &&
                      device->timing.bus_clock_hz == 8000000u,
           "automatic trim uses the configured 8 MHz bus reference");
    expect(state, device->timing.mcg_trim_remaining_bus_cycles == 46116u,
           "slow IRC automatic trim schedules nine ATCV comparison windows");
    expect(state, (read8(state, device, MCG_SC) & 0xa0u) == 0x80u,
           "slow IRC automatic trim starts and clears its previous failure flag");
    kinetis_timing_advance(&device->timing, 46115u);
    expect(state, device->timing.mcg_trim_remaining_bus_cycles == 1u,
           "slow IRC automatic trim consumes bus-reference cycles");
    expect(state, (read8(state, device, MCG_SC) & 0x80u) != 0u,
           "slow IRC automatic trim remains active through its comparison sequence");
    kinetis_timing_advance(&device->timing, 1u);
    expect(state, (read8(state, device, MCG_SC) & 0xa0u) == 0u &&
                      read8(state, device, MCG_C3) == 0x55u &&
                      read8(state, device, MCG_C4) == 0x0au,
           "slow IRC automatic trim completes normally at the configured target");

    write8(state, device, MCG_ATCVH, 0x15u);
    write8(state, device, MCG_ATCVL, 0x00u);
    write8(state, device, MCG_SC, 0xe2u);
    kinetis_timing_advance(&device->timing, 21503u);
    expect(state, (read8(state, device, MCG_SC) & 0x80u) != 0u,
           "fast IRC automatic trim remains active through four SAR decisions");
    kinetis_timing_advance(&device->timing, 1u);
    expect(state, (read8(state, device, MCG_SC) & 0xa0u) == 0u,
           "fast IRC automatic trim completes normally");

    write8(state, device, MCG_SC, 0xa2u);
    write8(state, device, MCG_C3, 0x33u);
    expect(state, (read8(state, device, MCG_SC) & 0xa0u) == 0x20u,
           "writing a trim register aborts automatic trim and sets ATMF");
    write8(state, device, MCG_SC, 0x20u);
    expect(state, (read8(state, device, MCG_SC) & 0x20u) == 0u,
           "ATMF clears when software writes one");

    write8(state, device, MCG_SC, 0xa2u);
    kinetis_timing_set_cpu_sleeping(&device->timing, true, true);
    expect(state, (read8(state, device, MCG_SC) & 0xa0u) == 0x20u,
           "entering Stop aborts automatic trim and sets ATMF");
    kinetis_destroy(device);
}

static void test_uart_shared_irqs(TestState* state, Kinetis* device) {
    for (uint8_t instance = 0u; instance < 2u; instance++) {
        KinetisSerialUart* uart = &device->serial.uart[instance];
        const uint32_t irq_mask = 1u << (12u + instance);
        uart->clock_enabled = true;
        uart->registers[UART_C2] = 0x80u;
        uart->registers[UART_C3] = 0u;
        kinetis_internal_refresh_serial_signals(device);
        expect(state, (device->cpu->irq_level[0] & irq_mask) != 0u,
               "MKV10 UART status source asserts its shared IRQ");
        uart->registers[UART_C2] = 0u;
        uart->registers[UART_S1] |= 1u;
        uart->registers[UART_C3] = 1u;
        kinetis_internal_refresh_serial_signals(device);
        expect(state, (device->cpu->irq_level[0] & irq_mask) != 0u,
               "MKV10 UART error source asserts its shared IRQ");
        uart->registers[UART_S1] &= 0xf0u;
        uart->registers[UART_C3] = 0u;
        kinetis_internal_refresh_serial_signals(device);
        expect(state, (device->cpu->irq_level[0] & irq_mask) == 0u,
               "MKV10 UART shared IRQ clears after both sources clear");
    }
}

static void divide(TestState* state, Kinetis* device, uint32_t dividend, uint32_t divisor,
                   uint32_t mode) {
    write32(state, device, MMDVSQ_CSR, mode);
    write32(state, device, MMDVSQ_DEND, dividend);
    write32(state, device, MMDVSQ_DSOR, divisor);
    write32(state, device, MMDVSQ_CSR, mode | 1u);
}

static void test_mmdvsq(TestState* state, Kinetis* device) {
    divide(state, device, (uint32_t)-17, 5u, 0u);
    expect(state, read32(state, device, MMDVSQ_RES) == (uint32_t)-3,
           "read32(state, device, MMDVSQ_RES) == (uint32_t)-3");
    divide(state, device, (uint32_t)-17, 5u, 4u);
    expect(state, read32(state, device, MMDVSQ_RES) == (uint32_t)-2,
           "read32(state, device, MMDVSQ_RES) == (uint32_t)-2");
    divide(state, device, UINT32_MAX, 2u, 2u);
    expect(state, read32(state, device, MMDVSQ_RES) == UINT32_MAX / 2u,
           "read32(state, device, MMDVSQ_RES) == UINT32_MAX / 2u");
    divide(state, device, (uint32_t)INT32_MIN, UINT32_MAX, 0u);
    expect(state, read32(state, device, MMDVSQ_RES) == (uint32_t)INT32_MIN,
           "read32(state, device, MMDVSQ_RES) == (uint32_t)INT32_MIN");
    divide(state, device, 7u, 0u, 2u);
    expect(state, (read32(state, device, MMDVSQ_CSR) & 0x40000010u) == 0x40000010u,
           "(read32(state, device, MMDVSQ_CSR) & 0x40000010u) == 0x40000010u");
    expect(state, read32(state, device, MMDVSQ_RES) == 0u,
           "read32(state, device, MMDVSQ_RES) == 0u");
    divide(state, device, 7u, 0u, 0xau);
    uint32_t result = 0u;
    expect(state, !kinetis_read(device, MMDVSQ_RES, &result, sizeof(result)),
           "!kinetis_read(device, MMDVSQ_RES, &result, sizeof(result))");
    write32(state, device, MMDVSQ_RCND, 65535u);
    expect(state, !kinetis_read(device, MMDVSQ_RCND, &result, sizeof(result)),
           "!kinetis_read(device, MMDVSQ_RCND, &result, sizeof(result))");
    expect(state, read32(state, device, MMDVSQ_RES) == 255u,
           "read32(state, device, MMDVSQ_RES) == 255u");
    expect(state, (read32(state, device, MMDVSQ_CSR) & 0x60000010u) == 0x20000000u,
           "(read32(state, device, MMDVSQ_CSR) & 0x60000010u) == 0x20000000u");
    write32(state, device, MMDVSQ_RES, 0x12345678u);
    expect(state, read32(state, device, MMDVSQ_RES) == 0x12345678u,
           "read32(state, device, MMDVSQ_RES) == 0x12345678u");
    write32(state, device, MMDVSQ_CSR, 0x20u);
    write32(state, device, MMDVSQ_DEND, 0x100u);
    write32(state, device, MMDVSQ_DSOR, 3u);
    write32(state, device, MMDVSQ_CSR, 0x21u);
    expect(state, (read32(state, device, MMDVSQ_CSR) & 0x80000000u) != 0u,
           "MMDVSQ reports busy during a multi-cycle divide");
    kinetis_advance(device, 5u);
    expect(state, (read32(state, device, MMDVSQ_CSR) & 0x80000000u) != 0u,
           "MMDVSQ remains busy for the documented dividend-dependent latency");
    kinetis_advance(device, 1u);
    expect(state, (read32(state, device, MMDVSQ_CSR) & 0x80000000u) == 0u,
           "MMDVSQ clears busy at the documented completion cycle");
    expect(state, read32(state, device, MMDVSQ_RES) == 0x55u,
           "MMDVSQ publishes the divide result at completion");
    write32(state, device, MMDVSQ_CSR, 0x20u);
    write32(state, device, MMDVSQ_DEND, 0x100u);
    write32(state, device, MMDVSQ_DSOR, 3u);
    write32(state, device, MMDVSQ_CSR, 0x21u);
    const uint16_t load_result = 0x6801u;
    expect(state, kinetis_load(device, 0x100u, &load_result, sizeof(load_result)),
           "load MMDVSQ result read instruction");
    cortex_m4_set_register(kinetis_cpu(device), 0u, MMDVSQ_RES);
    const uint64_t cycles_before = cortex_m4_get_cycle_count(kinetis_cpu(device));
    const CortexM4Result cpu_result = cortex_m4_step(kinetis_cpu(device));
    expect(state, cpu_result.stop == CORTEX_M4_STOP_RUNNING,
           "MMDVSQ result read executes without a fault");
    expect(state, cortex_m4_get_register(kinetis_cpu(device), 1u) == 0x55u,
           "MMDVSQ stalled read receives the completed result");
    expect(state, cortex_m4_get_cycle_count(kinetis_cpu(device)) - cycles_before >= 7u,
           "MMDVSQ stalled read charges the remaining hardware cycles");
}

int main(void) {
    TestState state = {0};
    Kinetis* device = create_device(&state);
    if (device != NULL) {
        test_core(&state, device);
        test_reset_pin_filter(&state, device);
        test_execute_never(&state, device);
        test_mtb(&state, device);
        test_fast_gpio(&state, device);
        test_bme(&state, device);
        test_cpu_only(&state, device);
        test_port_irqs(&state, device);
        test_ftm_system_clock(&state, device);
        test_ftm_fixed_clock_mux(&state, device);
        test_ftm_external_clock_mux(&state, device);
        test_ftm_crossbar(&state, device);
        test_llwu_layout(&state, device);
        test_dmamux(&state, device);
        test_lptmr(&state);
        test_cmp_modes(&state, device);
        test_cmp_trigger_mode(&state);
        test_dac_modes(&state, device);
        test_power_modes(&state);
        test_sim_access_controls(&state);
        test_flash_access_control(&state);
        test_watchdog_sources(&state);
        test_mcg_auto_trim(&state);
        test_uart_shared_irqs(&state, device);
        test_pdb_instances(&state, device);
        test_pdb_adc_routing(&state, device);
        test_mmdvsq(&state, device);
    }
    kinetis_destroy(device);
    return test_finish(&state);
}
