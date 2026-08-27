#include "kinetis.h"

#include <stdint.h>

#include "kinetis_test.h"
#include "test.h"

enum {
    LPUART0_BAUD = 0x4002a000u,
    LPUART0_CTRL = 0x4002a008u,
    LPUART0_DATA = 0x4002a00cu,
    LPTMR0_CSR = 0x40040000u,
    MCG_C1 = 0x40064000u,
    MCG_C5 = 0x40064004u,
    MCG_C6 = 0x40064005u,
    MCG_S = 0x40064006u,
    MCG_C7 = 0x4006400cu,
    RTC_CR = 0x4003d010u,
    SMC_PMPROT = 0x4007e000u,
    SMC_PMCTRL = 0x4007e001u,
    SMC_PMSTAT = 0x4007e003u,
    SIM_SCGC5 = 0x40048038u,
    SIM_SCGC6 = 0x4004803cu,
    SIM_SOPT2 = 0x40048004u,
};

static void write_register(TestState* state, Kinetis* device, uint32_t address, uint8_t size,
                           uint32_t value) {
    expect(state, kinetis_write(device, address, &value, size),
           "kinetis_write(device, address, &value, size)");
}

static uint32_t read_register(TestState* state, Kinetis* device, uint32_t address, uint8_t size) {
    uint32_t output_value = 0;
    expect(state, kinetis_read(device, address, &output_value, size),
           "kinetis_read(device, address, &output_value, size)");
    return output_value;
}

int main(void) {
    TestState state = {0};
    Kinetis* device = kinetis_create(kinetis_configuration(KINETIS_PROFILE_MK22FN51212));
    expect(&state, device != NULL, "device != NULL");
    expect(&state, kinetis_test_disable_watchdog(device), "watchdog disabled for clock tests");

    write_register(&state, device, SMC_PMPROT, 1, 0x80u);
    write_register(&state, device, SMC_PMCTRL, 1, 0x60u);
    expect(&state, read_register(&state, device, SMC_PMSTAT, 1) == 0x80u,
           "read_register(&state, device, SMC_PMSTAT, 1) == 0x80u");
    write_register(&state, device, SMC_PMCTRL, 1, 0);
    expect(&state, read_register(&state, device, SMC_PMSTAT, 1) == 1,
           "read_register(&state, device, SMC_PMSTAT, 1) == 1");

    write_register(&state, device, MCG_C1, 1, 0x32u);
    expect(&state, (read_register(&state, device, MCG_S, 1) & 0x1cu) == 0,
           "(read_register(&state, device, MCG_S, 1) & 0x1cu) == 0");
    write_register(&state, device, MCG_C6, 1, 0);
    expect(&state, (read_register(&state, device, MCG_S, 1) & 0x1cu) == 0,
           "(read_register(&state, device, MCG_S, 1) & 0x1cu) == 0");

    write_register(&state, device, SIM_SCGC5, 4, read_register(&state, device, SIM_SCGC5, 4) | 1u);
    write_register(&state, device, LPTMR0_CSR, 4, 1);
    expect(&state, read_register(&state, device, LPTMR0_CSR, 4) == 1u,
           "read_register(&state, device, LPTMR0_CSR, 4) == 1u");
    write_register(&state, device, LPTMR0_CSR, 4, 0);
    expect(&state, read_register(&state, device, LPTMR0_CSR, 4) == 0,
           "read_register(&state, device, LPTMR0_CSR, 4) == 0");

    write_register(&state, device, SIM_SCGC5, 4, 0xa55ac33cu);
    expect(&state, read_register(&state, device, SIM_SCGC5, 4) == 0x00040382u,
           "read_register(&state, device, SIM_SCGC5, 4) == 0x00040382u");

    expect(&state,
           cortex_m4_write_memory(kinetis_cpu(device), SIM_SCGC6, 4,
                                  read_register(&state, device, SIM_SCGC6, 4) | (1u << 10)),
           "firmware enables the LPUART clock gate");
    expect(&state, (read_register(&state, device, SIM_SCGC6, 4) & (1u << 10)) != 0u,
           "LPUART clock gate remains enabled");
    write_register(&state, device, LPUART0_BAUD, 4, 0x0f000001u);
    write_register(&state, device, LPUART0_CTRL, 4, 1u << 19);
    write_register(&state, device, LPUART0_DATA, 4, 0x5au);
    kinetis_advance(device, 1000u);
    uint16_t transmitted_value = 0u;
    expect(&state, !kinetis_serial_transmit(device, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "reset LPUART clock selection prevents transmission");
    write_register(&state, device, SIM_SOPT2, 4, 1u << 26);
    kinetis_advance(device, 1000u);
    expect(&state, kinetis_serial_transmit(device, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "selected LPUART source completes transmission");

    kinetis_destroy(device);

    device = kinetis_create(kinetis_configuration(KINETIS_PROFILE_MK22FN51212));
    expect(&state, device != NULL, "PLL-clocked LPUART device is created");
    expect(&state, kinetis_test_disable_watchdog(device), "PLL-clocked LPUART watchdog disabled");
    write_register(&state, device, MCG_C1, 1u, 0x80u);
    write_register(&state, device, MCG_C5, 1u, 1u);
    write_register(&state, device, MCG_C6, 1u, 0x40u);
    expect(&state, (read_register(&state, device, MCG_S, 1u) & 0x60u) == 0x20u,
           "PLL is enabled but unlocked during acquisition");
    kinetis_advance(device, 3349u);
    expect(&state, (read_register(&state, device, MCG_S, 1u) & (1u << 6u)) == 0u,
           "PLL remains unlocked through its penultimate reference clock");
    kinetis_advance(device, 1u);
    expect(&state, (read_register(&state, device, MCG_S, 1u) & (1u << 6u)) != 0u,
           "PLL locks after its acquisition interval");
    write_register(&state, device, MCG_C1, 1u, 0u);
    write_register(&state, device, SIM_SOPT2, 4u, (1u << 26u) | (1u << 16u));
    write_register(&state, device, SIM_SCGC6, 4u,
                   read_register(&state, device, SIM_SCGC6, 4u) | (1u << 10u));
    write_register(&state, device, LPUART0_BAUD, 4u, 0x0f000001u);
    write_register(&state, device, LPUART0_CTRL, 4u, 1u << 19u);
    write_register(&state, device, LPUART0_DATA, 4u, 0x5au);
    kinetis_advance(device, 159u);
    expect(&state, !kinetis_serial_transmit(device, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "PLL-clocked LPUART frame remains active through its penultimate clock");
    kinetis_advance(device, 1u);
    expect(&state, kinetis_serial_transmit(device, KINETIS_SERIAL_LPUART0, &transmitted_value),
           "PLL-clocked LPUART frame completes on its final clock");
    expect(&state, transmitted_value == 0x5au, "PLL-clocked LPUART transmits its data");
    kinetis_destroy(device);

    device = kinetis_create(kinetis_configuration(KINETIS_PROFILE_MK22FN51212));
    expect(&state, device != NULL, "PLL Stop-wake device is created");
    uint32_t sleep_vectors[17] = {0u};
    sleep_vectors[0] = 0x20001000u;
    sleep_vectors[1] = 0x101u;
    sleep_vectors[16] = 0x105u;
    const uint16_t sleep_program[] = {0xbf30u, 0xbf00u, 0x4770u};
    expect(&state, kinetis_load(device, 0u, sleep_vectors, sizeof(sleep_vectors)),
           "PLL Stop-wake vectors load");
    expect(&state, kinetis_load(device, 0x100u, sleep_program, sizeof(sleep_program)),
           "PLL Stop-wake program loads");
    expect(&state, kinetis_reset(device), "PLL Stop-wake device resets");
    expect(&state, kinetis_test_disable_watchdog(device), "PLL Stop-wake watchdog disabled");
    write_register(&state, device, MCG_C1, 1u, 0x80u);
    write_register(&state, device, MCG_C5, 1u, 1u);
    write_register(&state, device, MCG_C6, 1u, 0x40u);
    kinetis_advance(device, 3350u);
    write_register(&state, device, MCG_C1, 1u, 0u);
    expect(&state, kinetis_core_clock_hz(device) == 96000000u, "Stop-wake test enters PEE");
    expect(&state, cortex_m4_write_memory(kinetis_cpu(device), 0xe000ed10u, 4u, 4u),
           "firmware enables deep sleep");
    expect(&state, cortex_m4_write_memory(kinetis_cpu(device), 0xe000e100u, 4u, 1u),
           "firmware enables its wake interrupt");
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "firmware executes WFI from PEE");
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_CLOCK,
           "normal Stop disables an unretained PLL");
    cortex_m4_set_irq(kinetis_cpu(device), 0u, true);
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "the wake interrupt restores the clock and executes without a host advance");
    expect(&state, kinetis_core_clock_hz(device) == 8000000u,
           "Stop wake forces PBE to restore the core clock");
    expect(&state, (read_register(&state, device, MCG_S, 1u) & 0x4cu) == 0x08u,
           "Stop wake reports external MCG clock while PLL reacquires");
    kinetis_advance(device, 3350u);
    expect(&state, (read_register(&state, device, MCG_S, 1u) & 0x40u) != 0u,
           "wake PLL reacquires without deadlocking the core");
    kinetis_destroy(device);

    KinetisConfiguration mkv30_configuration = kinetis_configuration(KINETIS_PROFILE_MKV30F12810);
    device = kinetis_create(mkv30_configuration);
    expect(&state, device != NULL, "MKV30 device is created");
    const uint32_t mkv30_reset_clock = kinetis_core_clock_hz(device);
    write_register(&state, device, MCG_C6, 1u, 0x40u);
    expect(&state, read_register(&state, device, MCG_C6, 1u) == 0u,
           "MKV30 reserved MCG_C6 bits read as zero");
    expect(&state, kinetis_core_clock_hz(device) == mkv30_reset_clock,
           "MKV30 reserved MCG_C6 bits do not select a PLL");
    kinetis_destroy(device);

    KinetisConfiguration missing_oscillator = kinetis_configuration(KINETIS_PROFILE_MK22FN51212);
    missing_oscillator.external_oscillator_hz = 0u;
    device = kinetis_create(missing_oscillator);
    expect(&state, device != NULL, "missing-oscillator device is created");
    const uint32_t vectors[2] = {0x20001000u, 0x101u};
    const uint16_t nop = 0xbf00u;
    expect(&state, kinetis_load(device, 0u, vectors, sizeof(vectors)), "clock test vectors load");
    expect(&state, kinetis_load(device, 0x100u, &nop, sizeof(nop)), "clock test program loads");
    expect(&state, kinetis_reset(device), "clock test device resets");
    write_register(&state, device, MCG_C1, 1u, 0u);
    expect(&state, kinetis_core_clock_hz(device) == 0u,
           "external-reference FLL has no output without its source");
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_CLOCK,
           "the core stops when its selected clock disappears");
    expect(&state,
           cortex_m4_get_register(kinetis_cpu(device), 15u) == 0x100u &&
               cortex_m4_get_instruction_count(kinetis_cpu(device)) == 0u,
           "clock loss retires no instruction");
    write_register(&state, device, MCG_C1, 1u, 0x40u);
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "the core resumes when the host restores a valid clock");
    expect(&state, cortex_m4_get_instruction_count(kinetis_cpu(device)) == 1u,
           "the resumed core retires its pending instruction");
    write_register(&state, device, MCG_C1, 1u, 0x80u);
    expect(&state, kinetis_core_clock_hz(device) == 0u,
           "external clock has no output without its source");
    write_register(&state, device, MCG_C6, 1u, 0x40u);
    write_register(&state, device, MCG_C1, 1u, 0u);
    expect(&state, kinetis_core_clock_hz(device) == 0u,
           "PLL without an external oscillator has no usable output");
    expect(&state, (read_register(&state, device, MCG_S, 1u) & (1u << 6u)) == 0u,
           "PLL without an external oscillator remains unlocked");
    kinetis_destroy(device);

    device = kinetis_create(missing_oscillator);
    expect(&state, device != NULL, "IRC48M PLL device is created");
    expect(&state, kinetis_test_disable_watchdog(device), "IRC48M PLL watchdog disabled");
    write_register(&state, device, MCG_C7, 1u, 2u);
    write_register(&state, device, MCG_C1, 1u, 0x80u);
    write_register(&state, device, MCG_C5, 1u, 23u);
    write_register(&state, device, MCG_C6, 1u, 0x58u);
    kinetis_advance(device, 32999u);
    expect(&state, (read_register(&state, device, MCG_S, 1u) & (1u << 6u)) == 0u,
           "IRC48M PLL observes its full acquisition interval");
    kinetis_advance(device, 1u);
    write_register(&state, device, MCG_C1, 1u, 0u);
    expect(&state, kinetis_core_clock_hz(device) == 96000000u,
           "PLL uses its selected IRC48M reference");
    expect(&state, (read_register(&state, device, MCG_S, 1u) & (1u << 6u)) != 0u,
           "valid IRC48M PLL configuration locks");
    kinetis_destroy(device);

    device = kinetis_create(kinetis_configuration(KINETIS_PROFILE_MK22FN51212));
    expect(&state, device != NULL, "RTC-clocked instruction device is created");
    const uint16_t rtc_program[] = {0x4802u, 0x4903u, 0x6001u, 0xbf00u, 0xbf00u, 0xbf00u};
    const uint32_t rtc_literals[] = {RTC_CR, 0x300u};
    expect(&state, kinetis_load(device, 0u, vectors, sizeof(vectors)), "RTC vectors load");
    expect(&state, kinetis_load(device, 0x100u, rtc_program, sizeof(rtc_program)),
           "RTC clock program loads");
    expect(&state, kinetis_load(device, 0x10cu, rtc_literals, sizeof(rtc_literals)),
           "RTC clock literals load");
    expect(&state, kinetis_reset(device), "RTC clock device resets");
    expect(&state, kinetis_test_disable_watchdog(device), "RTC clock watchdog disabled");
    write_register(&state, device, SIM_SCGC6, 4u,
                   read_register(&state, device, SIM_SCGC6, 4u) | (1u << 29u));
    write_register(&state, device, RTC_CR, 4u, 0x100u);
    write_register(&state, device, MCG_C7, 1u, 1u);
    write_register(&state, device, MCG_C1, 1u, 0x80u);
    expect(&state, kinetis_core_clock_hz(device) == 32768u, "RTC output clocks the core");
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "RTC program loads its target register");
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "RTC program loads its clock-control value");
    expect(&state,
           cortex_m4_get_register(kinetis_cpu(device), 0u) == RTC_CR &&
               cortex_m4_get_register(kinetis_cpu(device), 1u) == 0x300u,
           "RTC program loaded its register address and control value");
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "the RTC clock-disabling instruction retires normally");
    expect(&state,
           cortex_m4_get_register(kinetis_cpu(device), 15u) == 0x106u &&
               cortex_m4_get_instruction_count(kinetis_cpu(device)) == 3u,
           "RTC clock loss occurs after exactly one control-register write");
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_CLOCK,
           "clockless core performs no following fetch");
    expect(&state,
           cortex_m4_get_register(kinetis_cpu(device), 15u) == 0x106u &&
               cortex_m4_get_instruction_count(kinetis_cpu(device)) == 3u,
           "clockless core state remains unchanged");
    write_register(&state, device, RTC_CR, 4u, 0x100u);
    expect(&state, cortex_m4_step(kinetis_cpu(device)).stop == CORTEX_M4_STOP_RUNNING,
           "restoring RTC output resumes the pending instruction");
    expect(&state, cortex_m4_get_instruction_count(kinetis_cpu(device)) == 4u,
           "restored RTC clock retires the pending instruction");
    kinetis_destroy(device);

    device = kinetis_create(kinetis_configuration(KINETIS_PROFILE_MK22FN51212));
    expect(&state, device != NULL, "invalid PLL device is created");
    write_register(&state, device, MCG_C5, 1u, 1u);
    write_register(&state, device, MCG_C6, 1u, 0x5fu);
    write_register(&state, device, MCG_C1, 1u, 0u);
    expect(&state, kinetis_core_clock_hz(device) == 0u, "out-of-range PLL has no usable output");
    expect(&state, (read_register(&state, device, MCG_S, 1u) & (1u << 6u)) == 0u,
           "out-of-range PLL remains unlocked");
    kinetis_destroy(device);
    return test_finish(&state);
}
