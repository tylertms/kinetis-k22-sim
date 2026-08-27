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
    write_register(&state, device, MCG_C5, 1u, 1u);
    write_register(&state, device, MCG_C6, 1u, 0x40u);
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
    write_register(&state, device, MCG_C1, 1u, 0u);
    expect(&state, kinetis_core_clock_hz(device) == 32768u * 640u,
           "external-reference FLL falls back to the slow internal clock");
    write_register(&state, device, MCG_C6, 1u, 0x40u);
    expect(&state, kinetis_core_clock_hz(device) == 1u,
           "PLL without an external oscillator has no usable output");
    expect(&state, (read_register(&state, device, MCG_S, 1u) & (1u << 6u)) == 0u,
           "PLL without an external oscillator remains unlocked");
    kinetis_destroy(device);
    return test_finish(&state);
}
