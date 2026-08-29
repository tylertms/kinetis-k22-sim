#include "device/kinetis/timing/support.h"

static void irq_signal(void* context, uint8_t irq, bool asserted) {
    Observations* observations = context;
    observations->irq[irq] = asserted;
    if (asserted)
        observations->irq_assertions[irq]++;
    observations->irq_changes++;
}

static void dma_signal(void* context, uint8_t source) {
    Observations* observations = context;
    observations->dma_requests++;
    observations->last_dma = source;
}

static void dma_trigger_signal(void* context, uint8_t channel) {
    Observations* observations = context;
    observations->dma_triggers[channel]++;
}

static void reset_signal(void* context, uint8_t srs0, uint8_t srs1) {
    Observations* observations = context;
    observations->resets++;
    observations->last_srs0 = srs0;
    observations->last_srs1 = srs1;
}

static void trigger_signal(void* context, KinetisTimingTrigger trigger, uint8_t instance,
                           uint8_t channel, uint8_t source_instance) {
    Observations* observations = context;
    if (trigger == KINETIS_TIMING_TRIGGER_PDB_ADC)
        observations->adc_triggers++;
    else if (trigger == KINETIS_TIMING_TRIGGER_PDB_DAC)
        observations->dac_triggers++;
    else if (trigger == KINETIS_TIMING_TRIGGER_ADC_ALTERNATE)
        observations->alternate_triggers++;
    else
        observations->pdb_ftm_triggers++;
    observations->last_trigger_instance = instance;
    observations->last_trigger_channel = channel;
    observations->last_trigger_source = source_instance;
}

KinetisTimingSignals kinetis_timing_test_signals(Observations* observations) {
    KinetisTimingSignals value = {
        .context = observations,
        .irq = irq_signal,
        .dma = dma_signal,
        .reset = reset_signal,
        .trigger = trigger_signal,
        .dma_trigger = dma_trigger_signal,
    };
    return value;
}

static uint16_t lptmr_ticks_in_one_second(KinetisTiming timing, uint32_t prescaler) {
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    timing.sim_scgc5 |= 1u;
    timing.lptmr_psr = prescaler | 4u;
    timing.lptmr_cmr = 0xffffu;
    timing.lptmr_csr = 1u;
    timing.lptmr_counter = 0u;
    timing.lptmr_remainder = 0u;
    kinetis_timing_advance(&timing, timing.core_clock_hz);
    return timing.lptmr_counter;
}

void kinetis_timing_test_expect_read(TestState* state, KinetisTiming* timing, uint32_t address,
                                     uint8_t size, uint32_t expected) {
    uint32_t value = 0xdeadbeefu;
    expect(state, kinetis_timing_read(timing, address, size, &value),
           "kinetis_timing_read(timing, address, size, &value)");
    if (value != expected) {
        fprintf(stderr, "address 0x%08x: expected 0x%08x, got 0x%08x\n", address, expected, value);
    }
    expect(state, value == expected, "value == expected");
}

void kinetis_timing_test_expect_write(TestState* state, KinetisTiming* timing, uint32_t address,
                                      uint8_t size, uint32_t value) {
    expect(state, kinetis_timing_write(timing, address, size, value),
           "kinetis_timing_write(timing, address, size, value)");
}

uint32_t kinetis_timing_test_cycles_for_ticks(const KinetisTiming* timing, uint32_t ticks,
                                              uint32_t clock_hz) {
    return (uint32_t)(((uint64_t)timing->core_clock_hz * ticks + clock_hz - 1u) / clock_hz);
}

void kinetis_timing_test_disable_watchdog_fixture(KinetisTiming* timing) {
    timing->wdog[0] = 0u;
    timing->wdog_pending[0] = 0u;
    timing->wdog_initial_unlock_required = false;
    timing->wdog_update_open = false;
    timing->wdog_reset_pending = false;
}

void kinetis_timing_test_test_profiles_and_reset(TestState* state) {
    static const uint32_t expected_fcfg1[KINETIS_PROFILE_COUNT] = {
        0x0f0f0f00u, 0x0f0f0f00u, 0x07000000u, 0x0f0f0f00u, 0x0f0f0f00u,
        0x090f0f00u, 0x0f0f0f00u, 0xff0f0f00u, 0xff0f0f00u,
    };
    static const uint32_t expected_fcfg2[KINETIS_PROFILE_COUNT] = {
        0x10000000u, 0x10800000u, 0x10800000u, 0x10000000u, 0x20000000u,
        0x20000000u, 0x20200000u, 0x40c00000u, 0x40100000u,
    };
    for (KinetisProfile id = 0; id < KINETIS_PROFILE_COUNT; id++) {
        KinetisTiming timing;
        Observations observations = {0};
        const KinetisDeviceProfile* profile = kinetis_profile_get(id);
        expect(state, profile != NULL, "profile != NULL");
        expect(state,
               kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                                   kinetis_timing_test_signals(&observations)),
               "kinetis_timing_init(&timing, profile, 8000000u, 32768u, "
               "kinetis_timing_test_signals(&observations))");
        kinetis_timing_test_expect_read(state, &timing, SIM_SDID, 4, profile->sim_sdid_reset);
        kinetis_timing_test_expect_read(state, &timing, SIM_FCFG2, 4, expected_fcfg2[id]);
        kinetis_timing_test_expect_read(state, &timing, 0x4004804cu, 4, expected_fcfg1[id]);
        expect(state,
               kinetis_timing_read(&timing, SIM_SCGC3, 4, &(uint32_t){0}) ==
                   (id == KINETIS_PROFILE_MK22FN1M012 || id == KINETIS_PROFILE_MK22FX51212),
               "kinetis_timing_read(&timing, SIM_SCGC3, 4, &(uint32_t){0}) == (id == "
               "KINETIS_PROFILE_MK22FN1M012 || id == KINETIS_PROFILE_MK22FX51212)");
        if (id != KINETIS_PROFILE_MK22FN1M012 && id != KINETIS_PROFILE_MK22FX51212)
            expect(state, !kinetis_timing_write(&timing, SIM_SCGC3, 4, 0u),
                   "profile without SCGC3 rejects the register");
        kinetis_timing_test_expect_read(state, &timing, RCM_SRS0, 1, 0x82u);
        if (kinetis_profile_has_peripheral(profile, KINETIS_PERIPHERAL_PIT))
            kinetis_timing_test_expect_read(
                state, &timing, 0x40037000u, 4,
                id == KINETIS_PROFILE_MK22FN1M012 || id == KINETIS_PROFILE_MK22FX51212 ? 2u : 6u);
        else
            expect(state, !kinetis_timing_read(&timing, 0x40037000u, 4, &(uint32_t){0}),
                   "profile without PIT rejects the register");
        if (kinetis_profile_has_peripheral(profile, KINETIS_PERIPHERAL_RTC))
            kinetis_timing_test_expect_read(state, &timing, 0x4003d014u, 4, 1u);
        else
            expect(state, !kinetis_timing_read(&timing, 0x4003d014u, 4, &(uint32_t){0}),
                   "profile without RTC rejects the register");
        kinetis_timing_test_expect_read(state, &timing, 0x40052000u, 2, 0x01d3u);
        expect(state, kinetis_timing_core_clock_hz(&timing) == 20971520u,
               "kinetis_timing_core_clock_hz(&timing) == 20971520u");
        expect(state,
               kinetis_timing_bus_clock_hz(&timing) ==
                   (id == KINETIS_PROFILE_MKV10Z1287 ? 10485760u : 20971520u),
               "reset bus clock matches the device clock tree");
        expect(state, !kinetis_timing_read(&timing, MCG_S + 1u, 1, &(uint32_t){0}),
               "!kinetis_timing_read(&timing, MCG_S + 1u, 1, &(uint32_t){0})");
        expect(state, !kinetis_timing_read(&timing, SIM_SDID, 1, &(uint32_t){0}),
               "!kinetis_timing_read(&timing, SIM_SDID, 1, &(uint32_t){0})");
        expect(state, !kinetis_timing_write(&timing, SIM_SDID, 4, 0),
               "!kinetis_timing_write(&timing, SIM_SDID, 4, 0)");
    }
}

void kinetis_timing_test_test_clock_tree_and_power(TestState* state, KinetisTiming* timing) {
    KinetisTiming waiting = *timing;
    kinetis_timing_set_cpu_sleeping(&waiting, true, false);
    expect(state, kinetis_timing_system_clock_running(&waiting), "system clock runs in Wait");
    expect(state, kinetis_timing_bus_clock_running(&waiting), "bus clock runs in Wait");

    KinetisTiming stopped = *timing;
    kinetis_timing_set_cpu_sleeping(&stopped, true, true);
    expect(state, !kinetis_timing_system_clock_running(&stopped),
           "system clock stops in normal Stop");
    expect(state, !kinetis_timing_bus_clock_running(&stopped), "bus clock stops in normal Stop");
    stopped.smc[2] = 2u << 6u;
    expect(state, kinetis_timing_bus_clock_running(&stopped),
           "bus clock remains active in partial Stop 2");
    expect(state, !kinetis_timing_system_clock_running(NULL),
           "null timing state has no system clock");
    expect(state, !kinetis_timing_bus_clock_running(NULL), "null timing state has no bus clock");

    stopped.pit_mcr = 0u;
    stopped.sim_scgc6 |= 1u << 23u;
    stopped.pit[0].load = 9u;
    stopped.pit[0].current = 9u;
    stopped.pit[0].control = 1u;
    stopped.smc[2] = 0u;
    kinetis_timing_advance(&stopped, 1u);
    expect(state, stopped.pit[0].current == 9u, "PIT stops with the bus clock");
    stopped.smc[2] = 2u << 6u;
    kinetis_timing_advance(&stopped, 1u);
    expect(state, stopped.pit[0].current == 8u, "PIT runs in partial Stop 2");

    stopped.sim_scgc6 |= (1u << 22u) | (1u << 24u);
    stopped.pdb_sc[0] = 0x80u;
    stopped.pdb_mod[0] = 9u;
    stopped.pdb_counter[0] = 0u;
    stopped.pdb_running[0] = true;
    stopped.ftm[0].sc = 8u;
    stopped.ftm[0].modulo = 9u;
    stopped.ftm[0].counter = 0u;
    stopped.smc[2] = 0u;
    kinetis_timing_advance(&stopped, 1u);
    expect(state, stopped.pdb_counter[0] == 0u, "PDB stops with the bus clock");
    expect(state, stopped.ftm[0].counter == 0u, "FTM stops with the bus clock");
    stopped.smc[2] = 2u << 6u;
    kinetis_timing_advance(&stopped, 1u);
    expect(state, stopped.pdb_counter[0] == 1u, "PDB runs in partial Stop 2");
    expect(state, stopped.ftm[0].counter == 1u, "FTM runs in partial Stop 2");

    stopped.lptmr_psr = 5u;
    stopped.lptmr_cmr = 0xffffu;
    stopped.lptmr_csr = 1u;
    stopped.sim_scgc5 |= 1u;
    stopped.lptmr_counter = 0u;
    stopped.lptmr_remainder = 0u;
    kinetis_timing_test_disable_watchdog_fixture(&stopped);
    kinetis_timing_advance(&stopped,
                           kinetis_timing_test_cycles_for_ticks(&stopped, 1u, stopped.lpo_hz));
    expect(state, stopped.lptmr_counter == 1u, "LPO-clocked LPTMR runs in Stop");

    KinetisTiming clock_source = *timing;
    expect(state, kinetis_timing_lpuart_clock_hz(NULL) == 0u,
           "null timing state has no LPUART clock");
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "reset LPUART clock source is disabled");
    clock_source.sim_sopt2 = 1u << 26;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 32768u * 640u,
           "LPUART selects MCGFLLCLK");
    clock_source.sim_sopt2 |= 3u << 16;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 48000000u,
           "LPUART selects IRC48M");
    clock_source.sim_sopt2 = 2u << 26;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "disabled OSCERCLK does not clock LPUART");
    clock_source.osc_cr = 0x80u;
    expect(state,
           kinetis_timing_lpuart_clock_hz(&clock_source) == clock_source.external_oscillator_hz,
           "LPUART selects enabled OSCERCLK");
    clock_source.sim_sopt2 = 3u << 26;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "disabled MCGIRCLK does not clock LPUART");
    clock_source.mcg[0] |= 2u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == clock_source.slow_irc_hz,
           "LPUART selects slow MCGIRCLK");
    clock_source.mcg[1] |= 1u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == clock_source.fast_irc_hz / 2u,
           "LPUART selects divided fast MCGIRCLK");
    clock_source.deep_sleeping = true;
    clock_source.smc[3] = 2u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "MCGIRCLK stops without IREFSTEN");
    clock_source.mcg[0] |= 1u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == clock_source.fast_irc_hz / 2u,
           "IREFSTEN keeps MCGIRCLK active in Stop");
    clock_source.smc[3] = 0x20u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "LPUART clock sources stop in LLS");
    clock_source.smc[3] = 2u;
    clock_source.sim_sopt2 = 2u << 26u;
    clock_source.osc_cr = 0x80u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "OSCERCLK stops without EREFSTEN");
    clock_source.osc_cr |= 0x20u;
    expect(state,
           kinetis_timing_lpuart_clock_hz(&clock_source) == clock_source.external_oscillator_hz,
           "EREFSTEN keeps OSCERCLK active in Stop");
    clock_source.sim_sopt2 = 1u << 26u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "MCGFLLCLK stops in low-power modes");
    clock_source.sim_sopt2 |= 3u << 16u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "IRC48M stops in low-power modes");
    clock_source.sim_sopt2 = (1u << 26u) | (1u << 16u);
    clock_source.mcg[4] = 0x41u;
    clock_source.mcg[5] = 0u;
    clock_source.smc[3] = 2u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u,
           "PLL stops without PLLSTEN0");
    clock_source.mcg[4] |= 0x20u;
    clock_source.pll_locked = true;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 96000000u,
           "PLLSTEN0 keeps the PLL active in normal Stop");
    clock_source.smc[3] = 0x10u;
    expect(state, kinetis_timing_lpuart_clock_hz(&clock_source) == 0u, "PLL stops in VLPS");

    KinetisTiming peripheral_pll = *timing;
    kinetis_timing_test_expect_write(state, &peripheral_pll, MCG_C1, 1u, 0x80u);
    kinetis_timing_test_expect_write(state, &peripheral_pll, MCG_C5, 1u, 0x41u);
    kinetis_timing_test_expect_write(state, &peripheral_pll, MCG_C6, 1u, 0u);
    kinetis_timing_test_disable_watchdog_fixture(&peripheral_pll);
    kinetis_timing_advance(&peripheral_pll, 3349u);
    expect(state, (peripheral_pll.mcg[6] & 0x60u) == 0u,
           "peripheral-only PLL remains selected off and unlocked before its boundary");
    kinetis_timing_advance(&peripheral_pll, 1u);
    expect(state, (peripheral_pll.mcg[6] & 0x60u) == 0x40u,
           "PLLCLKEN0 acquires lock without selecting PLL as the MCG output");
    peripheral_pll.sim_sopt2 = (1u << 26u) | (1u << 16u);
    expect(state, kinetis_timing_lpuart_clock_hz(&peripheral_pll) == 96000000u,
           "LPUART receives the independently enabled PLL clock");
    kinetis_timing_test_expect_write(state, &peripheral_pll, MCG_C6, 1u, 0x40u);
    kinetis_timing_test_expect_write(state, &peripheral_pll, MCG_C6, 1u, 0u);
    expect(state, peripheral_pll.pll_locked,
           "changing PLL selection does not restart an independently enabled PLL");
    kinetis_timing_test_expect_write(state, &peripheral_pll, MCG_C5, 1u, 1u);
    expect(state, !peripheral_pll.pll_locked, "disabling the peripheral PLL clears lock");
    kinetis_timing_test_expect_write(state, &peripheral_pll, MCG_C5, 1u, 0x41u);
    expect(state, !peripheral_pll.pll_locked, "re-enabled peripheral PLL reacquires lock");

    KinetisTiming divided_acquisition = *timing;
    kinetis_timing_test_expect_write(state, &divided_acquisition, MCG_C1, 1u, 0x80u);
    kinetis_timing_test_expect_write(state, &divided_acquisition, MCG_C5, 1u, 1u);
    kinetis_timing_test_expect_write(state, &divided_acquisition, MCG_C6, 1u, 0x40u);
    kinetis_timing_test_disable_watchdog_fixture(&divided_acquisition);
    kinetis_timing_advance(&divided_acquisition, 1u);
    kinetis_timing_test_expect_write(state, &divided_acquisition, SIM_CLKDIV1, 4u, 0x10000000u);
    kinetis_timing_advance(&divided_acquisition, 1674u);
    expect(state, !divided_acquisition.pll_locked,
           "core divider changes preserve elapsed PLL acquisition time");
    kinetis_timing_advance(&divided_acquisition, 1u);
    expect(state, divided_acquisition.pll_locked,
           "PLL locks after the remaining divided-core interval");

    KinetisTiming accelerated_acquisition = *timing;
    kinetis_timing_test_expect_write(state, &accelerated_acquisition, MCG_C1, 1u, 0x80u);
    kinetis_timing_test_expect_write(state, &accelerated_acquisition, SIM_CLKDIV1, 4u, 0x10000000u);
    kinetis_timing_test_expect_write(state, &accelerated_acquisition, MCG_C5, 1u, 1u);
    kinetis_timing_test_expect_write(state, &accelerated_acquisition, MCG_C6, 1u, 0x40u);
    kinetis_timing_test_disable_watchdog_fixture(&accelerated_acquisition);
    kinetis_timing_advance(&accelerated_acquisition, 1u);
    kinetis_timing_test_expect_write(state, &accelerated_acquisition, SIM_CLKDIV1, 4u, 0u);
    kinetis_timing_advance(&accelerated_acquisition, 3347u);
    expect(state, !accelerated_acquisition.pll_locked,
           "faster core divider preserves the remaining PLL acquisition time");
    kinetis_timing_advance(&accelerated_acquisition, 1u);
    expect(state, accelerated_acquisition.pll_locked,
           "PLL locks after the accelerated remaining interval");

    KinetisTiming stopped_pll = divided_acquisition;
    kinetis_timing_set_cpu_sleeping(&stopped_pll, true, true);
    expect(state, !stopped_pll.pll_locked, "normal Stop disables PLL without PLLSTEN0");
    kinetis_timing_set_cpu_sleeping(&stopped_pll, false, false);
    kinetis_timing_advance(&stopped_pll, 1674u);
    expect(state, !stopped_pll.pll_locked, "wake restarts the full PLL acquisition interval");
    kinetis_timing_advance(&stopped_pll, 1u);
    expect(state, stopped_pll.pll_locked, "PLL reacquires after wake");

    KinetisTiming retained_pll = divided_acquisition;
    kinetis_timing_test_expect_write(state, &retained_pll, MCG_C5, 1u, 0x21u);
    kinetis_timing_set_cpu_sleeping(&retained_pll, true, true);
    expect(state, retained_pll.pll_locked, "PLLSTEN0 retains PLL lock in normal Stop");

    KinetisTiming vlps_pll = divided_acquisition;
    kinetis_timing_test_expect_write(state, &vlps_pll, MCG_C5, 1u, 0x21u);
    kinetis_timing_test_expect_write(state, &vlps_pll, SMC_PMPROT, 1u, 0x20u);
    kinetis_timing_test_expect_write(state, &vlps_pll, SMC_PMCTRL, 1u, 2u);
    kinetis_timing_set_cpu_sleeping(&vlps_pll, true, true);
    expect(state, !vlps_pll.pll_locked, "VLPS disables PLL despite PLLSTEN0");
    kinetis_timing_set_cpu_sleeping(&vlps_pll, false, false);
    expect(state, !vlps_pll.pll_locked, "VLPS wake requires PLL reacquisition");

    KinetisTiming pee_stop = divided_acquisition;
    kinetis_timing_test_expect_write(state, &pee_stop, MCG_C1, 1u, 0u);
    kinetis_timing_set_cpu_sleeping(&pee_stop, true, true);
    kinetis_timing_set_cpu_sleeping(&pee_stop, false, false);
    expect(state, (pee_stop.mcg[0] & 0xc0u) == 0x80u,
           "normal Stop wake forces unlocked PEE into PBE");
    expect(state, kinetis_timing_core_clock_hz(&pee_stop) == 4000000u,
           "PBE provides a core clock for PLL reacquisition");
    kinetis_timing_advance(&pee_stop, 1675u);
    expect(state, pee_stop.pll_locked, "PLL reacquires after PEE Stop wake");

    KinetisTiming retained_pee = divided_acquisition;
    kinetis_timing_test_expect_write(state, &retained_pee, MCG_C5, 1u, 0x21u);
    kinetis_timing_test_expect_write(state, &retained_pee, MCG_C1, 1u, 0u);
    kinetis_timing_set_cpu_sleeping(&retained_pee, true, true);
    kinetis_timing_set_cpu_sleeping(&retained_pee, false, false);
    expect(state, (retained_pee.mcg[0] & 0xc0u) == 0u && retained_pee.pll_locked,
           "retained PLL preserves PEE through normal Stop");

    KinetisTiming pee_vlps = divided_acquisition;
    kinetis_timing_test_expect_write(state, &pee_vlps, MCG_C5, 1u, 0x21u);
    kinetis_timing_test_expect_write(state, &pee_vlps, MCG_C1, 1u, 0u);
    kinetis_timing_test_expect_write(state, &pee_vlps, SMC_PMPROT, 1u, 0x20u);
    kinetis_timing_test_expect_write(state, &pee_vlps, SMC_PMCTRL, 1u, 2u);
    kinetis_timing_set_cpu_sleeping(&pee_vlps, true, true);
    kinetis_timing_set_cpu_sleeping(&pee_vlps, false, false);
    expect(state, (pee_vlps.mcg[0] & 0xc0u) == 0x80u, "VLPS wake forces unlocked PEE into PBE");

    KinetisTiming pee_lls = divided_acquisition;
    kinetis_timing_test_expect_write(state, &pee_lls, MCG_C1, 1u, 0u);
    kinetis_timing_test_expect_write(state, &pee_lls, SMC_PMPROT, 1u, 8u);
    kinetis_timing_test_expect_write(state, &pee_lls, SMC_PMCTRL, 1u, 3u);
    kinetis_timing_set_cpu_sleeping(&pee_lls, true, true);
    kinetis_timing_set_cpu_sleeping(&pee_lls, false, false);
    expect(state, (pee_lls.mcg[0] & 0xc0u) == 0x80u, "LLS wake forces unlocked PEE into PBE");

    KinetisTiming mcg_source = *timing;
    mcg_source.profile = kinetis_profile_get(KINETIS_PROFILE_MKV30F12810);
    mcg_source.sim_clkdiv1 = 0u;
    mcg_source.mcg[0] = 6u << 3u;
    mcg_source.mcg[1] = 1u << 4u;
    mcg_source.mcg[3] = 3u << 5u;
    mcg_source.mcg[5] = 0u;
    mcg_source.mcg[12] = 2u;
    kinetis_timing_test_expect_write(state, &mcg_source, MCG_C1, 1u, mcg_source.mcg[0]);
    expect(state, kinetis_timing_core_clock_hz(&mcg_source) == 96000000u,
           "KV30 FLL uses the selected 48 MHz internal reference");
    mcg_source.mcg[0] = 2u << 6u;
    kinetis_timing_test_expect_write(state, &mcg_source, MCG_C1, 1u, mcg_source.mcg[0]);
    expect(state, kinetis_timing_core_clock_hz(&mcg_source) == 48000000u,
           "KV30 external-reference mode uses the selected 48 MHz internal clock");

    mcg_source.profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN51212);
    mcg_source.mcg[12] = 1u;
    mcg_source.rtc_cr = 0x100u;
    kinetis_timing_test_expect_write(state, &mcg_source, MCG_C1, 1u, mcg_source.mcg[0]);
    expect(state, kinetis_timing_core_clock_hz(&mcg_source) == mcg_source.rtc_oscillator_hz,
           "K22 external-reference mode uses the selected RTC oscillator");
    mcg_source.rtc_cr = 0x300u;
    kinetis_timing_test_expect_write(state, &mcg_source, MCG_C1, 1u, mcg_source.mcg[0]);
    expect(state, kinetis_timing_core_clock_hz(&mcg_source) == 0u,
           "K22 external-reference mode has no output without RTC clock output");
    mcg_source.rtc_cr = 0x100u;
    mcg_source.mcg[0] = 6u << 3u;
    kinetis_timing_test_expect_write(state, &mcg_source, MCG_C1, 1u, mcg_source.mcg[0]);
    expect(state, kinetis_timing_core_clock_hz(&mcg_source) == 1310720u,
           "K22 RTC FLL reference uses the low-range divider");

    KinetisTiming switched_rtc = *timing;
    kinetis_timing_test_expect_write(state, &switched_rtc, RTC_CR, 4u, 0x100u);
    kinetis_timing_test_expect_write(state, &switched_rtc, MCG_C7, 1u, 1u);
    kinetis_timing_test_expect_write(state, &switched_rtc, MCG_C1, 1u, 0x80u);
    expect(state, kinetis_timing_core_clock_hz(&switched_rtc) == 32768u,
           "enabled RTC output immediately clocks the MCG");
    kinetis_timing_test_expect_write(state, &switched_rtc, RTC_CR, 4u, 0x300u);
    expect(state, kinetis_timing_core_clock_hz(&switched_rtc) == 0u,
           "disabling RTC output immediately stops the selected clock");
    kinetis_timing_test_expect_write(state, &switched_rtc, RTC_CR, 4u, 0x100u);
    expect(state, kinetis_timing_core_clock_hz(&switched_rtc) == 32768u,
           "restoring RTC output immediately restores the selected clock");
    kinetis_timing_test_expect_write(state, &switched_rtc, RTC_CR, 4u, 1u);
    expect(state, kinetis_timing_core_clock_hz(&switched_rtc) == 0u,
           "RTC software reset immediately removes the selected clock");

    KinetisTiming lptmr_source = *timing;
    lptmr_source.mcg[0] &= ~2u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 0u) == 0u,
           "disabled MCGIRCLK does not clock LPTMR");
    lptmr_source.mcg[0] |= 2u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 0u) == 32768u,
           "LPTMR clock zero selects MCGIRCLK");
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 1u) == 1000u,
           "LPTMR clock one selects LPO");
    lptmr_source.sim_sopt1 = 2u << 18u;
    lptmr_source.rtc_cr = 0x100u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 32768u,
           "MK22 ERCLK32K selects the enabled RTC oscillator");
    lptmr_source.rtc_cr = 0x300u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 0u,
           "MK22 ERCLK32K requires RTC clock output");
    lptmr_source.rtc_cr = 0x100u;
    lptmr_source.profile = kinetis_profile_get(KINETIS_PROFILE_MKV30F12810);
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 0u,
           "KV30 reserved ERCLK32K selection is disabled");
    lptmr_source.profile = timing->profile;
    lptmr_source.sim_sopt1 = 0u;
    lptmr_source.osc_cr = 0x80u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 0u,
           "high-frequency system oscillator cannot drive ERCLK32K");
    lptmr_source.external_oscillator_hz = 32768u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 32768u,
           "ERCLK32K selects an enabled 32 kHz system oscillator");
    lptmr_source.deep_sleeping = true;
    lptmr_source.smc[3] = 2u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 0u,
           "K22 ERCLK32K system oscillator stops without EREFSTEN");
    lptmr_source.profile = kinetis_profile_get(KINETIS_PROFILE_MKV30F12810);
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 0u,
           "KV30 ERCLK32K system oscillator stops without EREFSTEN");
    lptmr_source.osc_cr |= 0x20u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 32768u,
           "EREFSTEN keeps the KV30 ERCLK32K system oscillator active in Stop");
    lptmr_source.profile = timing->profile;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 2u) == 32768u,
           "EREFSTEN keeps the K22 ERCLK32K system oscillator active in Stop");
    lptmr_source.deep_sleeping = false;
    lptmr_source.external_oscillator_hz = timing->external_oscillator_hz;
    lptmr_source.osc_cr = 0u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 3u) == 0u,
           "disabled OSCERCLK does not clock LPTMR");
    lptmr_source.osc_cr = 0x80u;
    expect(state, lptmr_ticks_in_one_second(lptmr_source, 3u) == 4608u,
           "LPTMR clock three selects OSCERCLK");

    kinetis_timing_test_expect_write(state, timing, MCG_C1, 1, 0x32u);
    kinetis_timing_test_expect_write(state, timing, MCG_C2, 1, 0xa0u);
    kinetis_timing_test_expect_write(state, timing, MCG_C4, 1, 0x60u);
    kinetis_timing_test_expect_write(state, timing, SIM_CLKDIV1, 4, 0x03030000u);
    expect(state, kinetis_timing_core_clock_hz(timing) == 16000000u,
           "kinetis_timing_core_clock_hz(timing) == 16000000u");
    expect(state, kinetis_timing_bus_clock_hz(timing) == 4000000u,
           "kinetis_timing_bus_clock_hz(timing) == 4000000u");
    kinetis_timing_test_expect_write(state, timing, MCG_C1, 1, 0x80u);
    expect(state, kinetis_timing_core_clock_hz(timing) == 8000000u,
           "kinetis_timing_core_clock_hz(timing) == 8000000u");
    kinetis_timing_test_expect_read(state, timing, MCG_S, 1, 0x08u);
    kinetis_timing_test_expect_write(state, timing, SIM_CLKDIV1, 4, 0x12000000u);
    expect(state, kinetis_timing_core_clock_hz(timing) == 4000000u,
           "kinetis_timing_core_clock_hz(timing) == 4000000u");
    expect(state, kinetis_timing_bus_clock_hz(timing) == 8000000u / 3u,
           "kinetis_timing_bus_clock_hz(timing) == 8000000u / 3u");
    kinetis_timing_test_expect_write(state, timing, MCG_C5, 1, 1u);
    kinetis_timing_test_expect_write(state, timing, MCG_C6, 1, 0x40u);
    kinetis_timing_test_disable_watchdog_fixture(timing);
    kinetis_timing_advance(timing, 1674u);
    kinetis_timing_test_expect_read(state, timing, MCG_S, 1, 0x28u);
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, MCG_S, 1, 0x68u);
    kinetis_timing_test_expect_write(state, timing, MCG_C1, 1, 0);
    expect(state, kinetis_timing_core_clock_hz(timing) == 96000000u / 2u,
           "kinetis_timing_core_clock_hz(timing) == 96000000u / 2u");
    kinetis_timing_test_expect_read(state, timing, MCG_S, 1, 0x6cu);
    kinetis_timing_test_expect_write(state, timing, SMC_PMPROT, 1, 0x80u);
    kinetis_timing_test_expect_write(state, timing, SMC_PMCTRL, 1, 0x60u);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1, 0x80u);
    kinetis_timing_test_expect_write(state, timing, SMC_PMCTRL, 1, 0);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1, 1u);
    kinetis_timing_test_expect_write(state, timing, RCM_SSRS0, 1, 0x80u);
    kinetis_timing_test_expect_read(state, timing, RCM_SSRS0, 1, 2u);
}

void kinetis_timing_test_test_low_voltage_control(TestState* state, KinetisTiming* timing,
                                                  Observations* observations) {
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC1, 1u, 0x10u);
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC2, 1u, 0u);
    kinetis_timing_test_expect_read(state, timing, PMC_REGSC, 1u, 4u);
    kinetis_timing_test_expect_write(state, timing, PMC_LVDSC1, 1u, 0x20u);
    kinetis_timing_test_expect_write(state, timing, PMC_LVDSC1, 1u, 0x30u);
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC1, 1u, 0x20u);
    expect(state, kinetis_timing_trigger_low_voltage_detect(timing),
           "kinetis_timing_trigger_low_voltage_detect(timing)");
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC1, 1u, 0xa0u);
    expect(state, observations->irq[20u], "observations->irq[20u]");
    kinetis_timing_test_expect_write(state, timing, PMC_LVDSC1, 1u, 0x60u);
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC1, 1u, 0x20u);
    expect(state, !observations->irq[20u], "!observations->irq[20u]");

    kinetis_timing_test_expect_write(state, timing, PMC_LVDSC2, 1u, 0x20u);
    expect(state, kinetis_timing_trigger_low_voltage_warning(timing),
           "kinetis_timing_trigger_low_voltage_warning(timing)");
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC2, 1u, 0xa0u);
    expect(state, observations->irq[20u], "observations->irq[20u]");
    kinetis_timing_test_expect_write(state, timing, PMC_LVDSC2, 1u, 0x60u);
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC2, 1u, 0x20u);
    expect(state, !observations->irq[20u], "!observations->irq[20u]");

    timing->pmc[2] |= 8u;
    kinetis_timing_test_expect_write(state, timing, PMC_REGSC, 1u, 0x19u);
    kinetis_timing_test_expect_read(state, timing, PMC_REGSC, 1u, 0x15u);

    const uint32_t reset_count = observations->resets;
    kinetis_timing_reset(timing, 0x82u, 0u);
    expect(state, kinetis_timing_trigger_low_voltage_detect(timing),
           "kinetis_timing_trigger_low_voltage_detect(timing)");
    expect(state, observations->resets == reset_count + 1u,
           "observations->resets == reset_count + 1u");
    kinetis_timing_test_expect_read(state, timing, RCM_SRS0, 1u, 2u);
    expect(state, !kinetis_timing_trigger_low_voltage_warning(NULL),
           "!kinetis_timing_trigger_low_voltage_warning(NULL)");
    expect(state, !kinetis_timing_trigger_low_voltage_detect(NULL),
           "!kinetis_timing_trigger_low_voltage_detect(NULL)");
}

void kinetis_timing_test_test_low_leakage_wakeup(TestState* state, KinetisTiming* timing,
                                                 Observations* observations) {
    kinetis_timing_set_cpu_sleeping(timing, true, true);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 2u);
    kinetis_timing_set_cpu_sleeping(timing, false, false);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 1u);

    kinetis_timing_test_expect_write(state, timing, SMC_PMPROT, 1u, 0x0au);
    kinetis_timing_test_expect_write(state, timing, SMC_PMCTRL, 1u, 3u);
    kinetis_timing_set_cpu_sleeping(timing, true, true);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 0x20u);
    kinetis_timing_test_expect_write(state, timing, LLWU_PE1, 1u, 1u);
    expect(state, kinetis_timing_set_llwu_pin(timing, 0u, false),
           "kinetis_timing_set_llwu_pin(timing, 0u, false)");
    expect(state, kinetis_timing_set_llwu_pin(timing, 0u, true),
           "kinetis_timing_set_llwu_pin(timing, 0u, true)");
    kinetis_timing_test_expect_read(state, timing, LLWU_F1, 1u, 1u);
    expect(state, observations->irq[21u], "observations->irq[21u]");
    kinetis_timing_test_expect_write(state, timing, LLWU_F1, 1u, 1u);
    expect(state, !observations->irq[21u], "!observations->irq[21u]");
    kinetis_timing_set_cpu_sleeping(timing, false, false);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 1u);

    kinetis_timing_test_expect_write(state, timing, LLWU_FILT1, 1u, 0x22u);
    kinetis_timing_set_cpu_sleeping(timing, true, true);
    expect(state, kinetis_timing_set_llwu_pin(timing, 2u, false),
           "kinetis_timing_set_llwu_pin(timing, 2u, false)");
    expect(state, kinetis_timing_set_llwu_pin(timing, 2u, true),
           "kinetis_timing_set_llwu_pin(timing, 2u, true)");
    kinetis_timing_test_expect_read(state, timing, LLWU_FILT1, 1u, 0xa2u);
    expect(state, observations->irq[21u], "observations->irq[21u]");
    kinetis_timing_test_expect_write(state, timing, LLWU_FILT1, 1u, 0x80u);
    kinetis_timing_test_expect_read(state, timing, LLWU_FILT1, 1u, 0u);
    expect(state, !observations->irq[21u], "!observations->irq[21u]");

    kinetis_timing_reset(timing, 0x82u, 0u);
    memset(observations, 0, sizeof(*observations));
    kinetis_timing_test_expect_write(state, timing, SMC_PMPROT, 1u, 8u);
    kinetis_timing_test_expect_write(state, timing, SMC_PMCTRL, 1u, 3u);
    kinetis_timing_test_expect_write(state, timing, LLWU_ME, 1u, 8u);
    kinetis_timing_set_cpu_sleeping(timing, true, true);
    expect(state, kinetis_timing_trigger_llwu_module(timing, 3u),
           "kinetis_timing_trigger_llwu_module(timing, 3u)");
    kinetis_timing_test_expect_read(state, timing, LLWU_F3, 1u, 8u);
    expect(state, observations->irq[21u], "observations->irq[21u]");

    kinetis_timing_reset(timing, 0x82u, 0u);
    memset(observations, 0, sizeof(*observations));
    kinetis_timing_test_expect_write(state, timing, SMC_PMPROT, 1u, 0x20u);
    kinetis_timing_test_expect_write(state, timing, SMC_PMCTRL, 1u, 0x40u);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 4u);
    expect(state, kinetis_timing_trigger_low_voltage_warning(timing),
           "kinetis_timing_trigger_low_voltage_warning(timing)");
    expect(state, kinetis_timing_trigger_low_voltage_detect(timing),
           "kinetis_timing_trigger_low_voltage_detect(timing)");
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC1, 1u, 0x10u);
    kinetis_timing_test_expect_read(state, timing, PMC_LVDSC2, 1u, 0u);
    expect(state, observations->resets == 0u, "observations->resets == 0u");

    kinetis_timing_reset(timing, 0x82u, 0u);
    memset(observations, 0, sizeof(*observations));
    kinetis_timing_test_expect_write(state, timing, SMC_PMPROT, 1u, 0x20u);
    kinetis_timing_test_expect_write(state, timing, SMC_PMCTRL, 1u, 2u);
    kinetis_timing_set_cpu_sleeping(timing, true, true);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 0x10u);
    kinetis_timing_set_cpu_sleeping(timing, false, false);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 1u);

    kinetis_timing_reset(timing, 0x82u, 0u);
    memset(observations, 0, sizeof(*observations));
    kinetis_timing_test_expect_write(state, timing, SMC_PMPROT, 1u, 2u);
    kinetis_timing_test_expect_write(state, timing, SMC_PMCTRL, 1u, 4u);
    kinetis_timing_test_expect_write(state, timing, LLWU_PE1, 1u, 4u);
    kinetis_timing_set_cpu_sleeping(timing, true, true);
    kinetis_timing_test_expect_read(state, timing, SMC_PMSTAT, 1u, 0x40u);
    expect(state, kinetis_timing_set_llwu_pin(timing, 1u, true),
           "kinetis_timing_set_llwu_pin(timing, 1u, true)");
    expect(state, observations->resets == 1u, "observations->resets == 1u");
    kinetis_timing_test_expect_read(state, timing, RCM_SRS0, 1u, 1u);

    expect(state, !kinetis_timing_set_llwu_pin(NULL, 0u, false),
           "!kinetis_timing_set_llwu_pin(NULL, 0u, false)");
    expect(state, !kinetis_timing_set_llwu_pin(timing, 16u, false),
           "!kinetis_timing_set_llwu_pin(timing, 16u, false)");
    expect(state, !kinetis_timing_trigger_llwu_module(NULL, 0u),
           "!kinetis_timing_trigger_llwu_module(NULL, 0u)");
    expect(state, !kinetis_timing_trigger_llwu_module(timing, 8u),
           "!kinetis_timing_trigger_llwu_module(timing, 8u)");
    kinetis_timing_set_cpu_sleeping(NULL, true, true);
}

void kinetis_timing_test_test_pit(TestState* state, KinetisTiming* timing,
                                  Observations* observations) {
    const uint32_t alternate_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, SIM_SCGC6, 4, timing->sim_scgc6 | (1u << 23u));
    kinetis_timing_test_expect_write(state, timing, PIT_MCR, 4, 0);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL0, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL1, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL1, 4, 7u);
    kinetis_timing_advance(timing, 6u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 2u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL1, 4, 1u);
    expect(state, observations->irq[48], "observations->irq[48]");
    expect(state, observations->alternate_triggers == alternate_before + 2u,
           "observations->alternate_triggers == alternate_before + 2u");
    expect(state, observations->dma_triggers[0] == 1u, "observations->dma_triggers[0] == 1u");
    expect(state, observations->dma_triggers[1] == 1u, "observations->dma_triggers[1] == 1u");
    expect(state, observations->last_trigger_instance == 5u,
           "observations->last_trigger_instance == 5u");
    kinetis_timing_test_expect_write(state, timing, PIT_TFLG0, 4, 1u);
    expect(state, !observations->irq[48], "!observations->irq[48]");
    kinetis_timing_test_expect_write(state, timing, SIM_SCGC6, 4, timing->sim_scgc6 & ~(1u << 23u));
    kinetis_timing_advance(timing, 100u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 2u);

    kinetis_timing_test_expect_write(state, timing, SIM_SCGC6, 4, timing->sim_scgc6 | (1u << 23u));
    kinetis_timing_test_expect_write(state, timing, PIT_MCR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL0, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 1u);
    kinetis_timing_advance(timing, 2u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL0, 4, 9u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 3u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 1u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 3u);
    kinetis_timing_advance(timing, 4u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 9u);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG0, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 3u);
    expect(state, observations->irq[48], "observations->irq[48]");
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 1u);
    expect(state, !observations->irq[48], "!observations->irq[48]");
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 7u);
    kinetis_timing_test_expect_read(state, timing, PIT_TCTRL0, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, PIT_TFLG0, 4, 1u);

    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL1, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL2, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL3, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL0, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL1, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL2, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL3, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL1, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL2, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL3, 4, 7u);
    kinetis_timing_advance(timing, 15u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL1, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL2, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL3, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG0, 4, 1u);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG1, 4, 1u);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG2, 4, 1u);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG3, 4, 0u);
    expect(state, !observations->irq[48], "!observations->irq[48]");
    expect(state, !observations->irq[49], "!observations->irq[49]");
    expect(state, !observations->irq[50], "!observations->irq[50]");
    expect(state, !observations->irq[51], "!observations->irq[51]");
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG3, 4, 1u);
    expect(state, observations->irq[51], "observations->irq[51]");
    kinetis_timing_test_expect_write(state, timing, PIT_TFLG0, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_TFLG1, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_TFLG2, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PIT_TFLG3, 4, 1u);

    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_LDVAL0, 4, UINT32_MAX);
    kinetis_timing_test_expect_write(state, timing, PIT_TCTRL0, 4, 1u);
    timing->wdog[0] = 0u;
    kinetis_timing_advance(timing, UINT32_MAX);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG0, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_MCR, 4, 2u);
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PIT_MCR, 4, 0u);
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, PIT_CVAL0, 4, UINT32_MAX);
    kinetis_timing_test_expect_read(state, timing, PIT_TFLG0, 4, 1u);
}

void kinetis_timing_test_test_lptmr(TestState* state, KinetisTiming* timing,
                                    Observations* observations) {
    const uint32_t alternate_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, SIM_SCGC5, 4, timing->sim_scgc5 | 1u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_PSR, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0x41u);
    kinetis_timing_advance(timing, kinetis_timing_test_cycles_for_ticks(timing, 2u, 1000u));
    kinetis_timing_test_expect_read(state, timing, LPTMR_CSR, 4, 0x41u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, UINT32_MAX);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 2u);
    kinetis_timing_advance(timing, kinetis_timing_test_cycles_for_ticks(timing, 1u, 1000u));
    kinetis_timing_test_expect_read(state, timing, LPTMR_CSR, 4, 0xc1u);
    expect(state, observations->irq[58], "observations->irq[58]");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0xc1u);
    expect(state, !observations->irq[58], "!observations->irq[58]");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 1u);
    kinetis_timing_advance(timing, kinetis_timing_test_cycles_for_ticks(timing, 1u, 1000u));
    kinetis_timing_test_expect_read(state, timing, LPTMR_CSR, 4, 0x81u);

    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_PSR, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0x47u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_PSR, 4, 0x7fu);
    kinetis_timing_test_expect_read(state, timing, LPTMR_PSR, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 9u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CMR, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0x7fu);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CSR, 4, 0x47u);

    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_PSR, 4, 4u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 1u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, false),
           "kinetis_timing_set_lptmr_input(timing, 0u, false)");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0x43u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 1u, true),
           "kinetis_timing_set_lptmr_input(timing, 1u, true)");
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, false),
           "kinetis_timing_set_lptmr_input(timing, 0u, false)");
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, true),
           "kinetis_timing_set_lptmr_input(timing, 0u, true)");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 1u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, false),
           "kinetis_timing_set_lptmr_input(timing, 0u, false)");
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, true),
           "kinetis_timing_set_lptmr_input(timing, 0u, true)");
    kinetis_timing_test_expect_read(state, timing, LPTMR_CSR, 4, 0xc3u);
    expect(state, observations->irq[58], "observations->irq[58]");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 0u);

    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_PSR, 4, 4u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 3u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 2u, true),
           "kinetis_timing_set_lptmr_input(timing, 2u, true)");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0x2fu);
    expect(state, kinetis_timing_set_lptmr_input(timing, 2u, false),
           "kinetis_timing_set_lptmr_input(timing, 2u, false)");
    expect(state, kinetis_timing_set_lptmr_input(timing, 2u, true),
           "kinetis_timing_set_lptmr_input(timing, 2u, true)");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 1u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 2u, false),
           "kinetis_timing_set_lptmr_input(timing, 2u, false)");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 2u);

    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_PSR, 4, 4u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 1u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, false),
           "kinetis_timing_set_lptmr_input(timing, 0u, false)");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0x47u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, false),
           "kinetis_timing_set_lptmr_input(timing, 0u, false)");
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, true),
           "kinetis_timing_set_lptmr_input(timing, 0u, true)");
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, false),
           "kinetis_timing_set_lptmr_input(timing, 0u, false)");
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, true),
           "kinetis_timing_set_lptmr_input(timing, 0u, true)");
    kinetis_timing_test_expect_read(state, timing, LPTMR_CSR, 4, 0xc7u);
    expect(state, observations->irq[58], "observations->irq[58]");
    expect(state, observations->alternate_triggers > alternate_before,
           "observations->alternate_triggers > alternate_before");
    expect(state, observations->last_trigger_instance == 14u,
           "observations->last_trigger_instance == 14u");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0xc7u);
    expect(state, !observations->irq[58], "!observations->irq[58]");

    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_PSR, 4, 9u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CMR, 4, 4u);
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, false),
           "kinetis_timing_set_lptmr_input(timing, 0u, false)");
    kinetis_timing_test_expect_write(state, timing, LPTMR_CSR, 4, 3u);
    kinetis_timing_advance(timing, kinetis_timing_test_cycles_for_ticks(timing, 2u, 1000u));
    kinetis_timing_advance(timing, kinetis_timing_test_cycles_for_ticks(timing, 1u, 1000u));
    expect(state, kinetis_timing_set_lptmr_input(timing, 0u, true),
           "kinetis_timing_set_lptmr_input(timing, 0u, true)");
    kinetis_timing_advance(timing, kinetis_timing_test_cycles_for_ticks(timing, 1u, 1000u));
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_advance(timing, kinetis_timing_test_cycles_for_ticks(timing, 1u, 1000u));
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 1u);

    kinetis_timing_warm_reset(timing, 0x20u, 0u);
    kinetis_timing_test_expect_write(state, timing, LPTMR_CNR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CNR, 4, 1u);
    kinetis_timing_warm_reset(timing, 0x04u, 0u);
    kinetis_timing_test_expect_read(state, timing, LPTMR_CSR, 4, 0u);
    expect(state, !kinetis_timing_set_lptmr_input(NULL, 0u, false),
           "!kinetis_timing_set_lptmr_input(NULL, 0u, false)");
    expect(state, !kinetis_timing_set_lptmr_input(timing, 3u, false),
           "!kinetis_timing_set_lptmr_input(timing, 3u, false)");
}

void kinetis_timing_test_test_rtc(TestState* state, KinetisTiming* timing,
                                  Observations* observations) {
    const uint32_t alternate_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, SIM_SCGC6, 4, timing->sim_scgc6 | (1u << 29u));
    kinetis_timing_test_expect_read(state, timing, RTC_TSR, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, RTC_TPR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, RTC_TPR, 4, 7u);
    kinetis_timing_test_expect_write(state, timing, RTC_TSR, 4, 10u);
    kinetis_timing_test_expect_read(state, timing, RTC_TPR, 4, 7u);
    kinetis_timing_test_expect_write(state, timing, RTC_TPR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, RTC_TAR, 4, 11u);
    kinetis_timing_test_expect_write(state, timing, RTC_IER, 4, 0x14u);
    kinetis_timing_test_expect_write(state, timing, RTC_CR, 4, 0x100u);
    kinetis_timing_test_expect_write(state, timing, RTC_SR, 4, 0x10u);
    kinetis_timing_test_expect_write(state, timing, RTC_TSR, 4, 20u);
    kinetis_timing_test_expect_write(state, timing, RTC_TPR, 4, 20u);
    kinetis_timing_advance(timing, timing->core_clock_hz);
    kinetis_timing_test_expect_read(state, timing, RTC_TSR, 4, 11u);
    kinetis_timing_test_expect_read(state, timing, RTC_TPR, 4, 0u);
    expect(state, observations->irq[46], "observations->irq[46]");
    expect(state, !observations->irq[47], "!observations->irq[47]");
    expect(state, observations->irq_assertions[47] == 1u, "observations->irq_assertions[47] == 1u");
    kinetis_timing_test_expect_read(state, timing, RTC_SR, 4, 0x14u);
    expect(state, observations->alternate_triggers == alternate_before + 2u,
           "observations->alternate_triggers == alternate_before + 2u");
    expect(state, observations->last_trigger_instance == 12u,
           "observations->last_trigger_instance == 12u");
    kinetis_timing_test_expect_write(state, timing, RTC_TAR, 4, 15u);
    kinetis_timing_test_expect_read(state, timing, RTC_SR, 4, 0x10u);
    expect(state, !observations->irq[46], "!observations->irq[46]");
    const uint32_t retained_tsr = timing->rtc_tsr;
    const uint16_t retained_tpr = timing->rtc_tpr;
    kinetis_timing_test_expect_write(state, timing, RTC_RAR, 4, 0xfbu);
    kinetis_timing_test_expect_read(state, timing, RTC_TAR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, RTC_RAR, 4, 0xffu);
    kinetis_timing_test_expect_read(state, timing, RTC_TAR, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, RTC_WAR, 4, 0xfbu);
    kinetis_timing_test_expect_write(state, timing, RTC_TAR, 4, 22u);
    kinetis_timing_warm_reset(timing, 0x20u, 0);
    expect(state, timing->rtc_tsr == retained_tsr, "timing->rtc_tsr == retained_tsr");
    expect(state, timing->rtc_tpr == retained_tpr, "timing->rtc_tpr == retained_tpr");
    kinetis_timing_test_expect_read(state, timing, RTC_WAR, 4, 0xffu);
    kinetis_timing_test_expect_read(state, timing, RTC_RAR, 4, 0xffu);
    kinetis_timing_test_expect_read(state, timing, RTC_TAR, 4, 15u);
    kinetis_timing_test_expect_read(state, timing, RCM_SRS0, 1, 0x20u);
}

void kinetis_timing_test_test_rtc_protection_and_compensation(TestState* state,
                                                              const KinetisDeviceProfile* profile) {
    Observations observations = {0};
    KinetisTiming timing;
    expect(state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "kinetis_timing_init(&timing, profile, 8000000u, 32768u, "
           "kinetis_timing_test_signals(&observations))");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    kinetis_timing_test_expect_write(state, &timing, RTC_TSR, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0x10u);
    kinetis_timing_advance(&timing, timing.core_clock_hz);
    kinetis_timing_test_expect_read(state, &timing, RTC_TSR, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, RTC_CR, 4, 0x100u);
    kinetis_timing_test_expect_write(state, &timing, RTC_TCR, 4, 0x017fu);
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0x10u);
    kinetis_timing_advance(&timing, timing.core_clock_hz);
    kinetis_timing_test_expect_read(state, &timing, RTC_TSR, 4, 2u);
    kinetis_timing_test_expect_read(state, &timing, RTC_TCR, 4, 0x017f017fu);
    kinetis_timing_advance(
        &timing, kinetis_timing_test_cycles_for_ticks(&timing, 32640u, timing.rtc_oscillator_hz));
    kinetis_timing_test_expect_read(state, &timing, RTC_TSR, 4, 2u);
    kinetis_timing_advance(
        &timing, kinetis_timing_test_cycles_for_ticks(&timing, 1u, timing.rtc_oscillator_hz));
    kinetis_timing_test_expect_read(state, &timing, RTC_TSR, 4, 3u);
    kinetis_timing_test_expect_read(state, &timing, RTC_TCR, 4, 0x0000017fu);

    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, RTC_TCR, 4, 0x0080u);
    kinetis_timing_test_expect_write(state, &timing, RTC_TPR, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0x10u);
    kinetis_timing_advance(&timing, timing.core_clock_hz);
    kinetis_timing_test_expect_read(state, &timing, RTC_TCR, 4, 0x00800080u);
    const uint32_t before = timing.rtc_tsr;
    kinetis_timing_advance(&timing, timing.core_clock_hz);
    kinetis_timing_test_expect_read(state, &timing, RTC_TSR, 4, before);
    kinetis_timing_advance(
        &timing, kinetis_timing_test_cycles_for_ticks(&timing, 128u, timing.rtc_oscillator_hz));
    kinetis_timing_test_expect_read(state, &timing, RTC_TSR, 4, before + 1u);

    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, RTC_TSR, 4, UINT32_MAX);
    kinetis_timing_test_expect_write(state, &timing, RTC_IER, 4, 2u);
    kinetis_timing_test_expect_write(state, &timing, RTC_CR, 4, 0x100u);
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0x10u);
    kinetis_timing_advance(
        &timing, timing.core_clock_hz +
                     kinetis_timing_test_cycles_for_ticks(&timing, 128u, timing.rtc_oscillator_hz));
    kinetis_timing_test_expect_read(state, &timing, RTC_SR, 4, 0x16u);
    kinetis_timing_test_expect_read(state, &timing, RTC_TSR, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, RTC_TPR, 4, 0u);
    expect(state, observations.irq[46], "observations.irq[46]");
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, RTC_TSR, 4, 9u);
    kinetis_timing_test_expect_write(state, &timing, RTC_TAR, 4, 10u);
    kinetis_timing_test_expect_read(state, &timing, RTC_SR, 4, 0u);
    expect(state, !observations.irq[46], "!observations.irq[46]");

    kinetis_timing_test_expect_write(state, &timing, RTC_CR, 4, 0x108u);
    kinetis_timing_test_expect_write(state, &timing, RTC_LR, 4, 0xdfu);
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0x10u);
    kinetis_timing_test_expect_read(state, &timing, RTC_SR, 4, 0x10u);
    kinetis_timing_test_expect_write(state, &timing, RTC_SR, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, RTC_SR, 4, 0x10u);
    kinetis_timing_test_expect_write(state, &timing, RTC_CR, 4, 1u);
    kinetis_timing_test_expect_read(state, &timing, RTC_SR, 4, 1u);

    kinetis_timing_test_expect_write(state, &timing, RTC_TCR, 4, 0x1234u);
    kinetis_timing_test_expect_write(state, &timing, RTC_LR, 4, 0xf7u);
    kinetis_timing_test_expect_write(state, &timing, RTC_TCR, 4, 0x5678u);
    kinetis_timing_test_expect_read(state, &timing, RTC_TCR, 4, 0x00001234u);
    kinetis_timing_test_expect_write(state, &timing, RTC_CR, 4, 1u);
    kinetis_timing_test_expect_read(state, &timing, RTC_CR, 4, 1u);
    kinetis_timing_test_expect_read(state, &timing, RTC_SR, 4, 1u);
    kinetis_timing_test_expect_read(state, &timing, RTC_LR, 4, 0xffu);
    kinetis_timing_test_expect_write(state, &timing, RTC_CR, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, RTC_CR, 4, 0u);

    kinetis_timing_test_expect_write(state, &timing, RTC_WAR, 4, 0x7fu);
    kinetis_timing_test_expect_write(state, &timing, RTC_IER, 4, 0x17u);
    kinetis_timing_test_expect_read(state, &timing, RTC_IER, 4, 7u);
    kinetis_timing_test_expect_write(state, &timing, RTC_RAR, 4, 0x7fu);
    kinetis_timing_test_expect_read(state, &timing, RTC_IER, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, RTC_WAR, 4, 0x7fu);
    kinetis_timing_test_expect_read(state, &timing, RTC_RAR, 4, 0x7fu);
}
