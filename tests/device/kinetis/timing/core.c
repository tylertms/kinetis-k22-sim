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
                           uint8_t channel) {
    Observations* observations = context;
    if (trigger == KINETIS_TIMING_TRIGGER_PDB_ADC)
        observations->adc_triggers++;
    else if (trigger == KINETIS_TIMING_TRIGGER_PDB_DAC)
        observations->dac_triggers++;
    else
        observations->alternate_triggers++;
    observations->last_trigger_instance = instance;
    observations->last_trigger_channel = channel;
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
    static const uint32_t expected_fcfg2[KINETIS_PROFILE_COUNT] = {
        0x10000000u, 0x10800000u, 0x10000000u, 0x20000000u, 0x20200000u, 0x40c00000u, 0x40100000u,
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
        kinetis_timing_test_expect_read(
            state, &timing, 0x4004804cu, 4,
            id == KINETIS_PROFILE_MK22FN1M012 || id == KINETIS_PROFILE_MK22FX51212 ? 0xff0f0f00u
                                                                                   : 0x0f0f0f00u);
        expect(state,
               kinetis_timing_read(&timing, SIM_SCGC3, 4, &(uint32_t){0}) ==
                   (id == KINETIS_PROFILE_MK22FN1M012 || id == KINETIS_PROFILE_MK22FX51212),
               "kinetis_timing_read(&timing, SIM_SCGC3, 4, &(uint32_t){0}) == (id == "
               "KINETIS_PROFILE_MK22FN1M012 || id == KINETIS_PROFILE_MK22FX51212)");
        if (id != KINETIS_PROFILE_MK22FN1M012 && id != KINETIS_PROFILE_MK22FX51212)
            expect(state, !kinetis_timing_write(&timing, SIM_SCGC3, 4, 0u),
                   "profile without SCGC3 rejects the register");
        kinetis_timing_test_expect_read(state, &timing, RCM_SRS0, 1, 0x82u);
        kinetis_timing_test_expect_read(
            state, &timing, 0x40037000u, 4,
            id == KINETIS_PROFILE_MK22FN1M012 || id == KINETIS_PROFILE_MK22FX51212 ? 2u : 6u);
        if (kinetis_profile_has_peripheral(profile, KINETIS_PERIPHERAL_RTC))
            kinetis_timing_test_expect_read(state, &timing, 0x4003d014u, 4, 1u);
        else
            expect(state, !kinetis_timing_read(&timing, 0x4003d014u, 4, &(uint32_t){0}),
                   "profile without RTC rejects the register");
        kinetis_timing_test_expect_read(state, &timing, 0x40052000u, 2, 0x01d3u);
        expect(state, kinetis_timing_core_clock_hz(&timing) == 20971520u,
               "kinetis_timing_core_clock_hz(&timing) == 20971520u");
        expect(state, kinetis_timing_bus_clock_hz(&timing) == 20971520u,
               "kinetis_timing_bus_clock_hz(&timing) == 20971520u");
        expect(state, !kinetis_timing_read(&timing, MCG_S + 1u, 1, &(uint32_t){0}),
               "!kinetis_timing_read(&timing, MCG_S + 1u, 1, &(uint32_t){0})");
        expect(state, !kinetis_timing_read(&timing, SIM_SDID, 1, &(uint32_t){0}),
               "!kinetis_timing_read(&timing, SIM_SDID, 1, &(uint32_t){0})");
        expect(state, !kinetis_timing_write(&timing, SIM_SDID, 4, 0),
               "!kinetis_timing_write(&timing, SIM_SDID, 4, 0)");
    }
}

void kinetis_timing_test_test_clock_tree_and_power(TestState* state, KinetisTiming* timing) {
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
    kinetis_timing_test_expect_write(state, timing, MCG_C5, 1, 0);
    kinetis_timing_test_expect_write(state, timing, MCG_C6, 1, 0x40u);
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
