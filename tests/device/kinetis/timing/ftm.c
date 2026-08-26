#include "device/kinetis/timing/support.h"

void kinetis_timing_test_test_pdb(TestState* state, KinetisTiming* timing,
                                  Observations* observations) {
    const uint32_t original_core_clock_hz = timing->core_clock_hz;
    const uint32_t original_bus_clock_hz = timing->bus_clock_hz;
    timing->core_clock_hz = 120000000u;
    timing->bus_clock_hz = 60000000u;
    timing->sim_scgc6 |= 1u << 22u;
    timing->pdb_sc = 0x700du;
    timing->pdb_mod = UINT16_MAX;
    timing->pdb_counter = 0u;
    timing->pdb_remainder = 0u;
    kinetis_timing_internal_advance_pdb(timing, 10239u);
    expect(state, timing->pdb_counter == 0u, "PDB waits for the complete divided clock period");
    kinetis_timing_internal_advance_pdb(timing, 1u);
    expect(state, timing->pdb_counter == 1u, "PDB preserves the exact prescaled clock ratio");
    timing->core_clock_hz = original_core_clock_hz;
    timing->bus_clock_hz = original_bus_clock_hz;
    timing->pdb_sc = 0u;
    timing->pdb_counter = 0u;
    timing->pdb_remainder = 0u;

    kinetis_timing_test_expect_write(state, timing, SIM_SCGC6, 4, timing->sim_scgc6 | (1u << 22u));
    kinetis_timing_test_expect_write(state, timing, PDB_MOD, 4, 4u);
    kinetis_timing_test_expect_write(state, timing, PDB_IDLY, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x10u, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x18u, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x1cu, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x150u, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x154u, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x23u);
    kinetis_timing_advance(timing, 2u);
    kinetis_timing_test_expect_read(state, timing, PDB_CNT, 4, 2u);
    kinetis_timing_test_expect_read(state, timing, PDB_SC, 4, 0x63u);
    kinetis_timing_test_expect_read(state, timing, PDB_SC + 0x14u, 4, 3u);
    expect(state, observations->adc_triggers == 2u, "observations->adc_triggers == 2u");
    expect(state, observations->dac_triggers == 1u, "observations->dac_triggers == 1u");
    expect(state, observations->last_trigger_instance == 0u,
           "observations->last_trigger_instance == 0u");
    expect(state, observations->last_trigger_channel == 0u,
           "observations->last_trigger_channel == 0u");
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x14u, 4, 3u);
    kinetis_timing_test_expect_read(state, timing, PDB_SC + 0x14u, 4, 0u);
    expect(state, observations->irq[52], "observations->irq[52]");
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x23u);
    expect(state, !observations->irq[52], "!observations->irq[52]");
}

void kinetis_timing_test_test_ftm(TestState* state, KinetisTiming* timing,
                                  Observations* observations) {
    kinetis_timing_test_expect_write(state, timing, SIM_SCGC6, 4, timing->sim_scgc6 | (1u << 24u));
    kinetis_timing_test_expect_write(state, timing, FTM0_MOD, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0V, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x51u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x10u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x48u);
    kinetis_timing_advance(timing, 4u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xc8u);
    expect(state, observations->irq[42], "observations->irq[42]");
    expect(state, observations->dma_requests != 0, "observations->dma_requests != 0");
    expect(state, observations->last_dma == 20u, "observations->last_dma == 20u");
    expect(state, observations->last_trigger_instance == 8u,
           "observations->last_trigger_instance == 8u");
    kinetis_timing_test_expect_read(state, timing, FTM0_EXTTRIG, 4, 0x90u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x90u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x10u);
    kinetis_timing_test_expect_read(state, timing, FTM0_EXTTRIG, 4, 0x90u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x10u);
    kinetis_timing_test_expect_read(state, timing, FTM0_EXTTRIG, 4, 0x10u);
    const uint32_t alternate_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x40u);
    kinetis_timing_advance(timing, 4u);
    expect(state, observations->alternate_triggers == alternate_before + 1u,
           "observations->alternate_triggers == alternate_before + 1u");
    kinetis_timing_test_expect_read(state, timing, FTM0_EXTTRIG, 4, 0xc0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x40u);
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xc8u);
    kinetis_timing_test_expect_read(state, timing, FTM0_C0SC, 4, 0xd1u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x51u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x48u);
    expect(state, !observations->irq[42], "!observations->irq[42]");

    kinetis_timing_test_expect_write(state, timing, FTM0_C0V, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C1V, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x50u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C1SC, 4, 0x50u);
    kinetis_timing_advance(timing, 4u);
    kinetis_timing_test_expect_read(state, timing, FTM0_STATUS, 4, 3u);
    expect(state, observations->irq[42], "observations->irq[42]");
    kinetis_timing_test_expect_read(state, timing, FTM0_C0SC, 4, 0xd0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x50u);
    expect(state, observations->irq[42], "observations->irq[42]");
    kinetis_timing_test_expect_read(state, timing, FTM0_STATUS, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, FTM0_STATUS, 4, 0u);
    expect(state, observations->irq[42], "observations->irq[42]");
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x48u);
    expect(state, observations->irq[42], "observations->irq[42]");
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xc8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x48u);
    expect(state, !observations->irq[42], "!observations->irq[42]");

    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_MOD, 4, 100u);
    kinetis_timing_test_expect_write(state, timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0V, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x50u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x08u);
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->irq[42], "observations->irq[42]");
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x50u);
    expect(state, observations->irq[42], "observations->irq[42]");
    kinetis_timing_test_expect_read(state, timing, FTM0_C0SC, 4, 0xd0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x50u);
    expect(state, !observations->irq[42], "!observations->irq[42]");

    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC + 0x4cu, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, FTM0_MOD, 4, 4u);
    const uint32_t center_write_trigger_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, FTM0_CNT, 4, 0xaaaau);
    expect(state, observations->alternate_triggers == center_write_trigger_before + 1u,
           "observations->alternate_triggers == center_write_trigger_before + 1u");
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C1SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C1V, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C1V + 8u, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0V, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x68u);
    const uint32_t center_start_trigger_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x68u);
    expect(state, observations->alternate_triggers == center_start_trigger_before + 1u,
           "observations->alternate_triggers == center_start_trigger_before + 1u");
    const uint32_t center_cycle_trigger_before = observations->alternate_triggers;
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 2u);
    kinetis_timing_test_expect_read(state, timing, FTM0_STATUS, 4, 3u);
    kinetis_timing_test_expect_read(state, timing, FTM0_C0SC, 4, 0xe8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x68u);
    kinetis_timing_test_expect_read(state, timing, FTM0_C1SC, 4, 0x80u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C1SC, 4, 0u);
    kinetis_timing_advance(timing, 2u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 4u);
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0x68u);
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 3u);
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xe8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x68u);
    expect(state, observations->alternate_triggers == center_cycle_trigger_before,
           "observations->alternate_triggers == center_cycle_trigger_before");
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 2u);
    kinetis_timing_test_expect_read(state, timing, FTM0_C0SC, 4, 0xe8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x68u);
    kinetis_timing_test_expect_read(state, timing, FTM0_C1SC, 4, 0x80u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C1SC, 4, 0u);
    kinetis_timing_advance(timing, 1u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 1u);
    kinetis_timing_test_expect_read(state, timing, FTM0_STATUS, 4, 0u);
    expect(state, observations->alternate_triggers == center_cycle_trigger_before + 1u,
           "observations->alternate_triggers == center_cycle_trigger_before + 1u");
    expect(state, !observations->irq[42], "!observations->irq[42]");
    kinetis_timing_set_debug_halted(timing, true);
    kinetis_timing_advance(timing, 3u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 1u);
    kinetis_timing_set_debug_halted(timing, false);
    const uint32_t center_dma_before = observations->dma_requests;
    const uint32_t center_trigger_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x10u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0x69u);
    kinetis_timing_advance(timing, 13u);
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 2u);
    expect(state, observations->dma_requests == center_dma_before + 1u,
           "observations->dma_requests == center_dma_before + 1u");
    expect(state, observations->alternate_triggers == center_trigger_before + 1u,
           "observations->alternate_triggers == center_trigger_before + 1u");
    kinetis_timing_test_expect_read(state, timing, FTM0_C0SC, 4, 0xe9u);
    kinetis_timing_test_expect_write(state, timing, FTM0_C0SC, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xe8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x40u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC + 0x4cu, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, FTM0_MOD, 4, 3u);
    const uint32_t redundant_write_trigger_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, FTM0_CNT, 4, 0u);
    expect(state, observations->alternate_triggers == redundant_write_trigger_before + 1u,
           "observations->alternate_triggers == redundant_write_trigger_before + 1u");
    const uint32_t redundant_start_trigger_before = observations->alternate_triggers;
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x68u);
    expect(state, observations->alternate_triggers == redundant_start_trigger_before + 1u,
           "observations->alternate_triggers == redundant_start_trigger_before + 1u");
    const uint32_t redundant_trigger_before = observations->alternate_triggers;
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->alternate_triggers == redundant_trigger_before + 1u,
           "observations->alternate_triggers == redundant_trigger_before + 1u");
    kinetis_timing_test_expect_read(state, timing, FTM0_CNT, 4, 3u);
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xe8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC + 0x4cu, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_MOD, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, FTM0_CONF, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, FTM0_EXTTRIG, 4, 0x40u);
    kinetis_timing_test_expect_write(state, timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x48u);
    const uint32_t periodic_trigger_before = observations->alternate_triggers;
    kinetis_timing_advance(timing, 2u);
    expect(state, observations->alternate_triggers == periodic_trigger_before + 1u,
           "observations->alternate_triggers == periodic_trigger_before + 1u");
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xc8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x48u);
    const uint32_t periodic_skip_trigger_before = observations->alternate_triggers;
    kinetis_timing_advance(timing, 4u);
    expect(state, observations->alternate_triggers == periodic_skip_trigger_before + 1u,
           "observations->alternate_triggers == periodic_skip_trigger_before + 1u");
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0x48u);
    const uint32_t periodic_resume_trigger_before = observations->alternate_triggers;
    kinetis_timing_advance(timing, 2u);
    expect(state, observations->alternate_triggers == periodic_resume_trigger_before + 1u,
           "observations->alternate_triggers == periodic_resume_trigger_before + 1u");
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xc8u);
    kinetis_timing_test_expect_write(state, timing, FTM0_SC, 4, 0x48u);
    kinetis_timing_test_expect_write(state, timing, FTM0_CNT, 4, 0xffffu);
    kinetis_timing_advance(timing, 2u);
    kinetis_timing_test_expect_read(state, timing, FTM0_SC, 4, 0xc8u);
    kinetis_timing_test_expect_write(state, timing, FTM1_EXTTRIG, 4, UINT32_MAX);
    kinetis_timing_test_expect_read(state, timing, FTM1_EXTTRIG, 4, 0x70u);
}

void kinetis_timing_test_test_ftm_clock_sources(TestState* state,
                                                const KinetisDeviceProfile* profile) {
    KinetisTiming timing;
    Observations observations = {0};
    expect(state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "FTM clock-source timing state initializes");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    kinetis_timing_test_expect_write(state, &timing, MCG_C2, 1u, 0x10u);
    kinetis_timing_test_expect_write(state, &timing, MCG_C1, 1u, 0x18u);
    expect(state, kinetis_timing_internal_fixed_clock_hz(&timing) == 31250u,
           "FTM fixed-frequency source follows the divided FLL reference");
    kinetis_timing_test_expect_write(state, &timing, SIM_SCGC6, 4u, timing.sim_scgc6 | (1u << 24));
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4u, 100u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4u, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4u, 2u << 3);
    kinetis_timing_advance(&timing, 639u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 0u);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 1u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4u, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4u, 0u);
    kinetis_timing_test_expect_write(state, &timing, SIM_SOPT4, 4u, 1u << 24);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4u, (3u << 3) | 1u);
    kinetis_timing_advance(&timing, 640u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 0u);
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 0u, true),
           "unselected FTM clock input accepts a rising edge");
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 0u);
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 0u, false),
           "unselected FTM clock input accepts a falling edge");
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 1u, true),
           "selected FTM clock input accepts its first rising edge");
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 0u);
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 1u, true),
           "steady FTM clock input is accepted");
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 1u, false),
           "selected FTM clock input accepts a falling edge");
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 1u, true),
           "selected FTM clock input accepts its second rising edge");
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 1u);

    kinetis_timing_test_expect_write(state, &timing, SIM_SOPT4, 4u, 0u);
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 0u, true),
           "rerouted FTM clock accepts its first rising edge");
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 1u);
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 0u, false),
           "rerouted FTM clock accepts a falling edge");
    expect(state, kinetis_timing_set_ftm_clock_input(&timing, 0u, true),
           "rerouted FTM clock accepts its second rising edge");
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 2u);
    expect(state, !kinetis_timing_set_ftm_clock_input(&timing, 2u, true),
           "invalid FTM clock input is rejected");
    expect(state, !kinetis_timing_set_ftm_clock_input(NULL, 0u, true),
           "null FTM timing state is rejected");
}

void kinetis_timing_test_test_ftm_input_capture(TestState* state,
                                                const KinetisDeviceProfile* profile) {
    KinetisTiming timing;
    Observations observations = {0};
    expect(state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "kinetis_timing_init(&timing, profile, 8000000u, 32768u, "
           "kinetis_timing_test_signals(&observations))");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    kinetis_timing_test_expect_write(state, &timing, SIM_SCGC6, 4, timing.sim_scgc6 | (1u << 24u));
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNTIN, 4, 5u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4, 100u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_EXTTRIG, 4, 0x40u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x47u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4, 0x1234u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0V, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, true),
           "kinetis_timing_set_ftm_input(&timing, 0u, 0u, true)");
    const uint32_t first_dma_before = observations.dma_requests;
    const uint32_t first_trigger_before = observations.alternate_triggers;
    kinetis_timing_advance(&timing, 2u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0V, 4, 0u);
    kinetis_timing_set_debug_halted(&timing, true);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0V, 4, 7u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4, 5u);
    expect(state, observations.irq[42], "observations.irq[42]");
    expect(state, observations.dma_requests == first_dma_before + 1u,
           "observations.dma_requests == first_dma_before + 1u");
    expect(state, observations.alternate_triggers == first_trigger_before + 1u,
           "observations.alternate_triggers == first_trigger_before + 1u");
    kinetis_timing_set_debug_halted(&timing, false);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0xc7u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x47u);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, false),
           "kinetis_timing_set_ftm_input(&timing, 0u, 0u, false)");
    kinetis_timing_advance(&timing, 3u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0x47u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x4du);
    kinetis_timing_test_expect_write(state, &timing, FTM0_FILTER, 4, 2u);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, true),
           "kinetis_timing_set_ftm_input(&timing, 0u, 0u, true)");
    const uint32_t filtered_dma_before = observations.dma_requests;
    kinetis_timing_advance(&timing, 11u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0x4du);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0V, 4, 20u);
    expect(state, observations.dma_requests == filtered_dma_before + 1u,
           "observations.dma_requests == filtered_dma_before + 1u");
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0xcdu);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x4du);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, false),
           "kinetis_timing_set_ftm_input(&timing, 0u, 0u, false)");
    kinetis_timing_advance(&timing, 4u);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, true),
           "kinetis_timing_set_ftm_input(&timing, 0u, 0u, true)");
    kinetis_timing_advance(&timing, 20u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0x4du);
    expect(state, !kinetis_timing_set_ftm_input(&timing, 1u, 2u, true),
           "!kinetis_timing_set_ftm_input(&timing, 1u, 2u, true)");
    expect(state, !kinetis_timing_set_ftm_input(&timing, 4u, 0u, true),
           "!kinetis_timing_set_ftm_input(&timing, 4u, 0u, true)");
    expect(state, !kinetis_timing_set_ftm_input(NULL, 0u, 0u, true),
           "!kinetis_timing_set_ftm_input(NULL, 0u, 0u, true)");
}

static void expect_ftm_output(TestState* state, const KinetisTiming* timing, bool expected) {
    bool high = !expected;
    expect(state, kinetis_timing_get_ftm_output(timing, 0u, 0u, &high),
           "kinetis_timing_get_ftm_output(timing, 0u, 0u, &high)");
    expect(state, high == expected, "high == expected");
}

void kinetis_timing_test_test_ftm_output(TestState* state, const KinetisDeviceProfile* profile) {
    KinetisTiming timing;
    Observations observations = {0};
    expect(state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "kinetis_timing_init(&timing, profile, 8000000u, 32768u, "
           "kinetis_timing_test_signals(&observations))");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    kinetis_timing_test_expect_write(state, &timing, SIM_SCGC6, 4, timing.sim_scgc6 | (1u << 24u));
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4, 5u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4, 3u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x14u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_advance(&timing, 3u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_advance(&timing, 6u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x1cu);
    kinetis_timing_advance(&timing, 6u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x18u);
    kinetis_timing_advance(&timing, 6u);
    expect_ftm_output(state, &timing, false);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTINIT, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x28u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4, 3u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    kinetis_timing_advance(&timing, 3u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_advance(&timing, 3u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x24u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTINIT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_advance(&timing, 3u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_advance(&timing, 3u);
    expect_ftm_output(state, &timing, false);

    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0xa4u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x28u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTINIT, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_advance(&timing, 6u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0xa8u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x28u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4, 6u);
    kinetis_timing_advance(&timing, 6u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0x28u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4, 4u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4, 2u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTINIT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x28u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0x28u);
    kinetis_timing_advance(&timing, 2u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_advance(&timing, 4u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_advance(&timing, 4u);
    expect_ftm_output(state, &timing, false);

    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0xa8u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x14u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_QDCTRL, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    kinetis_timing_advance(&timing, 3u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0x94u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_QDCTRL, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTINIT, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MODE, 4, 2u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_MODE, 4, 4u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 1u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_advance(&timing, 1u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 0x101u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_advance(&timing, 1u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTMASK, 4, 1u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_advance(&timing, 1u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTMASK, 4, 0u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_advance(&timing, 1u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 0u);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTINIT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MODE, 4, 2u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4, 4u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 0x101u);
    kinetis_timing_advance(&timing, 1u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4, 2u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 0x303u);
    kinetis_timing_advance(&timing, 1u);
    expect_ftm_output(state, &timing, true);
    bool channel_one = true;
    expect(state, kinetis_timing_get_ftm_output(&timing, 0u, 1u, &channel_one),
           "kinetis_timing_get_ftm_output(&timing, 0u, 1u, &channel_one)");
    expect(state, channel_one, "channel_one");

    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 0x101u);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_POL, 4, 1u);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_test_expect_write(state, &timing, FTM0_FMS, 4, 0x40u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_POL, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_POL, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4, 0x7fu);
    kinetis_timing_test_expect_read(state, &timing, FTM0_COMBINE, 4, 0x28u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_DEADTIME, 4, 0xffu);
    kinetis_timing_test_expect_read(state, &timing, FTM0_DEADTIME, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_FLTCTRL, 4, 0xfffu);
    kinetis_timing_test_expect_read(state, &timing, FTM0_FLTCTRL, 4, 0xf00u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_QDCTRL, 4, 0xf9u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_QDCTRL, 4, 0xf8u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_FLTPOL, 4, 0xfu);
    kinetis_timing_test_expect_read(state, &timing, FTM0_FLTPOL, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MODE, 4, 0xfdu);
    kinetis_timing_test_expect_read(state, &timing, FTM0_MODE, 4, 0x88u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_FMS, 4, 0x40u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MODE, 4, 4u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_MODE, 4, 4u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_FMS, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_POL, 4, 0u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4, UINT32_MAX);
    kinetis_timing_test_expect_read(state, &timing, FTM0_COMBINE, 4, 0x7f7f7f7fu);
    kinetis_timing_test_expect_write(state, &timing, FTM0_FMS, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_FMS, 4, 0u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 0u);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTINIT, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MODE, 4, 2u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_INVCTRL, 4, 1u);
    expect_ftm_output(state, &timing, true);
    kinetis_timing_advance(&timing, 1u);
    expect_ftm_output(state, &timing, true);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SYNCONF, 4,
                                     (1u << 12u) | (1u << 11u) | (1u << 10u) | (1u << 7u) |
                                         (1u << 5u) | (1u << 4u));
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SWOCTRL, 4, 1u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_INVCTRL, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SYNC, 4, 0x0au);
    kinetis_timing_test_expect_write(state, &timing, FTM0_OUTMASK, 4, 1u);
    kinetis_timing_advance(&timing, 2u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_SWOCTRL, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_INVCTRL, 4, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_OUTMASK, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SYNC, 4, 0x8au);
    kinetis_timing_test_expect_read(state, &timing, FTM0_SWOCTRL, 4, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_INVCTRL, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_OUTMASK, 4, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_SYNC, 4, 0x8au);
    expect_ftm_output(state, &timing, false);
    kinetis_timing_advance(&timing, 2u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_SYNC, 4, 0x0au);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4, 5u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    kinetis_timing_advance(&timing, 2u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4, 2u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SYNCONF, 4, (1u << 8u) | (1u << 7u));
    kinetis_timing_test_expect_write(state, &timing, FTM0_SYNC, 4, 0x80u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4, 0u);

    expect(state, !kinetis_timing_get_ftm_output(&timing, 1u, 2u, &(bool){false}),
           "!kinetis_timing_get_ftm_output(&timing, 1u, 2u, &(bool){false})");
    expect(state, !kinetis_timing_get_ftm_output(&timing, 4u, 0u, &(bool){false}),
           "!kinetis_timing_get_ftm_output(&timing, 4u, 0u, &(bool){false})");
    expect(state, !kinetis_timing_get_ftm_output(&timing, 0u, 0u, NULL),
           "!kinetis_timing_get_ftm_output(&timing, 0u, 0u, NULL)");
    expect(state, !kinetis_timing_get_ftm_output(NULL, 0u, 0u, &(bool){false}),
           "!kinetis_timing_get_ftm_output(NULL, 0u, 0u, &(bool){false})");
}
