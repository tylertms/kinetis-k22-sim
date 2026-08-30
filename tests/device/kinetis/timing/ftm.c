#include "device/kinetis/timing/support.h"

static void expect_pdb_back_to_back_mapping(TestState* state, KinetisTiming* timing,
                                            Observations* observations, uint8_t adc_instance,
                                            uint8_t adc_pretrigger, uint8_t expected_channel,
                                            uint8_t expected_pretrigger) {
    memset(timing->pdb_adc_locks, 0, sizeof(timing->pdb_adc_locks));
    memset(timing->pdb_back_to_back_pending, 0, sizeof(timing->pdb_back_to_back_pending));
    memset(timing->pdb_back_to_back_cycles, 0, sizeof(timing->pdb_back_to_back_cycles));
    memset(timing->pdb_channel_trigger_pending, 0, sizeof(timing->pdb_channel_trigger_pending));
    memset(timing->pdb_channel_trigger_cycles, 0, sizeof(timing->pdb_channel_trigger_cycles));
    timing->pdb_registers[0][0x10u >> 2u] = 0x30003u;
    timing->pdb_registers[0][0x38u >> 2u] = 0x30003u;
    timing->pdb_sc[0] = 0x80u;
    timing->pdb_mod[0] = UINT16_MAX;
    timing->pdb_counter[0] = 0u;
    timing->pdb_prescaler_cycles[0] = 0u;
    timing->pdb_bus_remainder[0] = 0u;
    timing->pdb_running[0] = true;
    timing->pdb_bypass_cycles[0] = 0u;

    const uint32_t triggers_before = observations->adc_triggers;
    kinetis_timing_adc_complete(timing, adc_instance, adc_pretrigger);
    expect(state,
           timing->pdb_back_to_back_pending[0][expected_channel] ==
                   (1u << expected_pretrigger) &&
               timing->pdb_back_to_back_pending[0][expected_channel ^ 1u] == 0u,
           "ADC acknowledgement selects one PDB back-to-back ring pretrigger");
    kinetis_timing_internal_advance_pdb(
        timing, 0u,
        kinetis_timing_test_cycles_for_ticks(timing, 3u, timing->bus_clock_hz));
    expect(state, observations->adc_triggers == triggers_before + 1u,
           "one ADC acknowledgement triggers one back-to-back pretrigger");
}

void kinetis_timing_test_test_pdb(TestState* state, KinetisTiming* timing,
                                  Observations* observations) {
    const uint32_t original_core_clock_hz = timing->core_clock_hz;
    const uint32_t original_bus_clock_hz = timing->bus_clock_hz;
    timing->core_clock_hz = 120000000u;
    timing->bus_clock_hz = 60000000u;
    timing->sim_scgc6 |= 1u << 22u;
    timing->pdb_sc[0] = 0x708cu;
    timing->pdb_mod[0] = UINT16_MAX;
    timing->pdb_counter[0] = 0u;
    timing->pdb_remainder[0] = 0u;
    timing->pdb_running[0] = true;
    kinetis_timing_internal_advance_pdb(timing, 0u, 10239u);
    expect(state, timing->pdb_counter[0] == 0u,
           "PDB waits for the complete divided clock period");
    kinetis_timing_internal_advance_pdb(timing, 0u, 1u);
    expect(state, timing->pdb_counter[0] == 1u, "PDB preserves the exact prescaled clock ratio");
    timing->core_clock_hz = original_core_clock_hz;
    timing->bus_clock_hz = original_bus_clock_hz;
    timing->pdb_sc[0] = 0u;
    timing->pdb_counter[0] = 0u;
    timing->pdb_remainder[0] = 0u;

    kinetis_timing_test_expect_write(state, timing, SIM_SCGC6, 4, timing->sim_scgc6 | (1u << 22u));
    timing->pdb_sc[0] = 0x80u;
    timing->pdb_mod[0] = 10u;
    timing->pdb_idly[0] = 0u;
    expect(state, kinetis_timing_trigger_pdb_input(timing, 0u),
           "PDB accepts the software test trigger");
    expect(state, (timing->pdb_sc[0] & (1u << 6u)) == 0u,
           "PDB delay interrupt waits until IDLY plus one");
    kinetis_timing_internal_advance_pdb(
        timing, 0u,
        kinetis_timing_test_cycles_for_ticks(timing, 1u, timing->bus_clock_hz));
    expect(state, (timing->pdb_sc[0] & (1u << 6u)) != 0u,
           "PDB delay interrupt asserts on the clock after IDLY");
    timing->pdb_sc[0] = 0u;
    kinetis_timing_test_expect_write(state, timing, PDB_MOD, 4, 10u);
    kinetis_timing_test_expect_write(state, timing, PDB_IDLY, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x10u, 4, 0x303u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x18u, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x1cu, 4, 4u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x150u, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x154u, 4, 2u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x10fa3u);
    kinetis_timing_advance(timing, 4u);
    expect(state, observations->adc_triggers == 1u, "first delayed PDB pretrigger waits two bus cycles");
    kinetis_timing_adc_complete(timing, 0u, 0u);
    kinetis_timing_advance(timing, 3u);
    kinetis_timing_test_expect_read(state, timing, PDB_CNT, 4, 7u);
    kinetis_timing_test_expect_read(state, timing, PDB_SC, 4, 0xfe2u);
    kinetis_timing_test_expect_read(state, timing, PDB_SC + 0x14u, 4, 0x30000u);
    expect(state, observations->adc_triggers == 2u, "observations->adc_triggers == 2u");
    expect(state, observations->dac_triggers == 1u, "observations->dac_triggers == 1u");
    expect(state, observations->last_trigger_instance == 0u,
           "observations->last_trigger_instance == 0u");
    expect(state, observations->last_trigger_channel == 1u,
           "observations->last_trigger_channel == 1u");
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x14u, 4, 0u);
    kinetis_timing_test_expect_read(state, timing, PDB_SC + 0x14u, 4, 0u);
    expect(state, observations->irq[52], "observations->irq[52]");

    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PDB_MOD, 4, 10u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x10u, 4, 1u);
    const uint32_t bypass_before = observations->adc_triggers;
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x10f81u);
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->adc_triggers == bypass_before,
           "PDB bypass waits for two peripheral clocks");
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->adc_triggers == bypass_before,
           "PDB channel trigger follows the bypass pretrigger");
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->adc_triggers == bypass_before + 1u,
           "PDB bypass reaches the ADC after the channel delay");
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x10f80u);
    kinetis_timing_advance(timing, 3u);
    kinetis_timing_test_expect_read(state, timing, PDB_SC + 0x14u, 4, 0x10001u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x14u, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x10u, 4, 0u);

    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x10u, 4, 0x20103u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x18u, 4, 0u);
    const uint32_t back_to_back_before = observations->adc_triggers;
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x10f81u);
    kinetis_timing_advance(timing, 3u);
    expect(state, observations->adc_triggers == back_to_back_before + 1u,
           "PDB delayed pretrigger zero uses the fixed two-cycle delay");
    kinetis_timing_adc_complete(timing, 0u, 0u);
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->adc_triggers == back_to_back_before + 1u,
           "PDB back-to-back trigger waits for two peripheral clocks");
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->adc_triggers == back_to_back_before + 1u,
           "PDB back-to-back pretrigger precedes its channel trigger");
    kinetis_timing_advance(timing, 1u);
    expect(state, observations->adc_triggers == back_to_back_before + 2u,
           "PDB back-to-back trigger follows ADC acknowledgement");

    expect_pdb_back_to_back_mapping(state, timing, observations, 0u, 0u, 0u, 1u);
    expect_pdb_back_to_back_mapping(state, timing, observations, 0u, 1u, 1u, 0u);
    expect_pdb_back_to_back_mapping(state, timing, observations, 1u, 0u, 1u, 1u);
    expect_pdb_back_to_back_mapping(state, timing, observations, 1u, 1u, 0u, 0u);

    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x150u, 4, 3u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x154u, 4, 1u);
    const uint32_t external_dac_before = observations->dac_triggers;
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x10f81u);
    kinetis_timing_advance(timing, 2u);
    expect(state, observations->dac_triggers == external_dac_before,
           "PDB DAC external mode bypasses the interval counter");
    expect(state, kinetis_timing_trigger_pdb_dac_input(timing, 0u, 0u),
           "PDB DAC external input accepts an enabled edge");
    expect(state, observations->dac_triggers == external_dac_before + 1u,
           "PDB DAC external input emits the trigger pulse");

    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x190u, 4, 1u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC + 0x194u, 4, 0x00010003u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x10f81u);
    kinetis_timing_advance(timing, 1u);
    expect(state, kinetis_timing_pdb_pulse_output(timing, 0u, 0u),
           "PDB pulse output rises at DLY1");
    kinetis_timing_advance(timing, 2u);
    expect(state, !kinetis_timing_pdb_pulse_output(timing, 0u, 0u),
           "PDB pulse output falls at DLY2");

    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0u);
    const uint16_t loaded_modulo = timing->pdb_mod[0];
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x80380u);
    kinetis_timing_test_expect_write(state, timing, PDB_MOD, 4, 7u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x80381u);
    kinetis_timing_test_expect_write(state, timing, PDB_MOD, 4, 9u);
    kinetis_timing_test_expect_read(state, timing, PDB_MOD, 4, loaded_modulo);
    expect(state, timing->pdb_mod[0] == loaded_modulo,
           "trigger load mode keeps buffered values inactive until a trigger");
    expect(state, (timing->pdb_sc[0] & 1u) != 0u, "LDOK remains set while a trigger load waits");
    expect(state, kinetis_timing_trigger_pdb_input(timing, 3u),
           "selected external PDB input triggers the counter");
    expect(state, timing->pdb_mod[0] == 7u, "trigger load mode transfers the buffered modulus");
    expect(state, (timing->pdb_sc[0] & 1u) == 0u, "trigger load clears LDOK");
    expect(state, timing->pdb_running[0], "selected trigger starts the PDB counter");
    kinetis_timing_advance(timing, 8u);
    expect(state, !timing->pdb_running[0], "one-shot PDB stops after modulus overflow");
    expect(state, timing->pdb_counter[0] == 0u, "one-shot PDB stops at zero");
    expect(state, !kinetis_timing_trigger_pdb_input(timing, 15u),
           "software trigger is not exposed as an external input");

    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, timing, PDB_MOD, 4, 5u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x81u);
    expect(state, timing->pdb_mod[0] == 5u, "immediate load establishes the active modulus");
    kinetis_timing_test_expect_write(state, timing, PDB_MOD, 4, 7u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x80381u);
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x80300u);
    expect(state, timing->pdb_mod[0] == 5u,
           "disabling PDB abandons a buffered update without loading it");
    expect(state, (timing->pdb_sc[0] & 1u) == 0u, "disabling PDB clears LDOK");
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x80380u);
    expect(state, kinetis_timing_trigger_pdb_input(timing, 3u),
           "PDB restarts without a new load request");
    expect(state, timing->pdb_mod[0] == 5u,
           "a later trigger does not load the abandoned buffered update");
    kinetis_timing_test_expect_write(state, timing, PDB_SC, 4, 0x23u);
    expect(state, !observations->irq[52], "!observations->irq[52]");
}

static void test_ftm_signed_up_count(TestState* state) {
    KinetisTiming timing;
    Observations observations = {0};
    expect(state,
           kinetis_timing_init(&timing, kinetis_profile_get(KINETIS_PROFILE_MKV10Z1287), 8000000u,
                               32768u, kinetis_timing_test_signals(&observations)),
           "signed FTM timing state initializes");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    kinetis_timing_test_expect_write(state, &timing, SIM_SCGC6, 4,
                                     timing.sim_scgc6 | (1u << 24u));
    kinetis_timing_test_expect_write(state, &timing, FTM0_MODE, 4, 5u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNTIN, 4, 0xf736u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4, 0x08c9u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x10u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    kinetis_timing_advance(&timing, 2249u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4, 0xffffu);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0x10u);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0SC, 4, 0x90u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x10u);
    kinetis_timing_advance(&timing, 2249u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4, 0x08c9u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_SC, 4, 8u);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4, 0xf736u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_SC, 4, 0x88u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4, 0x28u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4, 8u);
    kinetis_timing_advance(&timing, 4500u);
    bool output = false;
    expect(state, kinetis_timing_get_ftm_output(&timing, 0u, 0u, &output),
           "signed edge-aligned PWM output is available");
    expect(state, output, "signed edge-aligned PWM activates at CNTIN");
    kinetis_timing_advance(&timing, 2250u);
    expect(state, kinetis_timing_get_ftm_output(&timing, 0u, 0u, &output),
           "signed edge-aligned PWM output remains available");
    expect(state, !output, "signed edge-aligned PWM deactivates at its compare value");
}

void kinetis_timing_test_test_ftm(TestState* state, KinetisTiming* timing,
                                  Observations* observations) {
    test_ftm_signed_up_count(state);
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
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4u, 2u << 3u);
    kinetis_timing_test_expect_write(state, &timing, MCG_C2, 1u, 0x13u);
    kinetis_timing_test_expect_write(state, &timing, MCG_C1, 1u, 0x42u);
    expect(state, kinetis_timing_internal_fixed_clock_hz(&timing) == 0u,
           "FTM fixed-frequency source stops in BLPI");
    kinetis_timing_advance(&timing, 640u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 0u);
    kinetis_timing_test_expect_write(state, &timing, MCG_C2, 1u, 0x10u);
    kinetis_timing_test_expect_write(state, &timing, MCG_C1, 1u, 0x18u);
    kinetis_timing_advance(&timing, 640u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 1u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4u, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNT, 4u, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4u, 2u << 3u);
    kinetis_timing_set_cpu_sleeping(&timing, true, true);
    expect(state, kinetis_timing_internal_fixed_clock_hz(&timing) == 0u,
           "FTM fixed-frequency source stops in Stop mode");
    kinetis_timing_advance(&timing, 640u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_CNT, 4u, 0u);
    kinetis_timing_set_cpu_sleeping(&timing, false, false);
    kinetis_timing_advance(&timing, 640u);
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

void kinetis_timing_test_test_mkv10_ftm_dma(TestState* state) {
    KinetisTiming timing;
    Observations observations = {0};
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MKV10Z1287);
    expect(state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "MKV10 FTM DMA timing state initializes");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    kinetis_timing_test_expect_write(state, &timing, SIM_SCGC6, 4u,
                                     timing.sim_scgc6 | (1u << 6u));
    kinetis_timing_test_expect_write(state, &timing, 0x40026008u, 4u, 2u);
    kinetis_timing_test_expect_write(state, &timing, 0x40026030u, 4u, 1u);
    kinetis_timing_test_expect_write(state, &timing, 0x4002602cu, 4u, 0x51u);
    kinetis_timing_test_expect_write(state, &timing, 0x40026000u, 4u, 8u);
    kinetis_timing_advance(&timing, 1u);
    expect(state, observations.dma_requests == 1u,
           "MKV10 FTM3 channel 4 requests DMA on compare");
    expect(state, observations.last_dma == 54u,
           "MKV10 FTM3 channel 4 uses DMA request source 54");
    timing.ftm[3].channel_sc[4] |= 0x80u;
    timing.ftm[3].channel_flag_read[4] = true;
    kinetis_timing_internal_ftm_dma_complete(&timing, 54u);
    expect(state,
           (timing.ftm[3].channel_sc[4] & 0x80u) == 0u &&
               !timing.ftm[3].channel_flag_read[4],
           "FTM channel DMA completion clears its event flag");

    timing.sim_scgc6 |= (1u << 24u) | (1u << 25u) | (1u << 6u) | (1u << 7u);
    for (uint8_t instance = 0u; instance < 5u; instance++) {
        timing.ftm[instance].counter = 0u;
        timing.ftm[instance].modulo = 100u;
        timing.ftm[instance].sc = 8u;
    }
    timing.ftm[1].registers[12] = 1u << 9u;
    timing.ftm[4].registers[12] = 1u << 9u;
    kinetis_timing_advance(&timing, 1u);
    expect(state,
           timing.ftm[0].counter == 1u && timing.ftm[1].counter == 0u &&
               timing.ftm[3].counter == 1u && timing.ftm[4].counter == 0u,
           "MKV10 global time-base participants wait for their group output");
    timing.ftm[0].registers[12] = 1u << 10u;
    timing.ftm[3].registers[12] = 1u << 10u;
    kinetis_timing_advance(&timing, 1u);
    expect(state,
           timing.ftm[0].counter == 2u && timing.ftm[1].counter == 1u &&
               timing.ftm[3].counter == 2u && timing.ftm[4].counter == 1u,
           "MKV10 FTM0 and FTM3 release their global time-base groups");
}

static void test_ftm_dual_edge_capture(TestState* state, const KinetisDeviceProfile* profile) {
    KinetisTiming timing;
    Observations observations = {0};
    expect(state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "dual-edge capture fixture initializes");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    timing.sim_scgc6 |= 1u << 24u;
    timing.ftm[0].registers[0] = 4u;
    timing.ftm[0].modulo = 100u;
    timing.ftm[0].sc = 8u;
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4u, 0x44u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C1SC, 4u, 0x48u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4u, 0x0cu);

    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, true),
           "dual-edge initial input rises");
    kinetis_timing_advance(&timing, 3u);
    expect(state,
           timing.ftm[0].dual_capture_first[0] == 3u &&
               (timing.ftm[0].channel_sc[0] & 0x80u) != 0u,
           "dual-edge initial edge enters the even capture buffer");
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, false),
           "dual-edge final input falls");
    kinetis_timing_advance(&timing, 3u);
    expect(state,
           timing.ftm[0].channel_value[0] == 3u && timing.ftm[0].dual_capture_second[0] == 6u &&
               (timing.ftm[0].channel_sc[1] & 0x80u) != 0u &&
               (timing.ftm[0].registers[4] & 8u) == 0u,
           "one-shot dual-edge capture completes and clears DECAP");
    kinetis_timing_test_expect_read(state, &timing, FTM0_C1V, 4u, 0u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0V, 4u, 3u);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C1V, 4u, 6u);

    kinetis_timing_test_expect_write(state, &timing, FTM0_COMBINE, 4u, 0x0cu);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, true),
           "flagged one-shot dual-edge input rises");
    kinetis_timing_advance(&timing, 3u);
    expect(state, timing.ftm[0].dual_capture_first[0] == 3u,
           "one-shot dual-edge capture waits for both flags to clear");
    kinetis_timing_test_expect_write(state, &timing, FTM0_STATUS, 4u, 0u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4u, 0x54u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C1SC, 4u, 0x58u);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, false),
           "continuous dual-edge input returns low");
    kinetis_timing_advance(&timing, 3u);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 1u, true),
           "continuous dual-edge odd channel changes");
    kinetis_timing_advance(&timing, 3u);
    expect(state, !timing.ftm[0].dual_capture_waiting_final[0],
           "dual-edge capture ignores the odd channel input");
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, true),
           "continuous dual-edge initial input rises");
    kinetis_timing_advance(&timing, 3u);
    expect(state, kinetis_timing_set_ftm_input(&timing, 0u, 0u, false),
           "continuous dual-edge final input falls");
    kinetis_timing_advance(&timing, 3u);
    expect(state,
           (timing.ftm[0].registers[4] & 8u) != 0u &&
               !timing.ftm[0].dual_capture_waiting_final[0],
           "continuous dual-edge capture remains active for the next pair");
    const uint16_t continuous_first = timing.ftm[0].channel_value[0];
    const uint16_t continuous_second = timing.ftm[0].dual_capture_second[0];
    kinetis_timing_test_expect_read(state, &timing, FTM0_C0V, 4u, continuous_first);
    kinetis_timing_test_expect_read(state, &timing, FTM0_C1V, 4u, continuous_second);
}

void kinetis_timing_test_test_ftm_input_capture(TestState* state,
                                                const KinetisDeviceProfile* profile) {
    test_ftm_dual_edge_capture(state, profile);
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

static void test_ftm_debug_modes(TestState* state, const KinetisDeviceProfile* profile) {
    KinetisTiming timing;
    Observations observations = {0};
    expect(state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "initialize FTM debug mode fixture");
    kinetis_timing_test_disable_watchdog_fixture(&timing);
    kinetis_timing_test_expect_write(state, &timing, SIM_SCGC6, 4u,
                                     timing.sim_scgc6 | (1u << 24u));
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4u, 7u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4u, 3u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0SC, 4u, 0x14u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_SC, 4u, 8u);
    kinetis_timing_advance(&timing, 1u);
    kinetis_timing_set_debug_halted(&timing, true);
    kinetis_timing_advance(&timing, 4u);
    expect(state, timing.ftm[0].counter == 1u, "BDMMODE 00 stops the counter");
    kinetis_timing_test_expect_write(state, &timing, FTM0_MOD, 4u, 9u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_C0V, 4u, 5u);
    kinetis_timing_test_expect_write(state, &timing, FTM0_CNTIN, 4u, 2u);
    expect(state,
           timing.ftm[0].modulo == 9u && timing.ftm[0].channel_value[0] == 5u &&
               timing.ftm[0].initial == 2u && !timing.ftm[0].modulo_pending &&
               !timing.ftm[0].channel_value_pending[0] && !timing.ftm[0].initial_pending,
           "stopped BDM modes bypass FTM register buffers");
    kinetis_timing_set_debug_halted(&timing, false);

    timing.ftm[0].registers[12] = 1u << 6u;
    timing.ftm[0].registers[7] = 0u;
    timing.ftm[0].channel_output[0] = true;
    kinetis_timing_set_debug_halted(&timing, true);
    expect_ftm_output(state, &timing, false);
    timing.ftm[0].registers[7] = 1u;
    expect_ftm_output(state, &timing, true);
    kinetis_timing_set_debug_halted(&timing, false);

    timing.ftm[0].registers[12] = 2u << 6u;
    timing.ftm[0].registers[7] = 0u;
    timing.ftm[0].channel_output[0] = true;
    kinetis_timing_set_debug_halted(&timing, true);
    timing.ftm[0].channel_output[0] = false;
    expect_ftm_output(state, &timing, true);
    kinetis_timing_set_debug_halted(&timing, false);

    timing.ftm[0].registers[12] = 3u << 6u;
    timing.ftm[0].counter = 0u;
    timing.ftm[0].initial = 0u;
    timing.ftm[0].modulo = 7u;
    timing.ftm[0].sc = 8u;
    kinetis_timing_set_debug_halted(&timing, true);
    kinetis_timing_advance(&timing, 3u);
    expect(state, timing.ftm[0].counter == 3u, "BDMMODE 11 remains fully functional");
    kinetis_timing_set_debug_halted(&timing, false);

    timing.ftm[0].registers[12] = 1u << 6u;
    timing.ftm[0].channel_sc[0] = 4u;
    timing.ftm[0].channel_filtered_input[0] = false;
    timing.ftm[0].channel_input[0] = false;
    timing.ftm[0].channel_input_age[0] = 0u;
    kinetis_timing_set_debug_halted(&timing, true);
    kinetis_timing_set_ftm_input(&timing, 0u, 0u, true);
    kinetis_timing_advance(&timing, 4u);
    expect(state,
           timing.ftm[0].channel_value[0] == timing.ftm[0].counter &&
               (timing.ftm[0].channel_sc[0] & 0x80u) == 0u,
           "BDMMODE 01 captures the frozen counter without setting CHnF");
    kinetis_timing_set_debug_halted(&timing, false);
}

void kinetis_timing_test_test_ftm_output(TestState* state, const KinetisDeviceProfile* profile) {
    test_ftm_debug_modes(state, profile);
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
