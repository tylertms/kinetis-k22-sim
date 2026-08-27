#ifndef KINETIS_SIM_TIMING_H
#define KINETIS_SIM_TIMING_H

#include "device/kinetis/variants/profile.h"

#include <stdbool.h>
#include <stdint.h>

typedef void (*KinetisTimingIrqSignal)(void* context, uint8_t irq, bool asserted);
typedef void (*KinetisTimingDmaSignal)(void* context, uint8_t source);
typedef void (*KinetisTimingDmaTriggerSignal)(void* context, uint8_t channel);
typedef void (*KinetisTimingResetSignal)(void* context, uint8_t srs0, uint8_t srs1);

typedef enum {
    KINETIS_TIMING_TRIGGER_PDB_ADC,
    KINETIS_TIMING_TRIGGER_PDB_DAC,
    KINETIS_TIMING_TRIGGER_ADC_ALTERNATE,
} KinetisTimingTrigger;

typedef void (*KinetisTimingTriggerSignal)(void* context, KinetisTimingTrigger trigger,
                                           uint8_t instance, uint8_t channel);

typedef struct {
    void* context;
    KinetisTimingIrqSignal irq;
    KinetisTimingDmaSignal dma;
    KinetisTimingResetSignal reset;
    KinetisTimingTriggerSignal trigger;
    KinetisTimingDmaTriggerSignal dma_trigger;
} KinetisTimingSignals;

typedef struct {
    uint32_t load;
    uint32_t current;
    uint32_t control;
    bool flag;
} KinetisPitChannel;

typedef struct {
    uint32_t sc;
    uint16_t counter;
    uint16_t modulo;
    uint16_t initial;
    uint32_t channel_sc[8];
    uint16_t channel_value[8];
    uint16_t channel_value_buffer[8];
    uint32_t registers[20];
    uint16_t modulo_buffer;
    uint16_t initial_buffer;
    uint32_t outmask_buffer;
    uint32_t invctrl_buffer;
    uint32_t swoctrl_buffer;
    uint64_t remainder;
    bool trigger_flag_read;
    bool overflow_flag_read;
    bool channel_flag_read[8];
    bool channel_input[8];
    bool channel_filtered_input[8];
    bool channel_output[8];
    bool channel_deadtime_output[8];
    bool fault_input[4];
    bool fault_filtered_input[4];
    bool channel_value_pending[8];
    uint32_t channel_input_age[8];
    uint32_t channel_deadtime_remaining[8];
    uint32_t fault_input_age[4];
    bool outmask_pending;
    bool invctrl_pending;
    bool swoctrl_pending;
    bool modulo_pending;
    bool initial_pending;
    bool software_sync_pending;
    bool hardware_sync_pending;
    bool write_protection_read;
    bool fault_aggregate_read;
    bool fault_output_active;
    bool fault_release_pending;
    bool quadrature_capable;
    bool counting_down;
    uint8_t hardware_trigger_pending_mask;
    uint8_t fault_flags_read_mask;
    uint8_t overflow_count;
    uint8_t external_clock_edges;
    uint64_t deadtime_remainder;
    uint64_t fault_remainder;
} KinetisFtmState;

typedef struct {
    const KinetisDeviceProfile* profile;
    KinetisTimingSignals signals;
    uint32_t external_oscillator_hz;
    uint32_t rtc_oscillator_hz;
    uint32_t slow_irc_hz;
    uint32_t fast_irc_hz;
    uint32_t lpo_hz;
    uint32_t core_clock_hz;
    uint32_t bus_clock_hz;
    uint32_t flash_clock_hz;
    uint64_t pll_lock_attoseconds;
    uint16_t pll_configuration;
    bool pll_locked;
    uint64_t elapsed_core_cycles;
    bool debug_halted;
    uint8_t sim_sdid_pin_id;
    uint32_t sim_sopt1;
    uint32_t sim_sopt1cfg;
    uint32_t sim_sopt2;
    uint32_t sim_sopt4;
    uint32_t sim_sopt5;
    uint32_t sim_sopt7;
    uint32_t sim_sopt8;
    uint32_t sim_scgc3;
    uint32_t sim_scgc4;
    uint32_t sim_scgc5;
    uint32_t sim_scgc6;
    uint32_t sim_scgc7;
    uint32_t sim_clkdiv1;
    uint32_t sim_clkdiv2;
    bool ftm_clock_input[2];
    uint8_t mcg[14];
    uint8_t osc_cr;
    uint8_t osc_div;
    uint8_t llwu[11];
    uint8_t pmc[3];
    bool pmc_lvdre_written;
    uint8_t smc[4];
    uint8_t smc_run_status;
    bool cpu_sleeping;
    bool deep_sleeping;
    bool llwu_pin_level[16];
    uint8_t rcm[10];
    uint32_t pit_mcr;
    KinetisPitChannel pit[4];
    uint64_t pit_remainder;
    uint32_t lptmr_csr;
    uint32_t lptmr_psr;
    uint32_t lptmr_cmr;
    uint16_t lptmr_counter;
    uint16_t lptmr_latched_counter;
    uint64_t lptmr_remainder;
    uint64_t lptmr_filter_remainder;
    uint32_t lptmr_filter_ticks;
    bool lptmr_input[3];
    bool lptmr_observed_active;
    uint32_t rtc_tsr;
    uint16_t rtc_tpr;
    uint32_t rtc_tar;
    uint32_t rtc_tcr;
    uint32_t rtc_cr;
    uint32_t rtc_sr;
    uint32_t rtc_lr;
    uint32_t rtc_ier;
    uint32_t rtc_war;
    uint32_t rtc_rar;
    uint64_t rtc_remainder;
    uint32_t rtc_subsecond_ticks;
    uint32_t pdb_sc;
    uint16_t pdb_mod;
    uint16_t pdb_counter;
    uint16_t pdb_idly;
    uint32_t pdb_registers[104];
    uint64_t pdb_remainder;
    KinetisFtmState ftm[4];
    uint16_t wdog[12];
    uint16_t wdog_pending[12];
    uint32_t wdog_counter;
    uint32_t wdog_update_mask;
    uint8_t wdog_unlock_stage;
    uint8_t wdog_refresh_stage;
    uint64_t wdog_remainder;
    uint64_t wdog_bus_remainder;
    uint64_t wdog_bus_cycles;
    uint64_t wdog_sequence_deadline;
    uint64_t wdog_update_ready;
    uint64_t wdog_update_deadline;
    uint64_t wdog_reset_deadline;
    uint64_t wdog_initial_debug_remaining;
    bool wdog_initial_unlock_required;
    bool wdog_initial_debug_pause;
    bool wdog_update_open;
    bool wdog_update_written;
    bool wdog_reset_pending;
    uint8_t ewm_ctrl;
    uint8_t ewm_cmpl;
    uint8_t ewm_cmph;
    uint8_t ewm_prescaler;
    uint8_t ewm_service_stage;
    bool ewm_control_written;
    bool ewm_cmpl_written;
    bool ewm_cmph_written;
    bool ewm_prescaler_written;
    bool ewm_input;
    bool ewm_output;
    uint32_t ewm_counter;
    uint64_t ewm_remainder;
    uint64_t ewm_service_deadline;
    uint64_t ewm_service_remaining;
    bool ewm_service_paused;
    uint64_t reset_generation;
} KinetisTiming;

bool kinetis_timing_init(KinetisTiming* timing, const KinetisDeviceProfile* profile,
                         uint32_t external_oscillator_hz, uint32_t rtc_oscillator_hz,
                         KinetisTimingSignals signals);
bool kinetis_timing_read(KinetisTiming* timing, uint32_t address, uint8_t size, uint32_t* value);
bool kinetis_timing_projected_watchdog_read(const KinetisTiming* timing, uint32_t address,
                                            uint8_t size, uint32_t* value);
bool kinetis_timing_write(KinetisTiming* timing, uint32_t address, uint8_t size, uint32_t value);
bool kinetis_timing_set_reset_state(KinetisTiming* timing, uint8_t srs0, bool ackiso);
void kinetis_timing_advance(KinetisTiming* timing, uint32_t core_cycles);
void kinetis_timing_set_debug_halted(KinetisTiming* timing, bool halted);
bool kinetis_timing_set_lptmr_input(KinetisTiming* timing, uint8_t input_index, bool input_high);
bool kinetis_timing_trigger_low_voltage_warning(KinetisTiming* timing);
bool kinetis_timing_trigger_low_voltage_detect(KinetisTiming* timing);
bool kinetis_timing_set_llwu_pin(KinetisTiming* timing, uint8_t pin, bool high);
bool kinetis_timing_trigger_llwu_module(KinetisTiming* timing, uint8_t module);
void kinetis_timing_set_cpu_sleeping(KinetisTiming* timing, bool sleeping, bool deep_sleep);
bool kinetis_timing_set_ewm_input(KinetisTiming* timing, bool high);
bool kinetis_timing_ewm_output(const KinetisTiming* timing);
void kinetis_timing_watchdog_advance(KinetisTiming* timing, uint32_t elapsed_watchdog_ticks);
bool kinetis_timing_set_ftm_input(KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                  bool input_high);
bool kinetis_timing_set_ftm_clock_input(KinetisTiming* timing, uint8_t input_index,
                                        bool input_high);
bool kinetis_timing_set_ftm_fault(KinetisTiming* timing, uint8_t instance, uint8_t input_index,
                                  bool input_high);
bool kinetis_timing_trigger_ftm_hardware(KinetisTiming* timing, uint8_t instance, uint8_t trigger);
bool kinetis_timing_get_ftm_output(const KinetisTiming* timing, uint8_t instance, uint8_t channel,
                                   bool* output_high);
void kinetis_timing_reset(KinetisTiming* timing, uint8_t srs0, uint8_t srs1);
void kinetis_timing_warm_reset(KinetisTiming* timing, uint8_t srs0, uint8_t srs1);
bool kinetis_timing_copy(KinetisTiming* destination, const KinetisTiming* source,
                         KinetisTimingSignals signals);
uint32_t kinetis_timing_core_clock_hz(const KinetisTiming* timing);
uint32_t kinetis_timing_bus_clock_hz(const KinetisTiming* timing);
uint32_t kinetis_timing_lpuart_clock_hz(const KinetisTiming* timing);
bool kinetis_timing_system_clock_running(const KinetisTiming* timing);
bool kinetis_timing_bus_clock_running(const KinetisTiming* timing);

#endif
