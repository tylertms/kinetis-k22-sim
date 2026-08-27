#ifndef KINETIS_SIM_TIMING_INTERNAL_H
#define KINETIS_SIM_TIMING_INTERNAL_H

#include "device/kinetis/timing/api.h"

#include <limits.h>
#include <string.h>

enum {
    SIM_BASE = 0x40047000u,
    SIM_SOPT1 = SIM_BASE,
    SIM_SOPT1CFG = SIM_BASE + 0x4u,
    SIM_SCGC3 = SIM_BASE + 0x1030u,
    SIM_SOPT2 = SIM_BASE + 0x1004u,
    SIM_SOPT4 = SIM_BASE + 0x100cu,
    SIM_SOPT5 = SIM_BASE + 0x1010u,
    SIM_SOPT7 = SIM_BASE + 0x1018u,
    SIM_SOPT8 = SIM_BASE + 0x101cu,
    SIM_SDID = SIM_BASE + 0x1024u,
    SIM_SCGC4 = SIM_BASE + 0x1034u,
    SIM_SCGC5 = SIM_BASE + 0x1038u,
    SIM_SCGC6 = SIM_BASE + 0x103cu,
    SIM_SCGC7 = SIM_BASE + 0x1040u,
    SIM_CLKDIV1 = SIM_BASE + 0x1044u,
    SIM_CLKDIV2 = SIM_BASE + 0x1048u,
    SIM_FCFG1 = SIM_BASE + 0x104cu,
    SIM_FCFG2 = SIM_BASE + 0x1050u,
    MCG_BASE = 0x40064000u,
    OSC_BASE = 0x40065000u,
    LLWU_BASE = 0x4007c000u,
    PMC_BASE = 0x4007d000u,
    SMC_BASE = 0x4007e000u,
    RCM_BASE = 0x4007f000u,
    PIT_BASE = 0x40037000u,
    PIT_CHANNEL_BASE = PIT_BASE + 0x100u,
    LPTMR_BASE = 0x40040000u,
    RTC_BASE = 0x4003d000u,
    PDB_BASE = 0x40036000u,
    WDOG_BASE = 0x40052000u,
    EWM_BASE = 0x40061000u,
    IRQ_LVD = 20u,
    IRQ_LLWU = 21u,
    IRQ_WDOG_EWM = 22u,
    IRQ_FTM0 = 42u,
    IRQ_RTC = 46u,
    IRQ_RTC_SECONDS = 47u,
    IRQ_PIT0 = 48u,
    IRQ_PDB = 52u,
    IRQ_LPTMR = 58u,
    IRQ_FTM3 = 71u,
};

bool kinetis_timing_internal_contains(const KinetisTiming* timing, KinetisPeripheralId peripheral,
                                      uint32_t access_address, uint8_t access_size);
bool kinetis_timing_internal_ftm_input_capture_mode(const KinetisFtmState* ftm, uint8_t channel);
bool kinetis_timing_internal_ftm_location(const KinetisTiming* timing, uint32_t address,
                                          uint8_t* instance, uint32_t* offset);
bool kinetis_timing_internal_ftm_output_compare_mode(const KinetisFtmState* ftm, uint8_t channel);
bool kinetis_timing_internal_ftm_read(KinetisTiming* timing, uint8_t instance, uint32_t offset,
                                      uint8_t size, uint32_t* output_value);
bool kinetis_timing_internal_has(const KinetisTiming* timing, KinetisPeripheralId peripheral);
bool kinetis_timing_internal_lptmr_selected_active(const KinetisTiming* timing);
bool kinetis_timing_internal_mcg_register(uint32_t offset);
bool kinetis_timing_internal_pdb_auxiliary_offset(uint32_t offset);
bool kinetis_timing_internal_pit_read(const KinetisTiming* timing, uint32_t address, uint8_t size,
                                      uint32_t* output_value);
bool kinetis_timing_internal_pit_write(KinetisTiming* timing, uint32_t address, uint8_t size,
                                       uint32_t write_value);
bool kinetis_timing_internal_read_byte_block(const uint8_t* bytes, uint32_t base_address,
                                             uint32_t block_length, uint32_t address, uint8_t size,
                                             uint32_t* output_value);
bool kinetis_timing_internal_read_ewm(const KinetisTiming* timing, uint32_t address, uint8_t size,
                                      uint32_t* output_value);
bool kinetis_timing_internal_read_sim(const KinetisTiming* timing, uint32_t address, uint8_t size,
                                      uint32_t* output_value);
bool kinetis_timing_internal_read_wdog(const KinetisTiming* timing, uint32_t address, uint8_t size,
                                       uint32_t* output_value);
bool kinetis_timing_internal_write_ewm(KinetisTiming* timing, uint32_t address, uint8_t size,
                                       uint32_t write_value);
bool kinetis_timing_internal_write_sim(KinetisTiming* timing, uint32_t address, uint8_t size,
                                       uint32_t write_value);
bool kinetis_timing_internal_write_wdog(KinetisTiming* timing, uint32_t address, uint8_t size,
                                        uint32_t write_value);
uint32_t kinetis_timing_internal_rtc_access_reset(const KinetisTiming* timing);
uint64_t kinetis_timing_internal_clock_ticks(uint64_t* remainder, uint32_t cycles,
                                             uint32_t source_hz, uint32_t core_hz);
uint32_t kinetis_timing_internal_fixed_clock_hz(const KinetisTiming* timing);
uint32_t kinetis_timing_internal_mcgir_clock_hz(const KinetisTiming* timing);
uint32_t kinetis_timing_internal_oscer_clock_hz(const KinetisTiming* timing);
uint32_t kinetis_timing_internal_erclk32k_hz(const KinetisTiming* timing);
uint8_t kinetis_timing_internal_ftm_active_fault_mask(const KinetisFtmState* ftm);
uint8_t kinetis_timing_internal_ftm_channel_count(const KinetisTiming* timing, uint8_t instance);
uint8_t kinetis_timing_internal_ftm_fault_mode(const KinetisFtmState* ftm);
void kinetis_timing_internal_advance_ewm(KinetisTiming* timing, uint32_t cycles);
void kinetis_timing_internal_advance_ftm(KinetisTiming* timing, uint8_t instance, uint32_t cycles);
void kinetis_timing_internal_advance_lptmr(KinetisTiming* timing, uint32_t cycles);
void kinetis_timing_internal_advance_pdb(KinetisTiming* timing, uint32_t cycles);
void kinetis_timing_internal_advance_pit(KinetisTiming* timing, uint32_t cycles);
void kinetis_timing_internal_advance_rtc(KinetisTiming* timing, uint32_t cycles);
void kinetis_timing_internal_advance_wdog(KinetisTiming* timing, uint32_t cycles);
void kinetis_timing_internal_ftm_apply_software_sync(KinetisFtmState* ftm);
void kinetis_timing_internal_ftm_trigger(KinetisTiming* timing, uint8_t instance);
void kinetis_timing_internal_ftm_update_fault_status(KinetisFtmState* ftm);
void kinetis_timing_internal_request_dma(const KinetisTiming* timing, uint8_t request_source);
void kinetis_timing_internal_set_irq(const KinetisTiming* timing, uint8_t interrupt_number,
                                     bool interrupt_asserted);
void kinetis_timing_internal_signal_reset(KinetisTiming* timing, uint8_t srs0, uint8_t srs1);
void kinetis_timing_internal_trigger_adc_alternate(KinetisTiming* timing, uint8_t adc_source);
void kinetis_timing_internal_trigger_dma(const KinetisTiming* timing, uint8_t dma_channel);
void kinetis_timing_internal_trigger(KinetisTiming* timing, KinetisTimingTrigger trigger_type,
                                     uint8_t peripheral_instance, uint8_t peripheral_channel);
void kinetis_timing_internal_update_clocks(KinetisTiming* timing);
void kinetis_timing_internal_advance_pll(KinetisTiming* timing, uint32_t core_cycles);
void kinetis_timing_internal_update_ftm_irq(const KinetisTiming* timing, uint8_t instance);
void kinetis_timing_internal_update_llwu_irq(const KinetisTiming* timing);
void kinetis_timing_internal_update_pmc_irq(const KinetisTiming* timing);
void kinetis_timing_internal_update_rtc_irq(const KinetisTiming* timing);

#endif
