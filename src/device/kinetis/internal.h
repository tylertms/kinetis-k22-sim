#ifndef CORTEX_M4_KINETIS_INTERNAL_H
#define CORTEX_M4_KINETIS_INTERNAL_H

#include "architecture/cortex_m4/internal.h"
#include "device/kinetis/communication/sdhc.h"
#include "device/kinetis/communication/serial/api.h"
#include "device/kinetis/communication/usb_dcd.h"
#include "device/kinetis/gpio/io.h"
#include "device/kinetis/memory/api.h"
#include "device/kinetis/timing/api.h"
#include "device/kinetis/variants/manifest.h"
#include "device/kinetis/variants/package.h"
#include "device/kinetis/variants/profile.h"
#include "kinetis.h"

#include <string.h>

enum {
    KINETIS_FLASH_BASE = 0x00000000u,
    KINETIS_SRAM_CENTER = 0x20000000u,
    KINETIS_PERIPHERAL_BASE = 0x40000000u,
    KINETIS_PERIPHERAL_SIZE = 0x00100000u,
    KINETIS_PRIVATE_PERIPHERAL_BASE = 0xf0000000u,
    KINETIS_PRIVATE_PERIPHERAL_SIZE = 0x00005000u,
    KINETIS_BIT_BAND_BASE = 0x42000000u,
    KINETIS_BIT_BAND_SIZE = 0x02000000u,
    KINETIS_EVENT_CAPACITY = 64,
    KINETIS_SIM_SCGC1 = 0x40048028u,
    KINETIS_SIM_SCGC2 = 0x4004802cu,
    KINETIS_AIPS0 = 0x40000000u,
    KINETIS_AIPS1 = 0x40080000u,
    KINETIS_AXBS = 0x40004000u,
    KINETIS_FMC = 0x4001f000u,
    KINETIS_USBDCD = 0x40035000u,
    KINETIS_CMT = 0x40062000u,
    KINETIS_FMC_WAY_COUNT = 4,
    KINETIS_FMC_MAX_SET_COUNT = 8,
};

struct Kinetis {
    KinetisConfiguration configuration;
    const KinetisDeviceProfile* profile;
    const KinetisPackageSelection* package;
    const KinetisRegisterManifest* manifest;
    CortexM4* cpu;
    uint8_t* flash;
    uint8_t* sram;
    uint8_t* sram_initialized;
    uint8_t* peripheral;
    uint8_t private_peripheral[KINETIS_PRIVATE_PERIPHERAL_SIZE];
    uint8_t* flexbus_memory;
    uint32_t flexbus_address;
    size_t flexbus_size;
    bool flexbus_read_only;
    bool reset_pin_enabled;
    uint32_t sram_base;
    uint64_t uninitialized_sram_read_count;
    uint32_t first_uninitialized_sram_read;
    uint64_t cycles;
    uint64_t cmt_cycles;
    uint64_t cmt_bus_remainder;
    uint64_t cmt_mark_ticks;
    uint64_t cmt_period_ticks;
    uint64_t cmt_carrier_high_ticks;
    uint64_t cmt_carrier_period_ticks;
    uint64_t cmt_carrier_offset_ticks;
    uint64_t cmt_output_delay_ticks;
    bool cmt_eoc_read;
    bool cmt_running;
    bool cmt_stop_pending;
    bool cmt_fsk_secondary;
    bool cmt_extended_space;
    bool cmt_dma_pending;
    uint8_t fmc_bank[KINETIS_FMC_WAY_COUNT][KINETIS_FMC_MAX_SET_COUNT];
    uint64_t fmc_age[KINETIS_FMC_WAY_COUNT][KINETIS_FMC_MAX_SET_COUNT];
    uint64_t fmc_access_count;
    uint32_t mmdvsq_pending_result;
    uint32_t mmdvsq_pending_control;
    uint8_t mmdvsq_cycles_remaining;
    uint8_t mmdvsq_stall_cycles;
    uint32_t mtb_previous_address;
    uint32_t mtb_previous_opcode;
    uint32_t mtb_previous_lr;
    uint16_t mtb_previous_exception;
    bool mtb_previous_valid;
    bool mtb_first_packet;
    bool mtb_autostop_latched;
    bool mtb_stop_pending;
    bool cpo_entry_pending;
    KinetisUsbDcd usbdcd;
    KinetisData* data;
    KinetisIo io;
    KinetisSdhc sdhc;
    KinetisSerial serial;
    KinetisTiming timing;
    bool comparator_output[3];
    bool data_interrupt[KINETIS_DATA_INTERRUPT_COUNT];
    KinetisEvent events[KINETIS_EVENT_CAPACITY];
    uint8_t event_read_index;
    uint8_t event_write_index;
    uint8_t event_count;
};

bool kinetis_memory_read(Kinetis* device, uint32_t address, uint8_t access_size,
                         CortexM4Access access, uint32_t* output_value);
bool kinetis_memory_write(Kinetis* device, uint32_t address, uint8_t access_size,
                          CortexM4Access access, uint32_t write_value);
bool kinetis_dma_read(Kinetis* device, uint32_t address, uint8_t access_size,
                      uint32_t* output_value);
bool kinetis_dma_write(Kinetis* device, uint32_t address, uint8_t access_size,
                       uint32_t write_value);
bool kinetis_flash_controller_read(Kinetis* device, uint32_t address, uint8_t access_size,
                                   uint32_t* output_value);
bool kinetis_flash_controller_write(Kinetis* device, uint32_t address, uint8_t access_size,
                                    uint32_t write_value);
bool kinetis_peripheral_read(Kinetis* device, uint32_t address, uint8_t access_size,
                             CortexM4Access access, uint32_t* output_value);
bool kinetis_peripheral_write(Kinetis* device, uint32_t address, uint8_t access_size,
                              CortexM4Access access, uint32_t write_value);
void kinetis_peripheral_advance(Kinetis* device, uint32_t cycle_count);
void kinetis_peripheral_reset(Kinetis* device);
void kinetis_warm_reset(Kinetis* device, uint8_t reset_cause_0, uint8_t reset_cause_1);
void kinetis_refresh_signals(Kinetis* device);
void kinetis_sync_clock_gates(Kinetis* device);
KinetisDataBus kinetis_data_bus(Kinetis* device);
KinetisSdhcBus kinetis_sdhc_bus(Kinetis* device);
KinetisTimingSignals kinetis_timing_signals(Kinetis* device);
KinetisIoConfiguration kinetis_io_configuration(Kinetis* device);

bool kinetis_internal_aips_access_allowed(const Kinetis* device, uint32_t address,
                                          CortexM4Access access, bool write_access);
bool kinetis_internal_axbs_write_allowed(const Kinetis* device, uint32_t address);
bool kinetis_internal_enable_debug_clock(Kinetis* device, KinetisPeripheralId id);
bool kinetis_internal_manifest_extension(KinetisPeripheralId id);
bool kinetis_internal_peripheral_clock_enabled(const Kinetis* device, KinetisPeripheralId id);
bool kinetis_internal_pop_serial_event(KinetisSerial* serial, KinetisSerialEndpoint endpoint,
                                       KinetisSerialEvent* event);
bool kinetis_internal_semantic_read(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                                    uint8_t byte_count, uint32_t* output_value);
bool kinetis_internal_semantic_write(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                                     uint8_t byte_count, uint32_t write_value);
bool kinetis_internal_serial_endpoint_available(const Kinetis* device,
                                                KinetisSerialEndpoint endpoint);
const KinetisRegisterDescriptor*
kinetis_internal_manifest_descriptor_for_access(const Kinetis* device, uint32_t address,
                                                uint8_t byte_count);
KinetisPeripheralId kinetis_internal_serial_endpoint_peripheral(KinetisSerialEndpoint endpoint);
bool kinetis_internal_data_bus_read(void* context, uint32_t address, uint8_t byte_count,
                                    uint32_t* output_value);
bool kinetis_internal_flash_bus_read(void* context, uint32_t address, uint8_t byte_count,
                                     uint32_t* output_value);
bool kinetis_internal_data_bus_write(void* context, uint32_t address, uint8_t byte_count,
                                     uint32_t write_value);
bool kinetis_internal_flash_bus_write(void* context, uint32_t address, uint8_t byte_count,
                                      uint32_t write_value);
void kinetis_internal_data_dma_complete(void* context, uint8_t channel, uint8_t request_source);
void kinetis_internal_data_adc_complete(void* context, uint8_t instance, uint8_t pretrigger);
void kinetis_internal_data_interrupt(void* context, KinetisDataInterrupt interrupt, bool asserted);
void kinetis_internal_io_event(void* context, const KinetisIoEvent* event);
void kinetis_internal_timing_dma_trigger(void* context, uint8_t channel);
void kinetis_internal_timing_dma(void* context, uint8_t request_source);
void kinetis_internal_timing_pdb_pulse(void* context, uint8_t instance, uint8_t output,
                                       bool asserted);
void kinetis_internal_timing_irq(void* context, uint8_t irq, bool asserted);
void kinetis_internal_timing_reset(void* context, uint8_t reset_cause_0, uint8_t reset_cause_1);
void kinetis_internal_timing_trigger(void* context, KinetisTimingTrigger type, uint8_t instance,
                                     uint8_t channel, uint8_t source_instance);
uint32_t kinetis_internal_manifest_access_mask(const KinetisRegisterDescriptor* descriptor,
                                               uint32_t address, uint32_t mask);
uint32_t kinetis_internal_raw_load(const Kinetis* device, uint32_t address, uint8_t byte_count);
uint32_t kinetis_internal_width_mask(uint8_t byte_count);
void kinetis_internal_cmt_advance(Kinetis* device, uint32_t cycle_count);
void kinetis_internal_mmdvsq_advance(Kinetis* device, uint32_t cycle_count);
void kinetis_internal_mtb_trace(void* context, uint32_t address, uint32_t opcode, bool executed);
void kinetis_internal_exception_vector(void* context);
void kinetis_internal_mtbdwt_data_access(Kinetis* device, uint32_t address, uint8_t byte_count,
                                         bool write, uint32_t value);
bool kinetis_internal_mtb_read(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                               uint8_t byte_count, uint32_t* output_value);
bool kinetis_internal_mtb_write(Kinetis* device, KinetisPeripheralId id, uint32_t address,
                                uint8_t byte_count, uint32_t write_value);
bool kinetis_internal_mtb_unprivileged_ram_blocked(const Kinetis* device, uint32_t address,
                                                   uint8_t byte_count);
uint32_t kinetis_internal_wait_states(void* context, uint32_t address, uint8_t byte_count,
                                      CortexM4Access access, bool write, bool sequential);
void kinetis_internal_raw_store(Kinetis* device, uint32_t address, uint8_t byte_count,
                                uint32_t write_value);
void kinetis_internal_refresh_serial_signals(Kinetis* device);

#endif
