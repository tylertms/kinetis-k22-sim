#ifndef KINETIS_SIM_KINETIS_H
#define KINETIS_SIM_KINETIS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cortex_m4.h"

typedef struct Kinetis Kinetis;

typedef enum {
    KINETIS_PROFILE_MK22F12810,
    KINETIS_PROFILE_MKV30F12810,
    KINETIS_PROFILE_MK22FN12812,
    KINETIS_PROFILE_MK22FN25612,
    KINETIS_PROFILE_MK22FN51212,
    KINETIS_PROFILE_MK22FN1M012,
    KINETIS_PROFILE_MK22FX51212,
    KINETIS_PROFILE_COUNT,
} KinetisProfile;

typedef enum {
    KINETIS_PACKAGE_DEFAULT = -1,
    KINETIS_PACKAGE_LH_64_LQFP,
    KINETIS_PACKAGE_MP_64_MAPBGA,
    KINETIS_PACKAGE_AH_64_WLCSP,
    KINETIS_PACKAGE_LK_80_LQFP,
    KINETIS_PACKAGE_AP_80_WLCSP,
    KINETIS_PACKAGE_BP_80_WLCSP,
    KINETIS_PACKAGE_FX_88_HVQFN,
    KINETIS_PACKAGE_LL_100_LQFP,
    KINETIS_PACKAGE_DC_121_XFBGA,
    KINETIS_PACKAGE_MC_121_MAPBGA,
    KINETIS_PACKAGE_LQ_144_LQFP,
    KINETIS_PACKAGE_MD_144_MAPBGA,
    KINETIS_PACKAGE_AK_49_WLCSP,
    KINETIS_PACKAGE_COUNT,
} KinetisPackage;

typedef struct {
    KinetisProfile profile;
    KinetisPackage package;
    size_t flash_size;
    size_t sram_size;
    uint32_t vector_table_address;
    uint32_t external_oscillator_hz;
    uint32_t rtc_oscillator_hz;
} KinetisConfiguration;

typedef enum {
    KINETIS_SERIAL_LPUART0,
    KINETIS_SERIAL_SPI0,
    KINETIS_SERIAL_SPI1,
    KINETIS_SERIAL_SPI2,
    KINETIS_SERIAL_I2C0,
    KINETIS_SERIAL_I2C1,
    KINETIS_SERIAL_I2C2,
    KINETIS_SERIAL_UART0,
    KINETIS_SERIAL_UART1,
    KINETIS_SERIAL_UART2,
    KINETIS_SERIAL_UART3,
    KINETIS_SERIAL_UART4,
    KINETIS_SERIAL_UART5,
    KINETIS_SERIAL_ENDPOINT_COUNT,
} KinetisSerialEndpoint;

typedef struct {
    uint16_t data;
    uint8_t chip_selects;
    uint8_t clock_and_transfer_attributes;
    bool continuous_chip_select;
    bool end_of_queue;
} KinetisSpiTransfer;

typedef struct {
    uint32_t identifier;
    uint8_t length;
    uint8_t data[8];
    bool extended;
    bool remote;
} KinetisCanFrame;

typedef enum {
    KINETIS_EVENT_IRQ,
    KINETIS_EVENT_DMA,
    KINETIS_EVENT_GPIO_OUTPUT,
    KINETIS_EVENT_USB_TOKEN,
    KINETIS_EVENT_CAN_TRANSMIT,
    KINETIS_EVENT_I2S_TRANSMIT,
    KINETIS_EVENT_FLEXBUS_TRANSFER,
    KINETIS_EVENT_ACCESS_ERROR,
} KinetisEventType;

typedef struct {
    KinetisEventType type;
    uint32_t source;
    uint32_t value;
    uint32_t auxiliary;
    uint8_t data[8];
    uint8_t length;
    bool extended;
    bool remote;
} KinetisEvent;

typedef enum {
    KINETIS_I2C_START,
    KINETIS_I2C_REPEATED_START,
    KINETIS_I2C_WRITE,
    KINETIS_I2C_READ,
    KINETIS_I2C_STOP,
} KinetisI2cTransferType;

typedef enum {
    KINETIS_USB_CHARGER_NONE,
    KINETIS_USB_CHARGER_STANDARD_HOST,
    KINETIS_USB_CHARGER_CHARGING_PORT,
    KINETIS_USB_CHARGER_DEDICATED,
    KINETIS_USB_CHARGER_ERROR,
} KinetisUsbCharger;

typedef struct {
    KinetisI2cTransferType type;
    uint8_t value;
} KinetisI2cTransfer;

KinetisConfiguration kinetis_default_configuration(void);
KinetisConfiguration kinetis_configuration(KinetisProfile profile);
bool kinetis_profile_from_name(const char* name, KinetisProfile* profile);
const char* kinetis_profile_name(KinetisProfile profile);
bool kinetis_package_from_code(const char* code, KinetisPackage* package);
const char* kinetis_package_code(KinetisPackage package);
Kinetis* kinetis_create(KinetisConfiguration configuration);
void kinetis_destroy(Kinetis* device);
CortexM4* kinetis_cpu(Kinetis* device);
const CortexM4* kinetis_cpu_const(const Kinetis* device);
bool kinetis_reset(Kinetis* device);
bool kinetis_load(Kinetis* device, uint32_t address, const void* input_data, size_t data_size);
bool kinetis_seed(Kinetis* device, uint32_t address, const void* input_data, size_t data_size);
bool kinetis_read(const Kinetis* device, uint32_t address, void* output_data, size_t data_size);
bool kinetis_write(Kinetis* device, uint32_t address, const void* input_data, size_t data_size);
bool kinetis_copy(Kinetis* destination, const Kinetis* source);
uint64_t kinetis_get_uninitialized_sram_read_count(const Kinetis* device);
uint32_t kinetis_get_first_uninitialized_sram_read(const Kinetis* device);
void kinetis_clear_uninitialized_sram_reads(Kinetis* device);
bool kinetis_register_state_equal(Kinetis* first, Kinetis* second, uint32_t* first_difference,
                                  uint32_t* first_value, uint32_t* second_value);
void kinetis_advance(Kinetis* device, uint32_t cycle_count);
void kinetis_watchdog_advance(Kinetis* device, uint32_t ticks);
uint32_t kinetis_core_clock_hz(const Kinetis* device);
uint32_t kinetis_bus_clock_hz(const Kinetis* device);
bool kinetis_next_event(Kinetis* device, KinetisEvent* event);
bool kinetis_set_adc_channel(Kinetis* device, uint8_t instance, uint8_t channel,
                             uint16_t sample_value);
void kinetis_set_adc0_channel(Kinetis* device, uint8_t channel, uint16_t sample_value);
bool kinetis_set_cmp_input(Kinetis* device, uint8_t instance, uint8_t input_index,
                           uint8_t input_level);
bool kinetis_set_lptmr_input(Kinetis* device, uint8_t input_index, bool input_high);
bool kinetis_trigger_low_voltage_warning(Kinetis* device);
bool kinetis_trigger_low_voltage_detect(Kinetis* device);
bool kinetis_set_llwu_pin(Kinetis* device, uint8_t pin, bool pin_high);
bool kinetis_trigger_llwu_module(Kinetis* device, uint8_t module);
bool kinetis_set_ewm_input(Kinetis* device, bool input_high);
bool kinetis_ewm_output(const Kinetis* device);
bool kinetis_get_cmt_output(const Kinetis* device, bool* is_driven, bool* is_high);
bool kinetis_set_ftm_input(Kinetis* device, uint8_t instance, uint8_t channel, bool input_high);
bool kinetis_set_ftm_fault(Kinetis* device, uint8_t instance, uint8_t input_index, bool input_high);
bool kinetis_trigger_ftm_hardware(Kinetis* device, uint8_t instance, uint8_t trigger);
bool kinetis_get_ftm_output(const Kinetis* device, uint8_t instance, uint8_t channel,
                            bool* output_high);
bool kinetis_get_dac_output(const Kinetis* device, uint8_t instance, uint16_t* output_value);
void kinetis_rng_seed(Kinetis* device, uint32_t seed_value);
bool kinetis_gpio_drive(Kinetis* device, uint8_t port, uint8_t pin, bool pin_high);
bool kinetis_gpio_release(Kinetis* device, uint8_t port, uint8_t pin);
bool kinetis_gpio_pin(const Kinetis* device, uint8_t port, uint8_t pin, bool* pin_high);
bool kinetis_serial_receive(Kinetis* device, KinetisSerialEndpoint endpoint,
                            uint16_t received_value, uint8_t status);
bool kinetis_serial_transmit(Kinetis* device, KinetisSerialEndpoint endpoint,
                             uint16_t* output_value);
bool kinetis_spi_transfer(Kinetis* device, KinetisSerialEndpoint endpoint,
                          KinetisSpiTransfer* transfer);
bool kinetis_i2c_transfer(Kinetis* device, KinetisSerialEndpoint endpoint,
                          KinetisI2cTransfer* transfer);
bool kinetis_i2c_acknowledge(Kinetis* device, KinetisSerialEndpoint endpoint, bool acknowledge);
bool kinetis_i2c_detect_start(Kinetis* device, KinetisSerialEndpoint endpoint);
bool kinetis_i2c_detect_stop(Kinetis* device, KinetisSerialEndpoint endpoint);
bool kinetis_i2c_lose_arbitration(Kinetis* device, KinetisSerialEndpoint endpoint);
bool kinetis_i2c_receive(Kinetis* device, KinetisSerialEndpoint endpoint, uint8_t value);
bool kinetis_usb_token(Kinetis* device, uint8_t endpoint, uint8_t token, bool transmit);
bool kinetis_can_receive(Kinetis* device, const KinetisCanFrame* frame);
bool kinetis_i2s_receive(Kinetis* device, uint32_t sample);
bool kinetis_i2s_transmit(Kinetis* device, uint32_t* sample);
bool kinetis_sdhc_insert(Kinetis* device, const void* card_data, size_t card_size,
                         bool write_protected);
void kinetis_sdhc_eject(Kinetis* device);
bool kinetis_sdhc_read_card(const Kinetis* device, size_t card_offset, void* card_data,
                            size_t byte_count);
bool kinetis_flexbus_attach(Kinetis* device, uint32_t address, const void* input_data,
                            size_t window_size, bool read_only);
void kinetis_flexbus_detach(Kinetis* device);
bool kinetis_flexbus_read(const Kinetis* device, size_t window_offset, void* output_data,
                          size_t read_size);
bool kinetis_set_usb_charger(Kinetis* device, KinetisUsbCharger charger);
bool kinetis_set_usb_pullup(Kinetis* device, bool enabled);
bool kinetis_set_reset_state(Kinetis* device, uint8_t srs0, bool ackiso);
bool kinetis_uart1_receive(Kinetis* device, uint8_t value, uint8_t status);
bool kinetis_uart1_error(Kinetis* device, uint8_t status);
bool kinetis_uart1_transmit(Kinetis* device, uint8_t* output_value);
bool kinetis_spi0_receive(Kinetis* device, uint16_t value);
bool kinetis_spi0_transmit(Kinetis* device, uint16_t* output_value);
bool kinetis_i2c0_transfer(Kinetis* device, KinetisI2cTransfer* transfer);
void kinetis_i2c0_acknowledge(Kinetis* device, bool acknowledge);
bool kinetis_i2c0_lose_arbitration(Kinetis* device);
bool kinetis_i2c0_receive(Kinetis* device, uint8_t received_value);

#endif
