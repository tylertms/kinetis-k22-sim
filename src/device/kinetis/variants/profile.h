#ifndef KINETIS_SIM_PROFILE_H
#define KINETIS_SIM_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kinetis.h"

typedef enum {
    KINETIS_PERIPHERAL_FLASH_CONFIG,
    KINETIS_PERIPHERAL_AIPS0,
    KINETIS_PERIPHERAL_AIPS1,
    KINETIS_PERIPHERAL_AXBS,
    KINETIS_PERIPHERAL_DMA,
    KINETIS_PERIPHERAL_FB,
    KINETIS_PERIPHERAL_SYSMPU,
    KINETIS_PERIPHERAL_FMC,
    KINETIS_PERIPHERAL_FTFA,
    KINETIS_PERIPHERAL_FTFE,
    KINETIS_PERIPHERAL_DMAMUX,
    KINETIS_PERIPHERAL_CAN0,
    KINETIS_PERIPHERAL_FTM0,
    KINETIS_PERIPHERAL_FTM1,
    KINETIS_PERIPHERAL_FTM2,
    KINETIS_PERIPHERAL_FTM3,
    KINETIS_PERIPHERAL_ADC0,
    KINETIS_PERIPHERAL_ADC1,
    KINETIS_PERIPHERAL_DAC0,
    KINETIS_PERIPHERAL_DAC1,
    KINETIS_PERIPHERAL_RNG,
    KINETIS_PERIPHERAL_LPUART0,
    KINETIS_PERIPHERAL_SPI0,
    KINETIS_PERIPHERAL_SPI1,
    KINETIS_PERIPHERAL_SPI2,
    KINETIS_PERIPHERAL_SDHC,
    KINETIS_PERIPHERAL_I2S0,
    KINETIS_PERIPHERAL_CRC,
    KINETIS_PERIPHERAL_USBDCD,
    KINETIS_PERIPHERAL_PDB0,
    KINETIS_PERIPHERAL_PIT,
    KINETIS_PERIPHERAL_RTC,
    KINETIS_PERIPHERAL_RFVBAT,
    KINETIS_PERIPHERAL_LPTMR0,
    KINETIS_PERIPHERAL_RFSYS,
    KINETIS_PERIPHERAL_SIM,
    KINETIS_PERIPHERAL_PORTA,
    KINETIS_PERIPHERAL_PORTB,
    KINETIS_PERIPHERAL_PORTC,
    KINETIS_PERIPHERAL_PORTD,
    KINETIS_PERIPHERAL_PORTE,
    KINETIS_PERIPHERAL_WDOG,
    KINETIS_PERIPHERAL_EWM,
    KINETIS_PERIPHERAL_CMT,
    KINETIS_PERIPHERAL_MCG,
    KINETIS_PERIPHERAL_OSC,
    KINETIS_PERIPHERAL_I2C0,
    KINETIS_PERIPHERAL_I2C1,
    KINETIS_PERIPHERAL_I2C2,
    KINETIS_PERIPHERAL_UART0,
    KINETIS_PERIPHERAL_UART1,
    KINETIS_PERIPHERAL_UART2,
    KINETIS_PERIPHERAL_UART3,
    KINETIS_PERIPHERAL_UART4,
    KINETIS_PERIPHERAL_UART5,
    KINETIS_PERIPHERAL_USB0,
    KINETIS_PERIPHERAL_CMP0,
    KINETIS_PERIPHERAL_CMP1,
    KINETIS_PERIPHERAL_CMP2,
    KINETIS_PERIPHERAL_VREF,
    KINETIS_PERIPHERAL_LLWU,
    KINETIS_PERIPHERAL_PMC,
    KINETIS_PERIPHERAL_SMC,
    KINETIS_PERIPHERAL_RCM,
    KINETIS_PERIPHERAL_GPIOA,
    KINETIS_PERIPHERAL_GPIOB,
    KINETIS_PERIPHERAL_GPIOC,
    KINETIS_PERIPHERAL_GPIOD,
    KINETIS_PERIPHERAL_GPIOE,
    KINETIS_PERIPHERAL_MCM,
    KINETIS_PERIPHERAL_COUNT,
} KinetisPeripheralId;

typedef enum {
    KINETIS_CPU_ARCHITECTURE_ARMV7E_M,
} KinetisCpuArchitecture;

typedef struct {
    KinetisCpuArchitecture architecture;
    uint8_t core_revision_major;
    uint8_t core_revision_minor;
    uint8_t nvic_priority_bits;
    uint16_t external_irq_count;
    bool little_endian;
    bool has_fpu;
    bool has_mpu;
    bool has_vtor;
    bool has_systick;
    uint32_t maximum_core_clock_hz;
} KinetisCpuOptions;

typedef struct {
    KinetisPeripheralId id;
    uint32_t address;
    uint32_t size;
} KinetisPeripheralBlock;

typedef struct {
    KinetisProfile id;
    const char* name;
    uint32_t program_flash_size;
    uint8_t program_flash_block_count;
    uint32_t sram_lower_address;
    uint32_t sram_lower_size;
    uint32_t sram_upper_address;
    uint32_t sram_upper_size;
    uint32_t vlls2_sram_upper_size;
    uint32_t vlls2_sram_upper_size_with_ram2;
    uint32_t flexnvm_address;
    uint32_t flexnvm_size;
    uint32_t flexram_address;
    uint32_t flexram_size;
    uint32_t sim_sdid_reset;
    uint32_t sim_sdid_mask;
    uint32_t sim_clkdiv1_reset;
    bool sim_fcfg2_has_pflsh;
    uint8_t fmc_set_count;
    uint8_t fmc_line_size;
    KinetisCpuOptions cpu;
    const KinetisPeripheralBlock* peripheral_blocks;
    size_t peripheral_block_count;
} KinetisDeviceProfile;

typedef struct {
    KinetisPeripheralId id;
    uint32_t block_address;
    uint32_t block_size;
    uint32_t offset;
} KinetisPeripheralLocation;

const KinetisDeviceProfile* kinetis_profile_get(KinetisProfile id);
const KinetisDeviceProfile* kinetis_profile_find(const char* profile_name);
bool kinetis_profile_has_peripheral(const KinetisDeviceProfile* profile, KinetisPeripheralId id);
bool kinetis_profile_peripheral_block(const KinetisDeviceProfile* profile, KinetisPeripheralId id,
                                      KinetisPeripheralBlock* block_out);
bool kinetis_profile_resolve_peripheral(const KinetisDeviceProfile* profile, uint32_t address,
                                        uint8_t access_size, KinetisPeripheralLocation* location);

#endif
