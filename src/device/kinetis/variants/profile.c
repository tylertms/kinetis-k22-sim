#include "device/kinetis/variants/profile.h"

#include <string.h>

#define BLOCK(peripheral, base, length) {KINETIS_PERIPHERAL_##peripheral, base, length}

static const KinetisPeripheralBlock mkv30f12810_blocks[] = {
    BLOCK(FLASH_CONFIG, 0x00000400u, 0x0eu), BLOCK(DMA, 0x40008000u, 0x1080u),
    BLOCK(FMC, 0x4001f000u, 0x300u),         BLOCK(FTFA, 0x40020000u, 0x2cu),
    BLOCK(DMAMUX, 0x40021000u, 0x04u),       BLOCK(ADC1, 0x40027000u, 0x70u),
    BLOCK(SPI0, 0x4002c000u, 0x8cu),         BLOCK(CRC, 0x40032000u, 0x0cu),
    BLOCK(PDB0, 0x40036000u, 0x19cu),        BLOCK(PIT, 0x40037000u, 0x140u),
    BLOCK(FTM0, 0x40038000u, 0x9cu),         BLOCK(FTM1, 0x40039000u, 0x9cu),
    BLOCK(FTM2, 0x4003a000u, 0x9cu),         BLOCK(ADC0, 0x4003b000u, 0x70u),
    BLOCK(DAC0, 0x4003f000u, 0x24u),         BLOCK(LPTMR0, 0x40040000u, 0x10u),
    BLOCK(SIM, 0x40047000u, 0x1064u),        BLOCK(PORTA, 0x40049000u, 0xa4u),
    BLOCK(PORTB, 0x4004a000u, 0xa4u),        BLOCK(PORTC, 0x4004b000u, 0xa4u),
    BLOCK(PORTD, 0x4004c000u, 0xccu),        BLOCK(PORTE, 0x4004d000u, 0xa4u),
    BLOCK(WDOG, 0x40052000u, 0x18u),         BLOCK(EWM, 0x40061000u, 0x06u),
    BLOCK(MCG, 0x40064000u, 0x0eu),          BLOCK(OSC, 0x40065000u, 0x03u),
    BLOCK(I2C0, 0x40066000u, 0x0cu),         BLOCK(UART0, 0x4006a000u, 0x40u),
    BLOCK(UART1, 0x4006b000u, 0x17u),        BLOCK(CMP0, 0x40073000u, 0x06u),
    BLOCK(CMP1, 0x40073008u, 0x06u),         BLOCK(VREF, 0x40074000u, 0x02u),
    BLOCK(LLWU, 0x4007c000u, 0x0au),         BLOCK(PMC, 0x4007d000u, 0x03u),
    BLOCK(SMC, 0x4007e000u, 0x04u),          BLOCK(RCM, 0x4007f000u, 0x0au),
    BLOCK(GPIOA, 0x400ff000u, 0x18u),        BLOCK(GPIOB, 0x400ff040u, 0x18u),
    BLOCK(GPIOC, 0x400ff080u, 0x18u),        BLOCK(GPIOD, 0x400ff0c0u, 0x18u),
    BLOCK(GPIOE, 0x400ff100u, 0x18u),        BLOCK(MCM, 0xe0080008u, 0x3cu),
};

static const KinetisPeripheralBlock mk22fn12810_blocks[] = {
    BLOCK(FLASH_CONFIG, 0x00000400u, 0x0eu), BLOCK(DMA, 0x40008000u, 0x1080u),
    BLOCK(FMC, 0x4001f000u, 0x300u),         BLOCK(FTFA, 0x40020000u, 0x2cu),
    BLOCK(DMAMUX, 0x40021000u, 0x04u),       BLOCK(ADC1, 0x40027000u, 0x70u),
    BLOCK(LPUART0, 0x4002a000u, 0x18u),      BLOCK(SPI0, 0x4002c000u, 0x8cu),
    BLOCK(SPI1, 0x4002d000u, 0x8cu),         BLOCK(I2S0, 0x4002f000u, 0x108u),
    BLOCK(CRC, 0x40032000u, 0x0cu),          BLOCK(PDB0, 0x40036000u, 0x19cu),
    BLOCK(PIT, 0x40037000u, 0x140u),         BLOCK(FTM0, 0x40038000u, 0x9cu),
    BLOCK(FTM1, 0x40039000u, 0x9cu),         BLOCK(FTM2, 0x4003a000u, 0x9cu),
    BLOCK(ADC0, 0x4003b000u, 0x70u),         BLOCK(RTC, 0x4003d000u, 0x808u),
    BLOCK(RFVBAT, 0x4003e000u, 0x20u),       BLOCK(DAC0, 0x4003f000u, 0x24u),
    BLOCK(LPTMR0, 0x40040000u, 0x10u),       BLOCK(RFSYS, 0x40041000u, 0x20u),
    BLOCK(SIM, 0x40047000u, 0x1064u),        BLOCK(PORTA, 0x40049000u, 0xa4u),
    BLOCK(PORTB, 0x4004a000u, 0xa4u),        BLOCK(PORTC, 0x4004b000u, 0xa4u),
    BLOCK(PORTD, 0x4004c000u, 0xccu),        BLOCK(PORTE, 0x4004d000u, 0xa4u),
    BLOCK(WDOG, 0x40052000u, 0x18u),         BLOCK(EWM, 0x40061000u, 0x06u),
    BLOCK(MCG, 0x40064000u, 0x0eu),          BLOCK(OSC, 0x40065000u, 0x03u),
    BLOCK(I2C0, 0x40066000u, 0x0cu),         BLOCK(I2C1, 0x40067000u, 0x0cu),
    BLOCK(UART0, 0x4006a000u, 0x40u),        BLOCK(UART1, 0x4006b000u, 0x17u),
    BLOCK(UART2, 0x4006c000u, 0x17u),        BLOCK(USB0, 0x40072000u, 0x15du),
    BLOCK(CMP0, 0x40073000u, 0x06u),         BLOCK(CMP1, 0x40073008u, 0x06u),
    BLOCK(VREF, 0x40074000u, 0x02u),         BLOCK(LLWU, 0x4007c000u, 0x0au),
    BLOCK(PMC, 0x4007d000u, 0x03u),          BLOCK(SMC, 0x4007e000u, 0x04u),
    BLOCK(RCM, 0x4007f000u, 0x0au),          BLOCK(GPIOA, 0x400ff000u, 0x18u),
    BLOCK(GPIOB, 0x400ff040u, 0x18u),        BLOCK(GPIOC, 0x400ff080u, 0x18u),
    BLOCK(GPIOD, 0x400ff0c0u, 0x18u),        BLOCK(GPIOE, 0x400ff100u, 0x18u),
    BLOCK(MCM, 0xe0080008u, 0x3cu),
};

static const KinetisPeripheralBlock mk22f25612_blocks[] = {
    BLOCK(FLASH_CONFIG, 0x00000400u, 0x0eu), BLOCK(DMA, 0x40008000u, 0x1200u),
    BLOCK(FMC, 0x4001f000u, 0x300u),         BLOCK(FTFA, 0x40020000u, 0x2cu),
    BLOCK(DMAMUX, 0x40021000u, 0x10u),       BLOCK(ADC1, 0x40027000u, 0x70u),
    BLOCK(RNG, 0x40029000u, 0x10u),          BLOCK(LPUART0, 0x4002a000u, 0x18u),
    BLOCK(SPI0, 0x4002c000u, 0x8cu),         BLOCK(SPI1, 0x4002d000u, 0x8cu),
    BLOCK(I2S0, 0x4002f000u, 0x108u),        BLOCK(CRC, 0x40032000u, 0x0cu),
    BLOCK(PDB0, 0x40036000u, 0x19cu),        BLOCK(PIT, 0x40037000u, 0x140u),
    BLOCK(FTM0, 0x40038000u, 0x9cu),         BLOCK(FTM1, 0x40039000u, 0x9cu),
    BLOCK(FTM2, 0x4003a000u, 0x9cu),         BLOCK(ADC0, 0x4003b000u, 0x70u),
    BLOCK(RTC, 0x4003d000u, 0x808u),         BLOCK(RFVBAT, 0x4003e000u, 0x20u),
    BLOCK(DAC0, 0x4003f000u, 0x24u),         BLOCK(LPTMR0, 0x40040000u, 0x10u),
    BLOCK(RFSYS, 0x40041000u, 0x20u),        BLOCK(SIM, 0x40047000u, 0x1064u),
    BLOCK(PORTA, 0x40049000u, 0xa4u),        BLOCK(PORTB, 0x4004a000u, 0xa4u),
    BLOCK(PORTC, 0x4004b000u, 0xa4u),        BLOCK(PORTD, 0x4004c000u, 0xccu),
    BLOCK(PORTE, 0x4004d000u, 0xa4u),        BLOCK(WDOG, 0x40052000u, 0x18u),
    BLOCK(EWM, 0x40061000u, 0x06u),          BLOCK(MCG, 0x40064000u, 0x0eu),
    BLOCK(OSC, 0x40065000u, 0x03u),          BLOCK(I2C0, 0x40066000u, 0x0cu),
    BLOCK(I2C1, 0x40067000u, 0x0cu),         BLOCK(UART0, 0x4006a000u, 0x40u),
    BLOCK(UART1, 0x4006b000u, 0x17u),        BLOCK(UART2, 0x4006c000u, 0x17u),
    BLOCK(USB0, 0x40072000u, 0x15du),        BLOCK(CMP0, 0x40073000u, 0x06u),
    BLOCK(CMP1, 0x40073008u, 0x06u),         BLOCK(VREF, 0x40074000u, 0x02u),
    BLOCK(LLWU, 0x4007c000u, 0x0au),         BLOCK(PMC, 0x4007d000u, 0x03u),
    BLOCK(SMC, 0x4007e000u, 0x04u),          BLOCK(RCM, 0x4007f000u, 0x0au),
    BLOCK(GPIOA, 0x400ff000u, 0x18u),        BLOCK(GPIOB, 0x400ff040u, 0x18u),
    BLOCK(GPIOC, 0x400ff080u, 0x18u),        BLOCK(GPIOD, 0x400ff0c0u, 0x18u),
    BLOCK(GPIOE, 0x400ff100u, 0x18u),        BLOCK(MCM, 0xe0080008u, 0x3cu),
};

static const KinetisPeripheralBlock mk22f51212_blocks[] = {
    BLOCK(FLASH_CONFIG, 0x00000400u, 0x0eu), BLOCK(DMA, 0x40008000u, 0x1200u),
    BLOCK(FB, 0x4000c000u, 0x64u),           BLOCK(FMC, 0x4001f000u, 0x300u),
    BLOCK(FTFA, 0x40020000u, 0x2cu),         BLOCK(DMAMUX, 0x40021000u, 0x10u),
    BLOCK(FTM3, 0x40026000u, 0x9cu),         BLOCK(ADC1, 0x40027000u, 0x70u),
    BLOCK(DAC1, 0x40028000u, 0x24u),         BLOCK(RNG, 0x40029000u, 0x10u),
    BLOCK(LPUART0, 0x4002a000u, 0x18u),      BLOCK(SPI0, 0x4002c000u, 0x8cu),
    BLOCK(SPI1, 0x4002d000u, 0x8cu),         BLOCK(I2S0, 0x4002f000u, 0x108u),
    BLOCK(CRC, 0x40032000u, 0x0cu),          BLOCK(PDB0, 0x40036000u, 0x19cu),
    BLOCK(PIT, 0x40037000u, 0x140u),         BLOCK(FTM0, 0x40038000u, 0x9cu),
    BLOCK(FTM1, 0x40039000u, 0x9cu),         BLOCK(FTM2, 0x4003a000u, 0x9cu),
    BLOCK(ADC0, 0x4003b000u, 0x70u),         BLOCK(RTC, 0x4003d000u, 0x808u),
    BLOCK(RFVBAT, 0x4003e000u, 0x20u),       BLOCK(DAC0, 0x4003f000u, 0x24u),
    BLOCK(LPTMR0, 0x40040000u, 0x10u),       BLOCK(RFSYS, 0x40041000u, 0x20u),
    BLOCK(SIM, 0x40047000u, 0x1064u),        BLOCK(PORTA, 0x40049000u, 0xa4u),
    BLOCK(PORTB, 0x4004a000u, 0xa4u),        BLOCK(PORTC, 0x4004b000u, 0xa4u),
    BLOCK(PORTD, 0x4004c000u, 0xccu),        BLOCK(PORTE, 0x4004d000u, 0xa4u),
    BLOCK(WDOG, 0x40052000u, 0x18u),         BLOCK(EWM, 0x40061000u, 0x06u),
    BLOCK(MCG, 0x40064000u, 0x0eu),          BLOCK(OSC, 0x40065000u, 0x03u),
    BLOCK(I2C0, 0x40066000u, 0x0cu),         BLOCK(I2C1, 0x40067000u, 0x0cu),
    BLOCK(UART0, 0x4006a000u, 0x40u),        BLOCK(UART1, 0x4006b000u, 0x17u),
    BLOCK(UART2, 0x4006c000u, 0x17u),        BLOCK(USB0, 0x40072000u, 0x15du),
    BLOCK(CMP0, 0x40073000u, 0x06u),         BLOCK(CMP1, 0x40073008u, 0x06u),
    BLOCK(VREF, 0x40074000u, 0x02u),         BLOCK(LLWU, 0x4007c000u, 0x0au),
    BLOCK(PMC, 0x4007d000u, 0x03u),          BLOCK(SMC, 0x4007e000u, 0x04u),
    BLOCK(RCM, 0x4007f000u, 0x0au),          BLOCK(GPIOA, 0x400ff000u, 0x18u),
    BLOCK(GPIOB, 0x400ff040u, 0x18u),        BLOCK(GPIOC, 0x400ff080u, 0x18u),
    BLOCK(GPIOD, 0x400ff0c0u, 0x18u),        BLOCK(GPIOE, 0x400ff100u, 0x18u),
    BLOCK(MCM, 0xe0080008u, 0x3cu),
};

static const KinetisPeripheralBlock mk22f12_blocks[] = {
    BLOCK(FLASH_CONFIG, 0x00000400u, 0x10u), BLOCK(AIPS0, 0x40000000u, 0x70u),
    BLOCK(AIPS1, 0x40080000u, 0x70u),        BLOCK(AXBS, 0x40004000u, 0xc04u),
    BLOCK(DMA, 0x40008000u, 0x1200u),        BLOCK(FB, 0x4000c000u, 0x64u),
    BLOCK(SYSMPU, 0x4000d000u, 0x830u),      BLOCK(FMC, 0x4001f000u, 0x300u),
    BLOCK(FTFE, 0x40020000u, 0x18u),         BLOCK(DMAMUX, 0x40021000u, 0x10u),
    BLOCK(CAN0, 0x40024000u, 0x8c0u),        BLOCK(SPI0, 0x4002c000u, 0x8cu),
    BLOCK(SPI1, 0x4002d000u, 0x8cu),         BLOCK(SPI2, 0x400ac000u, 0x8cu),
    BLOCK(I2S0, 0x4002f000u, 0x108u),        BLOCK(CRC, 0x40032000u, 0x0cu),
    BLOCK(USBDCD, 0x40035000u, 0x1cu),       BLOCK(PDB0, 0x40036000u, 0x1a0u),
    BLOCK(PIT, 0x40037000u, 0x140u),         BLOCK(FTM0, 0x40038000u, 0x9cu),
    BLOCK(FTM1, 0x40039000u, 0x9cu),         BLOCK(ADC0, 0x4003b000u, 0x70u),
    BLOCK(RTC, 0x4003d000u, 0x808u),         BLOCK(RFVBAT, 0x4003e000u, 0x20u),
    BLOCK(LPTMR0, 0x40040000u, 0x10u),       BLOCK(RFSYS, 0x40041000u, 0x20u),
    BLOCK(SIM, 0x40047000u, 0x1064u),        BLOCK(PORTA, 0x40049000u, 0xa4u),
    BLOCK(PORTB, 0x4004a000u, 0xa4u),        BLOCK(PORTC, 0x4004b000u, 0xa4u),
    BLOCK(PORTD, 0x4004c000u, 0xccu),        BLOCK(PORTE, 0x4004d000u, 0xa4u),
    BLOCK(WDOG, 0x40052000u, 0x18u),         BLOCK(EWM, 0x40061000u, 0x04u),
    BLOCK(CMT, 0x40062000u, 0x0cu),          BLOCK(MCG, 0x40064000u, 0x0eu),
    BLOCK(OSC, 0x40065000u, 0x01u),          BLOCK(I2C0, 0x40066000u, 0x0cu),
    BLOCK(I2C1, 0x40067000u, 0x0cu),         BLOCK(UART0, 0x4006a000u, 0x20u),
    BLOCK(UART1, 0x4006b000u, 0x17u),        BLOCK(UART2, 0x4006c000u, 0x17u),
    BLOCK(UART3, 0x4006d000u, 0x17u),        BLOCK(SDHC, 0x400b1000u, 0x1000u),
    BLOCK(USB0, 0x40072000u, 0x115u),        BLOCK(CMP0, 0x40073000u, 0x06u),
    BLOCK(CMP1, 0x40073008u, 0x06u),         BLOCK(CMP2, 0x40073010u, 0x06u),
    BLOCK(VREF, 0x40074000u, 0x02u),         BLOCK(LLWU, 0x4007c000u, 0x0bu),
    BLOCK(PMC, 0x4007d000u, 0x03u),          BLOCK(SMC, 0x4007e000u, 0x04u),
    BLOCK(RCM, 0x4007f000u, 0x08u),          BLOCK(FTM2, 0x400b8000u, 0x9cu),
    BLOCK(FTM3, 0x400b9000u, 0x9cu),         BLOCK(ADC1, 0x400bb000u, 0x70u),
    BLOCK(DAC0, 0x400cc000u, 0x24u),         BLOCK(DAC1, 0x400cd000u, 0x24u),
    BLOCK(I2C2, 0x400e6000u, 0x0cu),         BLOCK(UART4, 0x400ea000u, 0x17u),
    BLOCK(UART5, 0x400eb000u, 0x17u),        BLOCK(GPIOA, 0x400ff000u, 0x18u),
    BLOCK(GPIOB, 0x400ff040u, 0x18u),        BLOCK(GPIOC, 0x400ff080u, 0x18u),
    BLOCK(GPIOD, 0x400ff0c0u, 0x18u),        BLOCK(GPIOE, 0x400ff100u, 0x18u),
    BLOCK(MCM, 0xe0080008u, 0x2cu),
};

#define COUNT(array) (sizeof(array) / sizeof((array)[0]))
#define CPU(clock, irqs)                                                                           \
    {.architecture = KINETIS_CPU_ARCHITECTURE_ARMV7E_M,                                            \
     .core_revision_major = 0,                                                                     \
     .core_revision_minor = 1,                                                                     \
     .nvic_priority_bits = 4,                                                                      \
     .external_irq_count = irqs,                                                                   \
     .little_endian = true,                                                                        \
     .has_fpu = true,                                                                              \
     .has_mpu = false,                                                                             \
     .has_vtor = true,                                                                             \
     .has_systick = true,                                                                          \
     .maximum_core_clock_hz = clock}

static const KinetisDeviceProfile profiles[KINETIS_PROFILE_COUNT] = {
    {.id = KINETIS_PROFILE_MK22FN12810,
     .name = "MK22FN12810",
     .program_flash_size = 0x20000u,
     .program_flash_block_count = 1u,
     .sram_lower_address = 0x1fffe000u,
     .sram_lower_size = 0x2000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x4000u,
     .vlls2_sram_upper_size = 0x2000u,
     .vlls2_sram_upper_size_with_ram2 = 0x2000u,
     .sim_sdid_reset = 0x680u,
     .sim_sdid_mask = 0x000f0f80u,
     .sim_clkdiv1_reset = 0x00010000u,
     .fmc_set_count = 8u,
     .fmc_line_size = 8u,
     .cpu = CPU(100000000u, 86u),
     .peripheral_blocks = mk22fn12810_blocks,
     .peripheral_block_count = COUNT(mk22fn12810_blocks)},
    {.id = KINETIS_PROFILE_MKV30F12810,
     .name = "MKV30F12810",
     .program_flash_size = 0x20000u,
     .program_flash_block_count = 1u,
     .sram_lower_address = 0x1fffe000u,
     .sram_lower_size = 0x2000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x2000u,
     .vlls2_sram_upper_size = 0x2000u,
     .vlls2_sram_upper_size_with_ram2 = 0x2000u,
     .sim_sdid_reset = 0x306002f5u,
     .sim_sdid_mask = 0xffffffffu,
     .sim_clkdiv1_reset = 0x00010000u,
     .fmc_set_count = 8u,
     .fmc_line_size = 8u,
     .cpu = CPU(100000000u, 74u),
     .peripheral_blocks = mkv30f12810_blocks,
     .peripheral_block_count = COUNT(mkv30f12810_blocks)},
    {.id = KINETIS_PROFILE_MK22FN12812,
     .name = "MK22FN12812",
     .program_flash_size = 0x20000u,
     .program_flash_block_count = 1u,
     .sram_lower_address = 0x1fffc000u,
     .sram_lower_size = 0x4000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x8000u,
     .vlls2_sram_upper_size = 0x4000u,
     .vlls2_sram_upper_size_with_ram2 = 0x4000u,
     .sim_sdid_reset = 0xa80u,
     .sim_sdid_mask = 0x000f0f80u,
     .sim_clkdiv1_reset = 0x00010000u,
     .fmc_set_count = 8u,
     .fmc_line_size = 8u,
     .cpu = CPU(120000000u, 86u),
     .peripheral_blocks = mk22f25612_blocks,
     .peripheral_block_count = COUNT(mk22f25612_blocks)},
    {.id = KINETIS_PROFILE_MK22FN25612,
     .name = "MK22FN25612",
     .program_flash_size = 0x40000u,
     .program_flash_block_count = 1u,
     .sram_lower_address = 0x1fffc000u,
     .sram_lower_size = 0x4000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x8000u,
     .vlls2_sram_upper_size = 0x4000u,
     .vlls2_sram_upper_size_with_ram2 = 0x4000u,
     .sim_sdid_reset = 0xa80u,
     .sim_sdid_mask = 0x000f0f80u,
     .sim_clkdiv1_reset = 0x00010000u,
     .fmc_set_count = 8u,
     .fmc_line_size = 8u,
     .cpu = CPU(120000000u, 86u),
     .peripheral_blocks = mk22f25612_blocks,
     .peripheral_block_count = COUNT(mk22f25612_blocks)},
    {.id = KINETIS_PROFILE_MK22FN256CAP12,
     .name = "MK22FN256CAP12",
     .program_flash_size = 0x40000u,
     .program_flash_block_count = 1u,
     .sram_lower_address = 0x1fff0000u,
     .sram_lower_size = 0x10000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x10000u,
     .vlls2_sram_upper_size = 0x8000u,
     .vlls2_sram_upper_size_with_ram2 = 0x8000u,
     .sim_sdid_reset = 0xe80u,
     .sim_sdid_mask = 0x000f0f80u,
     .sim_clkdiv1_reset = 0x00110000u,
     .fmc_set_count = 8u,
     .fmc_line_size = 8u,
     .cpu = CPU(120000000u, 86u),
     .peripheral_blocks = mk22f51212_blocks,
     .peripheral_block_count = COUNT(mk22f51212_blocks)},
    {.id = KINETIS_PROFILE_MK22FN51212,
     .name = "MK22FN51212",
     .program_flash_size = 0x80000u,
     .program_flash_block_count = 2u,
     .sram_lower_address = 0x1fff0000u,
     .sram_lower_size = 0x10000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x10000u,
     .vlls2_sram_upper_size = 0x8000u,
     .vlls2_sram_upper_size_with_ram2 = 0x8000u,
     .sim_sdid_reset = 0xe80u,
     .sim_sdid_mask = 0x000f0f80u,
     .sim_clkdiv1_reset = 0x00110000u,
     .fmc_set_count = 8u,
     .fmc_line_size = 8u,
     .cpu = CPU(120000000u, 86u),
     .peripheral_blocks = mk22f51212_blocks,
     .peripheral_block_count = COUNT(mk22f51212_blocks)},
    {.id = KINETIS_PROFILE_MK22FN1M012,
     .name = "MK22FN1M012",
     .program_flash_size = 0x100000u,
     .program_flash_block_count = 2u,
     .sram_lower_address = 0x1fff0000u,
     .sram_lower_size = 0x10000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x10000u,
     .vlls2_sram_upper_size = 0x1000u,
     .vlls2_sram_upper_size_with_ram2 = 0x4000u,
     .flexram_address = 0x14000000u,
     .flexram_size = 0x1000u,
     .sim_sdid_reset = 0x300u,
     .sim_sdid_mask = 0xffff0f80u,
     .sim_clkdiv1_reset = 0x00110000u,
     .sim_fcfg2_has_pflsh = true,
     .fmc_set_count = 4u,
     .fmc_line_size = 16u,
     .cpu = CPU(120000000u, 82u),
     .peripheral_blocks = mk22f12_blocks,
     .peripheral_block_count = COUNT(mk22f12_blocks)},
    {.id = KINETIS_PROFILE_MK22FX51212,
     .name = "MK22FX51212",
     .program_flash_size = 0x80000u,
     .program_flash_block_count = 1u,
     .sram_lower_address = 0x1fff0000u,
     .sram_lower_size = 0x10000u,
     .sram_upper_address = 0x20000000u,
     .sram_upper_size = 0x10000u,
     .vlls2_sram_upper_size = 0x1000u,
     .vlls2_sram_upper_size_with_ram2 = 0x4000u,
     .flexnvm_address = 0x10000000u,
     .flexnvm_size = 0x20000u,
     .flexram_address = 0x14000000u,
     .flexram_size = 0x1000u,
     .sim_sdid_reset = 0x300u,
     .sim_sdid_mask = 0xffff0f80u,
     .sim_clkdiv1_reset = 0x00110000u,
     .sim_fcfg2_has_pflsh = true,
     .fmc_set_count = 4u,
     .fmc_line_size = 16u,
     .cpu = CPU(120000000u, 82u),
     .peripheral_blocks = mk22f12_blocks,
     .peripheral_block_count = COUNT(mk22f12_blocks)},
};

const KinetisDeviceProfile* kinetis_profile_get(KinetisProfile id) {
    if ((unsigned)id >= KINETIS_PROFILE_COUNT)
        return NULL;
    return &profiles[id];
}

const KinetisDeviceProfile* kinetis_profile_find(const char* profile_name) {
    if (profile_name == NULL)
        return NULL;
    for (size_t index = 0; index < COUNT(profiles); index++) {
        if (strcmp(profiles[index].name, profile_name) == 0)
            return &profiles[index];
    }
    return NULL;
}

bool kinetis_profile_peripheral_block(const KinetisDeviceProfile* profile, KinetisPeripheralId id,
                                      KinetisPeripheralBlock* block_out) {
    if (profile == NULL || (unsigned)id >= KINETIS_PERIPHERAL_COUNT)
        return false;
    for (size_t index = 0; index < profile->peripheral_block_count; index++) {
        if (profile->peripheral_blocks[index].id == id) {
            if (block_out != NULL)
                *block_out = profile->peripheral_blocks[index];
            return true;
        }
    }
    return false;
}

bool kinetis_profile_has_peripheral(const KinetisDeviceProfile* profile, KinetisPeripheralId id) {
    return kinetis_profile_peripheral_block(profile, id, NULL);
}

bool kinetis_profile_resolve_peripheral(const KinetisDeviceProfile* profile, uint32_t address,
                                        uint8_t access_size, KinetisPeripheralLocation* location) {
    if (profile == NULL || (access_size != 1 && access_size != 2 && access_size != 4))
        return false;
    for (size_t index = 0; index < profile->peripheral_block_count; index++) {
        const KinetisPeripheralBlock* block = &profile->peripheral_blocks[index];
        if (address < block->address)
            continue;
        const uint32_t peripheral_offset = address - block->address;
        if (peripheral_offset >= block->size || access_size > block->size - peripheral_offset)
            continue;
        if (location != NULL) {
            location->id = block->id;
            location->block_address = block->address;
            location->block_size = block->size;
            location->offset = peripheral_offset;
        }
        return true;
    }
    return false;
}
