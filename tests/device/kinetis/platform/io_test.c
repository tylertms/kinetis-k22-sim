#include "device/kinetis/gpio/io.h"

#include <stdint.h>
#include <string.h>

#include "test.h"

enum {
    PORTA = 0x40049000u,
    GPIOA = 0x400ff000u,
    USB0 = 0x40072000u,
    CAN0 = 0x40024000u,
    I2S0 = 0x4002f000u,
    FLEXBUS = 0x4000c000u,
    SYSMPU = 0x4000d000u,
};

#define MCM UINT32_C(0xe0080008)

void kinetis_io_test_peripheral_boundaries(TestState* state);

typedef struct {
    KinetisIoEvent events[128];
    size_t event_count;
} EventLog;

static void record_event(void* context, const KinetisIoEvent* event) {
    EventLog* log = context;
    if (log->event_count < sizeof(log->events) / sizeof(log->events[0])) {
        log->events[log->event_count++] = *event;
    }
}

static uint32_t read_value(TestState* state, KinetisIo* io, uint32_t address, uint8_t access_size) {
    uint32_t output_value = 0u;
    expect(state, kinetis_io_read(io, address, access_size, &output_value),
           "kinetis_io_read(io, address, access_size, &output_value)");
    return output_value;
}

static void write_value(TestState* state, KinetisIo* io, uint32_t address, uint8_t access_size,
                        uint32_t write_data) {
    expect(state, kinetis_io_write(io, address, access_size, write_data),
           "kinetis_io_write(io, address, access_size, write_data)");
}

static uint32_t bit_band_address(uint32_t address, uint8_t bit_index) {
    return 0x42000000u + (address - 0x40000000u) * 32u + (uint32_t)bit_index * 4u;
}

static bool has_event(const EventLog* log, KinetisIoEventType type, uint32_t source) {
    for (size_t event_index = 0u; event_index < log->event_count; event_index++) {
        if (log->events[event_index].type == type && log->events[event_index].source == source) {
            return true;
        }
    }
    return false;
}

static const KinetisIoEvent* find_event(const EventLog* log, KinetisIoEventType type,
                                        uint32_t source) {
    for (size_t event_index = log->event_count; event_index > 0u; event_index--) {
        if (log->events[event_index - 1u].type == type &&
            log->events[event_index - 1u].source == source) {
            return &log->events[event_index - 1u];
        }
    }
    return NULL;
}

static void test_reset_clock_and_configuration(TestState* state) {
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN51212);
    EventLog log = {0};
    KinetisIoConfiguration configuration = kinetis_io_default_configuration(profile);
    configuration.package_pin_mask[0] = 0x0fu;
    configuration.flash_configuration[0] = 0x12u;
    configuration.flash_configuration[1] = 0x34u;
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    expect(state, !kinetis_io_init(NULL, configuration), "!kinetis_io_init(NULL, configuration)");
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");

    expect(state, read_value(state, &io, 0x400u, 2) == 0x3412u,
           "read_value(state, &io, 0x400u, 2) == 0x3412u");
    expect(state, !kinetis_io_write(&io, 0x400u, 1, 0), "!kinetis_io_write(&io, 0x400u, 1, 0)");

    uint32_t read_data = 0u;
    expect(state, !kinetis_io_read(&io, PORTA, 4, &read_data),
           "!kinetis_io_read(&io, PORTA, 4, &read_data)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_ACCESS_ERROR, PORTA),
           "has_event(&log, KINETIS_IO_EVENT_ACCESS_ERROR, PORTA)");

    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_PORTA, true);
    expect(state, kinetis_io_clock_enabled(&io, KINETIS_PERIPHERAL_PORTA),
           "kinetis_io_clock_enabled(&io, KINETIS_PERIPHERAL_PORTA)");
    expect(state, kinetis_io_clock_enabled(&io, KINETIS_PERIPHERAL_GPIOA),
           "kinetis_io_clock_enabled(&io, KINETIS_PERIPHERAL_GPIOA)");
    expect(state, read_value(state, &io, PORTA, 4) == 0x702u,
           "read_value(state, &io, PORTA, 4) == 0x702u");

    expect(state, !kinetis_io_read(&io, PORTA + 16u, 4, &read_data),
           "!kinetis_io_read(&io, PORTA + 16u, 4, &read_data)");
    expect(state, read_value(state, &io, MCM, 2) == 0x1fu,
           "read_value(state, &io, MCM, 2) == 0x1fu");
    expect(state, read_value(state, &io, MCM + 2u, 2) == 0x17u,
           "read_value(state, &io, MCM + 2u, 2) == 0x17u");
    expect(state, !kinetis_io_write(&io, MCM, 2, 0), "!kinetis_io_write(&io, MCM, 2, 0)");
    write_value(state, &io, MCM + 4u, 4, 1u);
    expect(state, read_value(state, &io, MCM + 4u, 4) == 1u,
           "read_value(state, &io, MCM + 4u, 4) == 1u");
    expect(state, !kinetis_io_read(&io, 0x40012340u, 4, &read_data),
           "!kinetis_io_read(&io, 0x40012340u, 4, &read_data)");
}

static void test_kv30_mcm_reset(TestState* state) {
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MKV30F12810));
    KinetisIo io;

    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    expect(state, read_value(state, &io, MCM, 2) == 0x0fu,
           "KV30 PLASC reports four crossbar slave ports");
    expect(state, read_value(state, &io, MCM + 2u, 2) == 0x17u,
           "KV30 PLAMC reports the documented master ports");
    expect(state, read_value(state, &io, MCM, 4) == 0x0017000fu,
           "KV30 combined MCM topology read matches its register reset values");
}

static void test_gpio_mux_pull_open_drain_and_lock(TestState* state) {
    EventLog log = {0};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN51212));
    configuration.package_pin_mask[0] = 0x0fu;
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_GPIOA, true);
    write_value(state, &io, PORTA, 4, 1u << 8);
    write_value(state, &io, GPIOA + 0x14u, 4, 1u);
    write_value(state, &io, GPIOA + 4u, 4, 1u);
    expect(state, read_value(state, &io, GPIOA, 4) == 1u, "read_value(state, &io, GPIOA, 4) == 1u");
    expect(state, (read_value(state, &io, GPIOA + 0x10u, 4) & 1u) != 0u,
           "(read_value(state, &io, GPIOA + 0x10u, 4) & 1u) != 0u");
    expect(state, has_event(&log, KINETIS_IO_EVENT_GPIO_OUTPUT, 0),
           "has_event(&log, KINETIS_IO_EVENT_GPIO_OUTPUT, 0)");
    write_value(state, &io, PORTA, 4, (1u << 8) | (1u << 5) | 3u);
    expect(state, (read_value(state, &io, GPIOA + 0x10u, 4) & 1u) != 0u,
           "(read_value(state, &io, GPIOA + 0x10u, 4) & 1u) != 0u");
    expect(state, kinetis_io_drive_pin(&io, 0, 0, false), "kinetis_io_drive_pin(&io, 0, 0, false)");
    expect(state, (read_value(state, &io, GPIOA + 0x10u, 4) & 1u) == 0,
           "(read_value(state, &io, GPIOA + 0x10u, 4) & 1u) == 0");
    expect(state, kinetis_io_release_pin(&io, 0, 0), "kinetis_io_release_pin(&io, 0, 0)");
    expect(state, (read_value(state, &io, GPIOA + 0x10u, 4) & 1u) != 0,
           "(read_value(state, &io, GPIOA + 0x10u, 4) & 1u) != 0");
    write_value(state, &io, PORTA + 4u, 4, (1u << 15) | (1u << 8));
    write_value(state, &io, PORTA + 4u, 4, 0);
    expect(state, (read_value(state, &io, PORTA + 4u, 4) & (1u << 15)) != 0,
           "(read_value(state, &io, PORTA + 4u, 4) & (1u << 15)) != 0");
    write_value(state, &io, PORTA + 0x80u, 4, (4u << 16) | (1u << 8));
    expect(state, (read_value(state, &io, PORTA + 8u, 4) & (7u << 8)) == (1u << 8),
           "(read_value(state, &io, PORTA + 8u, 4) & (7u << 8)) == (1u << 8)");
    write_value(state, &io, bit_band_address(GPIOA + 0x14u, 2), 4, 1u);
    write_value(state, &io, bit_band_address(GPIOA, 2), 4, 1u);
    expect(state, read_value(state, &io, bit_band_address(GPIOA, 2), 4) == 1u,
           "read_value(state, &io, bit_band_address(GPIOA, 2), 4) == 1u");
    expect(state, (read_value(state, &io, GPIOA, 4) & 4u) != 0,
           "(read_value(state, &io, GPIOA, 4) & 4u) != 0");
    uint32_t invalid_alias_value = 0u;
    const uint32_t invalid_alias = bit_band_address(0x400fffffu, 0u);
    expect(state, !kinetis_io_read(&io, invalid_alias, 4u, &invalid_alias_value),
           "bit-band read rejects an unmapped source byte");
    expect(state, !kinetis_io_write(&io, invalid_alias, 4u, 1u),
           "bit-band write rejects an unmapped source byte");
    expect(state, !kinetis_io_drive_pin(&io, 0, 7, true), "!kinetis_io_drive_pin(&io, 0, 7, true)");
    expect(state, !kinetis_io_release_pin(&io, 6, 0), "!kinetis_io_release_pin(&io, 6, 0)");
}

static void test_gpio_interrupt_dma_filter_and_bit_band(TestState* state) {
    EventLog log = {0};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN51212));
    configuration.package_pin_mask[3] = 3u;
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_PORTD, true);
    write_value(state, &io, 0x4004c000u, 4, 9u << 16);
    expect(state, kinetis_io_drive_pin(&io, 3, 0, false), "kinetis_io_drive_pin(&io, 3, 0, false)");
    expect(state, kinetis_io_drive_pin(&io, 3, 0, true), "kinetis_io_drive_pin(&io, 3, 0, true)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_IRQ, 62u),
           "has_event(&log, KINETIS_IO_EVENT_IRQ, 62u)");
    expect(state, kinetis_io_irq_asserted(&io, 62u), "kinetis_io_irq_asserted(&io, 62u)");
    expect(state, (read_value(state, &io, 0x4004c0a0u, 4) & 1u) != 0,
           "(read_value(state, &io, 0x4004c0a0u, 4) & 1u) != 0");
    write_value(state, &io, 0x4004c000u, 1, 3u);
    expect(state, (read_value(state, &io, 0x4004c000u, 4) & (1u << 24)) != 0,
           "(read_value(state, &io, 0x4004c000u, 4) & (1u << 24)) != 0");
    write_value(state, &io, 0x4004c0a0u, 4, 1u);
    expect(state, !kinetis_io_irq_asserted(&io, 62u), "!kinetis_io_irq_asserted(&io, 62u)");
    expect(state, (read_value(state, &io, 0x4004c000u, 4) & (1u << 24)) == 0,
           "(read_value(state, &io, 0x4004c000u, 4) & (1u << 24)) == 0");
    write_value(state, &io, 0x4004c004u, 4, 1u << 16);
    expect(state, kinetis_io_drive_pin(&io, 3, 1, false), "kinetis_io_drive_pin(&io, 3, 1, false)");
    expect(state, kinetis_io_drive_pin(&io, 3, 1, true), "kinetis_io_drive_pin(&io, 3, 1, true)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_DMA, 97u),
           "has_event(&log, KINETIS_IO_EVENT_DMA, 97u)");
    write_value(state, &io, 0x4004c0c0u, 4, 1u);
    write_value(state, &io, 0x4004c0c8u, 1, 3u);
    write_value(state, &io, 0x4004c000u, 4, 10u << 16);
    expect(state, kinetis_io_drive_pin(&io, 3, 0, false), "kinetis_io_drive_pin(&io, 3, 0, false)");
    kinetis_io_advance(&io, 3);
    expect(state, (kinetis_io_pin_input(&io, 3) & 1u) != 0,
           "(kinetis_io_pin_input(&io, 3) & 1u) != 0");
    kinetis_io_advance(&io, 1);
    expect(state, (kinetis_io_pin_input(&io, 3) & 1u) == 0,
           "(kinetis_io_pin_input(&io, 3) & 1u) == 0");
    write_value(state, &io, 0x4004c000u, 4, (10u << 16) | 3u);
    expect(state, kinetis_io_release_pin(&io, 3, 0), "kinetis_io_release_pin(&io, 3, 0)");
    kinetis_io_advance(&io, 4);
    expect(state, (kinetis_io_pin_input(&io, 3) & 1u) != 0,
           "(kinetis_io_pin_input(&io, 3) & 1u) != 0");
    write_value(state, &io, 0x4004c0c0u, 4, 3u);
    expect(state, kinetis_io_drive_pin(&io, 3, 1, true), "kinetis_io_drive_pin(&io, 3, 1, true)");
    expect(state, kinetis_io_release_pin(&io, 3, 1), "kinetis_io_release_pin(&io, 3, 1)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_USB0, true);
    write_value(state, &io, bit_band_address(USB0 + 0x84u, 3), 4, 1u);
    expect(state, read_value(state, &io, USB0 + 0x84u, 1) == 8u,
           "read_value(state, &io, USB0 + 0x84u, 1) == 8u");
    expect(state, read_value(state, &io, bit_band_address(USB0 + 0x84u, 3), 4) == 1u,
           "read_value(state, &io, bit_band_address(USB0 + 0x84u, 3), 4) == 1u");
}

static void test_usb(TestState* state) {
    EventLog log = {0};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN51212));
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_USB0, true);
    expect(state, read_value(state, &io, USB0, 1) == 4u, "read_value(state, &io, USB0, 1) == 4u");
    expect(state, read_value(state, &io, USB0 + 4u, 1) == 0xfbu,
           "read_value(state, &io, USB0 + 4u, 1) == 0xfbu");
    expect(state, read_value(state, &io, USB0 + 8u, 1) == 0x33u,
           "read_value(state, &io, USB0 + 8u, 1) == 0x33u");
    expect(state, !kinetis_io_write(&io, USB0, 1, 0), "!kinetis_io_write(&io, USB0, 1, 0)");
    write_value(state, &io, USB0 + 0x84u, 1, (1u << 3) | (1u << 2));
    write_value(state, &io, USB0 + 0x94u, 1, 1u);
    expect(state, kinetis_io_usb_token(&io, 3, 0x69u, false),
           "kinetis_io_usb_token(&io, 3, 0x69u, false)");
    expect(state, read_value(state, &io, USB0 + 0x90u, 1) == 0x30u,
           "read_value(state, &io, USB0 + 0x90u, 1) == 0x30u");
    expect(state, has_event(&log, KINETIS_IO_EVENT_USB_TOKEN, 3u),
           "has_event(&log, KINETIS_IO_EVENT_USB_TOKEN, 3u)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_IRQ, 53u),
           "has_event(&log, KINETIS_IO_EVENT_IRQ, 53u)");
    expect(state, kinetis_io_irq_asserted(&io, 53u), "kinetis_io_irq_asserted(&io, 53u)");
    write_value(state, &io, USB0 + 0x80u, 1, 1u << 3);
    expect(state, (read_value(state, &io, USB0 + 0x80u, 1) & (1u << 3)) == 0,
           "(read_value(state, &io, USB0 + 0x80u, 1) & (1u << 3)) == 0");
    expect(state, !kinetis_io_irq_asserted(&io, 53u), "!kinetis_io_irq_asserted(&io, 53u)");
    kinetis_io_advance(&io, 2500u);
    expect(state, read_value(state, &io, USB0 + 0xa0u, 1) == 2u,
           "read_value(state, &io, USB0 + 0xa0u, 1) == 2u");
    expect(state, (read_value(state, &io, USB0 + 0x80u, 1) & (1u << 2)) != 0,
           "(read_value(state, &io, USB0 + 0x80u, 1) & (1u << 2)) != 0");
    write_value(state, &io, USB0 + 0xd0u, 1, 0x80u);
    expect(state, read_value(state, &io, USB0, 1) == 4u, "read_value(state, &io, USB0, 1) == 4u");
    expect(state, read_value(state, &io, USB0 + 0x94u, 1) == 0,
           "read_value(state, &io, USB0 + 0x94u, 1) == 0");
}

static void test_can(TestState* state) {
    EventLog log = {0};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN1M012));
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_CAN0, true);
    expect(state, read_value(state, &io, CAN0, 4) == 0xd890000fu,
           "read_value(state, &io, CAN0, 4) == 0xd890000fu");
    write_value(state, &io, CAN0, 4, 0x0fu);
    write_value(state, &io, CAN0 + 0x10u, 4, 0);
    write_value(state, &io, CAN0 + 0x28u, 4, 1u);
    write_value(state, &io, CAN0 + 0x80u, 4, 4u << 24);
    KinetisCanFrame receive_frame = {0x123u, 8, {0, 1, 2, 3, 4, 5, 6, 7}, false, false};
    expect(state, kinetis_io_can_receive(&io, &receive_frame),
           "kinetis_io_can_receive(&io, &receive_frame)");
    expect(state, (read_value(state, &io, CAN0 + 0x30u, 4) & 1u) != 0,
           "(read_value(state, &io, CAN0 + 0x30u, 4) & 1u) != 0");
    expect(state, read_value(state, &io, CAN0 + 0x84u, 4) == 0x123u,
           "read_value(state, &io, CAN0 + 0x84u, 4) == 0x123u");
    expect(state, read_value(state, &io, CAN0 + 0x88u, 4) == 0x00010203u,
           "read_value(state, &io, CAN0 + 0x88u, 4) == 0x00010203u");
    expect(state, has_event(&log, KINETIS_IO_EVENT_IRQ, 75u),
           "has_event(&log, KINETIS_IO_EVENT_IRQ, 75u)");
    expect(state, kinetis_io_irq_asserted(&io, 75u), "kinetis_io_irq_asserted(&io, 75u)");
    write_value(state, &io, CAN0 + 0x30u, 4, 1u);
    expect(state, read_value(state, &io, CAN0 + 0x30u, 4) == 0,
           "read_value(state, &io, CAN0 + 0x30u, 4) == 0");
    expect(state, !kinetis_io_irq_asserted(&io, 75u), "!kinetis_io_irq_asserted(&io, 75u)");
    write_value(state, &io, CAN0 + 0x94u, 4, 0x321u);
    write_value(state, &io, CAN0 + 0x98u, 4, 0x01020304u);
    write_value(state, &io, CAN0 + 0x9cu, 4, 0x05060708u);
    write_value(state, &io, CAN0 + 0x90u, 4, (0xcu << 24) | (3u << 16));
    expect(state, has_event(&log, KINETIS_IO_EVENT_CAN_TRANSMIT, 1u),
           "has_event(&log, KINETIS_IO_EVENT_CAN_TRANSMIT, 1u)");
    const KinetisIoEvent* transmit_event = find_event(&log, KINETIS_IO_EVENT_CAN_TRANSMIT, 1u);
    expect(state, transmit_event != NULL, "transmit_event != NULL");
    expect(state, transmit_event->length == 3u, "transmit_event->length == 3u");
    expect(state, transmit_event->data[0] == 1u, "transmit_event->data[0] == 1u");
    expect(state, transmit_event->data[7] == 8u, "transmit_event->data[7] == 8u");
    kinetis_io_advance(&io, 17u);
    expect(state, read_value(state, &io, CAN0 + 8u, 4) == 17u,
           "read_value(state, &io, CAN0 + 8u, 4) == 17u");
    write_value(state, &io, CAN0, 4, 1u << 25);
    expect(state, read_value(state, &io, CAN0, 4) == 0xd890000fu,
           "read_value(state, &io, CAN0, 4) == 0xd890000fu");
}

static void test_i2s(TestState* state) {
    EventLog log = {0};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN51212));
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_I2S0, true);
    write_value(state, &io, I2S0, 4, UINT32_C(0x80000100));
    write_value(state, &io, I2S0 + 0x80u, 4, UINT32_C(0x80000100));
    write_value(state, &io, I2S0 + 0x20u, 4, 0x12345678u);
    expect(state, has_event(&log, KINETIS_IO_EVENT_I2S_TRANSMIT, 0),
           "has_event(&log, KINETIS_IO_EVENT_I2S_TRANSMIT, 0)");
    uint32_t transmit_sample = 0u;
    expect(state, kinetis_io_i2s_transmit(&io, &transmit_sample),
           "kinetis_io_i2s_transmit(&io, &transmit_sample)");
    expect(state, transmit_sample == 0x12345678u, "transmit_sample == 0x12345678u");
    expect(state, kinetis_io_i2s_receive(&io, 0x87654321u),
           "kinetis_io_i2s_receive(&io, 0x87654321u)");
    expect(state, read_value(state, &io, I2S0 + 0xa0u, 4) == 0x87654321u,
           "read_value(state, &io, I2S0 + 0xa0u, 4) == 0x87654321u");
    expect(state, read_value(state, &io, I2S0 + 0xa0u, 4) == 0,
           "read_value(state, &io, I2S0 + 0xa0u, 4) == 0");
    expect(state, (read_value(state, &io, I2S0 + 0x80u, 4) & (1u << 18)) != 0,
           "(read_value(state, &io, I2S0 + 0x80u, 4) & (1u << 18)) != 0");
    expect(state, has_event(&log, KINETIS_IO_EVENT_IRQ, 28u),
           "has_event(&log, KINETIS_IO_EVENT_IRQ, 28u)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_IRQ, 29u),
           "has_event(&log, KINETIS_IO_EVENT_IRQ, 29u)");
    expect(state, kinetis_io_irq_asserted(&io, 28u), "kinetis_io_irq_asserted(&io, 28u)");
    expect(state, !kinetis_io_irq_asserted(&io, 29u), "!kinetis_io_irq_asserted(&io, 29u)");
    write_value(state, &io, I2S0, 4, UINT32_C(0x80000000));
    expect(state, !kinetis_io_irq_asserted(&io, 28u), "!kinetis_io_irq_asserted(&io, 28u)");
}

static void test_flexbus_sysmpu_copy_and_reset(TestState* state) {
    EventLog log = {0};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN1M012));
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    KinetisIo copied_io;
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_FB, true);
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_SYSMPU, true);
    write_value(state, &io, FLEXBUS, 4, 0x60000000u);
    write_value(state, &io, FLEXBUS + 4u, 4, 1u);
    expect(state, read_value(state, &io, FLEXBUS, 4) == 0x60000000u,
           "read_value(state, &io, FLEXBUS, 4) == 0x60000000u");
    expect(state, has_event(&log, KINETIS_IO_EVENT_FLEXBUS_TRANSFER, 0),
           "has_event(&log, KINETIS_IO_EVENT_FLEXBUS_TRANSFER, 0)");
    expect(state, kinetis_io_flexbus_transfer(&io, 0x60001234u, 4u, false, 0u),
           "kinetis_io_flexbus_transfer(&io, 0x60001234u, 4u, false, 0u)");
    expect(state, kinetis_io_flexbus_transfer(&io, 0x60005678u, 2u, true, 0x55aau),
           "kinetis_io_flexbus_transfer(&io, 0x60005678u, 2u, true, 0x55aau)");
    expect(state, !kinetis_io_flexbus_transfer(&io, 0x50000000u, 4u, false, 0u),
           "!kinetis_io_flexbus_transfer(&io, 0x50000000u, 4u, false, 0u)");
    expect(state, !kinetis_io_flexbus_transfer(&io, 0x60000000u, 3u, false, 0u),
           "!kinetis_io_flexbus_transfer(&io, 0x60000000u, 3u, false, 0u)");
    expect(state, read_value(state, &io, SYSMPU, 4) == 0x00815101u,
           "read_value(state, &io, SYSMPU, 4) == 0x00815101u");
    expect(state, read_value(state, &io, SYSMPU + 0x404u, 4) == UINT32_MAX,
           "read_value(state, &io, SYSMPU + 0x404u, 4) == UINT32_MAX");
    expect(state, read_value(state, &io, SYSMPU + 0x408u, 4) == 0x0061f7dfu,
           "read_value(state, &io, SYSMPU + 0x408u, 4) == 0x0061f7dfu");
    expect(state, kinetis_io_sysmpu_access(&io, 0x1000u, 0, false, KINETIS_SYSMPU_WRITE),
           "kinetis_io_sysmpu_access(&io, 0x1000u, 0, false, KINETIS_SYSMPU_WRITE)");
    for (uint8_t region_index = 0u; region_index < 12u; region_index++) {
        write_value(state, &io, SYSMPU + 0x40cu + (uint32_t)region_index * 16u, 4, 0);
    }
    write_value(state, &io, SYSMPU + 0x400u, 4, 0x1000u);
    write_value(state, &io, SYSMPU + 0x404u, 4, 0x1fffu);
    write_value(state, &io, SYSMPU + 0x408u, 4, 4u | (3u << 3));
    expect(state, read_value(state, &io, SYSMPU + 0x800u, 4) == (4u | (3u << 3)),
           "read_value(state, &io, SYSMPU + 0x800u, 4) == (4u | (3u << 3))");
    write_value(state, &io, SYSMPU + 0x800u, 4, 7u);
    expect(state, read_value(state, &io, SYSMPU + 0x408u, 4) == 7u,
           "read_value(state, &io, SYSMPU + 0x408u, 4) == 7u");
    write_value(state, &io, SYSMPU + 0x408u, 4, 4u | (3u << 3));
    write_value(state, &io, SYSMPU + 0x40cu, 4, 1u);
    expect(state, kinetis_io_sysmpu_access(&io, 0x1800u, 0, false, KINETIS_SYSMPU_READ),
           "kinetis_io_sysmpu_access(&io, 0x1800u, 0, false, KINETIS_SYSMPU_READ)");
    expect(state, !kinetis_io_sysmpu_access(&io, 0x1800u, 0, false, KINETIS_SYSMPU_WRITE),
           "!kinetis_io_sysmpu_access(&io, 0x1800u, 0, false, KINETIS_SYSMPU_WRITE)");
    expect(state, read_value(state, &io, SYSMPU + 0x10u, 4) == 0x1800u,
           "read_value(state, &io, SYSMPU + 0x10u, 4) == 0x1800u");
    expect(state, has_event(&log, KINETIS_IO_EVENT_ACCESS_ERROR, 0x1800u),
           "has_event(&log, KINETIS_IO_EVENT_ACCESS_ERROR, 0x1800u)");
    write_value(state, &io, SYSMPU, 4, 1u << 27);
    expect(state, read_value(state, &io, SYSMPU, 4) == 0x00815100u,
           "read_value(state, &io, SYSMPU, 4) == 0x00815100u");
    expect(state, !kinetis_io_write(&io, SYSMPU + 0x10u, 4, 1u),
           "!kinetis_io_write(&io, SYSMPU + 0x10u, 4, 1u)");
    write_value(state, &io, SYSMPU + 0x400u, 4, 0x1234u);
    expect(state, read_value(state, &io, SYSMPU + 0x400u, 4) == 0x1234u,
           "read_value(state, &io, SYSMPU + 0x400u, 4) == 0x1234u");
    expect(state, kinetis_io_init(&copied_io, configuration),
           "kinetis_io_init(&copied_io, configuration)");
    expect(state, kinetis_io_copy(&copied_io, &io), "kinetis_io_copy(&copied_io, &io)");
    expect(state, read_value(state, &copied_io, FLEXBUS, 4) == 0x60000000u,
           "read_value(state, &copied_io, FLEXBUS, 4) == 0x60000000u");
    kinetis_io_reset(&copied_io);
    expect(state, !kinetis_io_clock_enabled(&copied_io, KINETIS_PERIPHERAL_FB),
           "!kinetis_io_clock_enabled(&copied_io, KINETIS_PERIPHERAL_FB)");
    expect(state, kinetis_io_clock_enabled(&copied_io, KINETIS_PERIPHERAL_MCM),
           "kinetis_io_clock_enabled(&copied_io, KINETIS_PERIPHERAL_MCM)");
    expect(state, kinetis_io_copy(&copied_io, &copied_io),
           "kinetis_io_copy(&copied_io, &copied_io)");
    expect(state, !kinetis_io_copy(NULL, &io), "!kinetis_io_copy(NULL, &io)");
}

static void test_edges_and_fail_closed_access(TestState* state) {
    EventLog log = {0};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN1M012));
    configuration.event_handler = record_event;
    configuration.event_context = &log;
    KinetisIo io;
    expect(state, kinetis_io_init(&io, configuration), "kinetis_io_init(&io, configuration)");
    kinetis_io_reset(NULL);
    kinetis_io_advance(NULL, 1);
    kinetis_io_advance(&io, 0);
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_COUNT, true);
    expect(state, !kinetis_io_clock_enabled(NULL, KINETIS_PERIPHERAL_USB0),
           "!kinetis_io_clock_enabled(NULL, KINETIS_PERIPHERAL_USB0)");
    uint32_t read_data = 0u;
    expect(state, !kinetis_io_read(NULL, PORTA, 4, &read_data),
           "!kinetis_io_read(NULL, PORTA, 4, &read_data)");
    expect(state, !kinetis_io_read(&io, PORTA, 3, &read_data),
           "!kinetis_io_read(&io, PORTA, 3, &read_data)");
    expect(state, !kinetis_io_read(&io, PORTA, 4, NULL), "!kinetis_io_read(&io, PORTA, 4, NULL)");
    expect(state, !kinetis_io_write(NULL, PORTA, 4, 0), "!kinetis_io_write(NULL, PORTA, 4, 0)");
    expect(state, !kinetis_io_write(&io, PORTA, 3, 0), "!kinetis_io_write(&io, PORTA, 3, 0)");
    expect(state, !kinetis_io_write(&io, PORTA, 4, 0), "!kinetis_io_write(&io, PORTA, 4, 0)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_ACCESS_ERROR, PORTA),
           "has_event(&log, KINETIS_IO_EVENT_ACCESS_ERROR, PORTA)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_PORTD, true);
    write_value(state, &io, 0x4004c0c0u, 4, 3u);
    write_value(state, &io, 0x4004c0c4u, 1, 1u);
    write_value(state, &io, 0x4004c0c8u, 1, 7u);
    expect(state, read_value(state, &io, 0x4004c0c0u, 4) == 3u,
           "read_value(state, &io, 0x4004c0c0u, 4) == 3u");
    expect(state, read_value(state, &io, 0x4004c0c4u, 1) == 1u,
           "read_value(state, &io, 0x4004c0c4u, 1) == 1u");
    expect(state, read_value(state, &io, 0x4004c0c8u, 1) == 7u,
           "read_value(state, &io, 0x4004c0c8u, 1) == 7u");
    expect(state, read_value(state, &io, 0x4004c080u, 4) == 0,
           "read_value(state, &io, 0x4004c080u, 4) == 0");
    write_value(state, &io, 0x4004c0c0u, 4, 0);
    write_value(state, &io, 0x4004c000u, 4, 3u << 16);
    expect(state, kinetis_io_drive_pin(&io, 3, 0, true), "kinetis_io_drive_pin(&io, 3, 0, true)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_DMA, 96u),
           "has_event(&log, KINETIS_IO_EVENT_DMA, 96u)");
    expect(state, !kinetis_io_irq_asserted(&io, 62u), "!kinetis_io_irq_asserted(&io, 62u)");
    write_value(state, &io, 0x4004c000u, 4, 8u << 16);
    expect(state, kinetis_io_drive_pin(&io, 3, 0, false), "kinetis_io_drive_pin(&io, 3, 0, false)");
    write_value(state, &io, 0x4004c000u, 4, 12u << 16);
    expect(state, kinetis_io_drive_pin(&io, 3, 0, true), "kinetis_io_drive_pin(&io, 3, 0, true)");
    write_value(state, &io, 0x4004c000u, 4, (9u << 16) | (1u << 15) | (1u << 24));
    expect(state, kinetis_io_drive_pin(&io, 3, 0, false), "kinetis_io_drive_pin(&io, 3, 0, false)");
    expect(state, kinetis_io_drive_pin(&io, 3, 0, true), "kinetis_io_drive_pin(&io, 3, 0, true)");
    write_value(state, &io, 0x4004c000u, 4, 1u << 24);
    expect(state, (read_value(state, &io, 0x4004c000u, 4) & (1u << 24)) == 0,
           "(read_value(state, &io, 0x4004c000u, 4) & (1u << 24)) == 0");
    write_value(state, &io, 0x4004c084u, 4, (1u << 16) | (1u << 8));
    expect(state, (read_value(state, &io, 0x4004c040u, 4) & (7u << 8)) != 0,
           "(read_value(state, &io, 0x4004c040u, 4) & (7u << 8)) != 0");
    write_value(state, &io, 0x400ff0c0u + 0x14u, 4, 3u);
    write_value(state, &io, 0x400ff0c0u, 4, 3u);
    write_value(state, &io, 0x400ff0c0u + 8u, 4, 1u);
    expect(state, read_value(state, &io, 0x400ff0c0u, 4) == 2u,
           "read_value(state, &io, 0x400ff0c0u, 4) == 2u");
    write_value(state, &io, 0x400ff0c0u + 0x0cu, 4, 3u);
    expect(state, read_value(state, &io, 0x400ff0c0u, 4) == 1u,
           "read_value(state, &io, 0x400ff0c0u, 4) == 1u");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_USB0, true);
    write_value(state, &io, USB0 + 0x114u, 1, 0x55u);
    expect(state, read_value(state, &io, USB0 + 0x114u, 1) == 0x55u,
           "read_value(state, &io, USB0 + 0x114u, 1) == 0x55u");
    expect(state, !kinetis_io_read(&io, USB0 + 0x114u, 2, &read_data),
           "!kinetis_io_read(&io, USB0 + 0x114u, 2, &read_data)");
    expect(state, !kinetis_io_usb_token(&io, 16, 0, false),
           "!kinetis_io_usb_token(&io, 16, 0, false)");
    expect(state, !kinetis_io_usb_token(&io, 0, 0, false),
           "!kinetis_io_usb_token(&io, 0, 0, false)");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_CAN0, true);
    write_value(state, &io, CAN0, 4, 0x0fu);
    write_value(state, &io, CAN0 + 0x28u, 4, 2u);
    write_value(state, &io, CAN0 + 0x94u, 4, 0x321u);
    write_value(state, &io, CAN0 + 0x90u, 4, (0xcu << 24) | (1u << 16));
    expect(state, has_event(&log, KINETIS_IO_EVENT_IRQ, 75u),
           "has_event(&log, KINETIS_IO_EVENT_IRQ, 75u)");
    write_value(state, &io, CAN0 + 0x10u, 4, UINT32_MAX);
    write_value(state, &io, CAN0 + 0xa0u, 4, 4u << 24);
    write_value(state, &io, CAN0 + 0xa4u, 4, 0x456u);
    KinetisCanFrame receive_frame = {0x123u, 1, {9, 0, 0, 0, 0, 0, 0, 0}, true, true};
    expect(state, !kinetis_io_can_receive(&io, &receive_frame),
           "!kinetis_io_can_receive(&io, &receive_frame)");
    receive_frame.identifier = 0x456u;
    expect(state, kinetis_io_can_receive(&io, &receive_frame),
           "kinetis_io_can_receive(&io, &receive_frame)");
    expect(state, (read_value(state, &io, CAN0 + 0xa0u, 4) & ((1u << 21) | (1u << 20))) != 0,
           "(read_value(state, &io, CAN0 + 0xa0u, 4) & ((1u << 21) | (1u << 20))) != 0");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_I2S0, true);
    write_value(state, &io, I2S0, 4, UINT32_C(0x80000001));
    for (uint8_t fifo_index = 0u; fifo_index < KINETIS_IO_FIFO_CAPACITY; fifo_index++) {
        write_value(state, &io, I2S0 + 0x20u, 4, fifo_index);
    }
    write_value(state, &io, I2S0 + 0x20u, 4, 99u);
    expect(state, (read_value(state, &io, I2S0, 4) & (1u << 18)) != 0,
           "(read_value(state, &io, I2S0, 4) & (1u << 18)) != 0");
    for (uint8_t fifo_index = 0u; fifo_index < KINETIS_IO_FIFO_CAPACITY; fifo_index++) {
        expect(state, kinetis_io_i2s_transmit(&io, &read_data),
               "kinetis_io_i2s_transmit(&io, &read_data)");
    }
    expect(state, !kinetis_io_i2s_transmit(&io, &read_data),
           "!kinetis_io_i2s_transmit(&io, &read_data)");
    write_value(state, &io, I2S0 + 0x100u, 4, 0x1234u);
    expect(state, read_value(state, &io, I2S0 + 0x100u, 4) == 0x1234u,
           "read_value(state, &io, I2S0 + 0x100u, 4) == 0x1234u");
    write_value(state, &io, I2S0, 4, 0);
    write_value(state, &io, I2S0 + 0x80u, 4, UINT32_C(0x80000001));
    expect(state, kinetis_io_i2s_receive(&io, 0xa5a55a5au),
           "kinetis_io_i2s_receive(&io, 0xa5a55a5au)");
    expect(state, has_event(&log, KINETIS_IO_EVENT_DMA, 1u),
           "has_event(&log, KINETIS_IO_EVENT_DMA, 1u)");
    write_value(state, &io, I2S0 + 0x80u, 4, 0);
    expect(state, read_value(state, &io, I2S0 + 0xa0u, 4) == 0u,
           "read_value(state, &io, I2S0 + 0xa0u, 4) == 0u");
    kinetis_io_set_clock(&io, KINETIS_PERIPHERAL_SYSMPU, true);
    write_value(state, &io, SYSMPU, 4, (1u << 27) | 1u);
    write_value(state, &io, SYSMPU + 0x408u, 4, 1u << 31);
    expect(state, kinetis_io_sysmpu_access(&io, 0, 7, false, KINETIS_SYSMPU_READ),
           "kinetis_io_sysmpu_access(&io, 0, 7, false, KINETIS_SYSMPU_READ)");
    expect(state, !kinetis_io_sysmpu_access(&io, 0, 7, false, KINETIS_SYSMPU_EXECUTE),
           "!kinetis_io_sysmpu_access(&io, 0, 7, false, KINETIS_SYSMPU_EXECUTE)");
    expect(state, !kinetis_io_sysmpu_access(&io, 0, 8, false, KINETIS_SYSMPU_READ),
           "!kinetis_io_sysmpu_access(&io, 0, 8, false, KINETIS_SYSMPU_READ)");
    expect(state, !kinetis_io_sysmpu_access(NULL, 0, 0, false, KINETIS_SYSMPU_READ),
           "!kinetis_io_sysmpu_access(NULL, 0, 0, false, KINETIS_SYSMPU_READ)");

    KinetisIoConfiguration quiet_configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN1M012));
    KinetisIo quiet;
    expect(state, kinetis_io_init(&quiet, quiet_configuration),
           "kinetis_io_init(&quiet, quiet_configuration)");
    expect(state, !kinetis_io_read(&quiet, PORTA, 4, &read_data),
           "!kinetis_io_read(&quiet, PORTA, 4, &read_data)");
    KinetisCanFrame oversized_frame = {0};
    oversized_frame.length = 9u;
    expect(state,
           !kinetis_io_can_receive(NULL, &oversized_frame) &&
               !kinetis_io_can_receive(&quiet, &oversized_frame) &&
               !kinetis_io_i2s_receive(&quiet, 0u) && !kinetis_io_irq_asserted(NULL, 0u) &&
               !kinetis_io_irq_asserted(&quiet, 0u),
           "I/O APIs reject unavailable inputs");
    expect(state, !kinetis_io_sysmpu_access(&quiet, 0u, 0u, false, KINETIS_SYSMPU_READ),
           "clock-gated SYSMPU rejects access");
    kinetis_io_set_clock(&quiet, KINETIS_PERIPHERAL_SYSMPU, true);
    expect(state, kinetis_io_sysmpu_access(&quiet, 0u, 0u, false, KINETIS_SYSMPU_READ),
           "disabled SYSMPU permits access");
    kinetis_io_set_clock(&quiet, KINETIS_PERIPHERAL_CAN0, true);
    write_value(state, &quiet, CAN0, 4, 0x0fu);
    write_value(state, &quiet, CAN0 + 0x90u, 4, (0xcu << 24) | (1u << 16));
}

int main(void) {
    TestState state = {0};
    test_reset_clock_and_configuration(&state);
    test_kv30_mcm_reset(&state);
    test_gpio_mux_pull_open_drain_and_lock(&state);
    test_gpio_interrupt_dma_filter_and_bit_band(&state);
    test_usb(&state);
    test_can(&state);
    test_i2s(&state);
    test_flexbus_sysmpu_copy_and_reset(&state);
    test_edges_and_fail_closed_access(&state);
    kinetis_io_test_peripheral_boundaries(&state);
    return test_finish(&state);
}
