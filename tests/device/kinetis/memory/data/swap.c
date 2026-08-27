#include "device/kinetis/memory/data/internal.h"

static void test_flash_command_guards(TestState* state) {
    TestBus bus;
    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);
    uint32_t value = 0u;
    expect(state, !kinetis_data_internal_flash_read(data, FTFA - 1u, 1u, &value),
           "!kinetis_data_internal_flash_read(data, FTFA - 1u, 1u, &value)");
    expect(state, !kinetis_data_internal_flash_read(data, FTFA + 0x14u, 1u, &value),
           "!kinetis_data_internal_flash_read(data, FTFA + 0x14u, 1u, &value)");
    expect(state, !kinetis_data_internal_flash_write(data, FTFA - 1u, 1u, 0u),
           "!kinetis_data_internal_flash_write(data, FTFA - 1u, 1u, 0u)");
    expect(state, !kinetis_data_internal_flash_write(data, FTFA + 2u, 1u, 0u),
           "!kinetis_data_internal_flash_write(data, FTFA + 2u, 1u, 0u)");
    data->flash[0] = 0u;
    expect(state, kinetis_data_internal_flash_write(data, FTFA + 4u, 1u, 0x55u),
           "kinetis_data_internal_flash_write(data, FTFA + 4u, 1u, 0x55u)");
    expect(state, kinetis_data_internal_flash_write(data, FTFA + 0x10u, 1u, 0u),
           "kinetis_data_internal_flash_write(data, FTFA + 0x10u, 1u, 0u)");
    data->flash[0] = 0x80u;
    kinetis_data_test_flash_command_without_address(state, data, 0xffu, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 1u, 8u);
    kinetis_data_test_flash_command_without_address(state, data, 0x41u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command_without_address(state, data, 0x43u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    bus.flash[0u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x08u, 1u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    expect(state, bus.flash[0u] == 0u, "bus.flash[0u] == 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command(state, data, 0x09u, 1u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    expect(state, bus.flash[0u] == 0u, "bus.flash[0u] == 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command(state, data, 0x08u, 0x700000u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x03u, 1u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN51212);
    kinetis_data_test_write_fccob(state, data, 1u, 8u);
    kinetis_data_test_flash_command_without_address(state, data, 0x41u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command_without_address(state, data, 0x43u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 8u, 0u);
    kinetis_data_test_flash_command(state, data, 0x03u, 1u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    uint8_t configuration[16];
    memset(configuration, 0xff, sizeof(configuration));
    expect(state, kinetis_data_set_flash_configuration(data, configuration, sizeof(configuration)),
           "kinetis_data_set_flash_configuration(data, configuration, sizeof(configuration))");
    kinetis_data_reset(data);
    kinetis_data_test_write_fccob(state, data, 1u, 0u);
    kinetis_data_test_flash_command_without_address(state, data, 0x81u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 3u, 0u);
    kinetis_data_test_write_fccob(state, data, 4u, 2u);
    kinetis_data_test_write_fccob(state, data, 5u, 1u);
    kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 3u, 0u);
    kinetis_data_test_write_fccob(state, data, 4u, 2u);
    kinetis_data_test_write_fccob(state, data, 5u, 3u);
    kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u");
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x00u, 0x800000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_value(state, data, 0x10000000u, 1u, 0u);
    kinetis_data_test_flash_command(state, data, 0x08u, 0x817ff0u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    expect(state, kinetis_data_test_read_value(state, data, 0x10000000u, 1u) == 0u,
           "kinetis_data_test_read_value(state, data, 0x10000000u, 1u) == 0u");
    kinetis_data_destroy(data);
}

static void test_flash_swap_lifecycle(TestState* state) {
    TestBus bus;
    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);

    kinetis_data_test_write_fccob(state, data, 4u, 4u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 0u");
    expect(state, kinetis_data_test_read_fccob(state, data, 6u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 6u) == 0u");
    expect(state, kinetis_data_test_read_fccob(state, data, 7u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 7u) == 0u");

    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x21u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x21u) == 0u");
    expect(state, bus.flash[0x1000u] == 0u, "bus.flash[0x1000u] == 0u");
    expect(state, bus.flash[0x1001u] == 0xffu, "bus.flash[0x1001u] == 0xffu");
    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 2u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 2u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x2000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 0x10u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x80000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    const uint8_t phrase[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    kinetis_data_test_set_flash_data(state, data, phrase, sizeof(phrase));
    kinetis_data_test_flash_command(state, data, 0x07u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u");
    expect(state, bus.flash[0x1001u] == 0xffu, "bus.flash[0x1001u] == 0xffu");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command(state, data, 0x09u, 0x1000u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u");
    expect(state, bus.flash[0x1001u] == 0xffu, "bus.flash[0x1001u] == 0xffu");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 3u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 3u");
    expect(state, kinetis_data_test_read_fccob(state, data, 6u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 6u) == 0u");
    expect(state, kinetis_data_test_read_fccob(state, data, 7u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 7u) == 0u");

    kinetis_data_test_write_fccob(state, data, 4u, 4u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, bus.flash[0x1000u] == 0u, "bus.flash[0x1000u] == 0u");
    expect(state, bus.flash[0x1001u] == 0u, "bus.flash[0x1001u] == 0u");
    kinetis_data_test_flash_command(state, data, 0x09u, 0x81000u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u");
    expect(state, bus.flash[0x81000u] == 0xffu, "bus.flash[0x81000u] == 0xffu");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_reset(data);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 8u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 8u) != 0u");
    expect(state, kinetis_data_program_flash_address(data, 0x20u) == 0x80020u,
           "kinetis_data_program_flash_address(data, 0x20u) == 0x80020u");
    expect(state, kinetis_data_program_flash_address(data, 0x80020u) == 0x20u,
           "kinetis_data_program_flash_address(data, 0x80020u) == 0x20u");
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 1u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 1u");
    expect(state, kinetis_data_test_read_fccob(state, data, 6u) == 1u,
           "kinetis_data_test_read_fccob(state, data, 6u) == 1u");
    expect(state, kinetis_data_test_read_fccob(state, data, 7u) == 1u,
           "kinetis_data_test_read_fccob(state, data, 7u) == 1u");

    kinetis_data_test_write_fccob(state, data, 4u, 2u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, bus.flash[0x81000u] == 0u, "bus.flash[0x81000u] == 0u");
    expect(state, bus.flash[0x81001u] == 0xffu, "bus.flash[0x81001u] == 0xffu");
    kinetis_data_test_flash_command(state, data, 0x09u, 0x81000u, 2000u);
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 3u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 3u");
    expect(state, kinetis_data_test_read_fccob(state, data, 6u) == 1u,
           "kinetis_data_test_read_fccob(state, data, 6u) == 1u");
    expect(state, kinetis_data_test_read_fccob(state, data, 7u) == 1u,
           "kinetis_data_test_read_fccob(state, data, 7u) == 1u");
    kinetis_data_test_write_fccob(state, data, 4u, 4u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, bus.flash[0x81000u] == 0u, "bus.flash[0x81000u] == 0u");
    expect(state, bus.flash[0x81001u] == 0u, "bus.flash[0x81001u] == 0u");
    kinetis_data_reset(data);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 8u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 8u) == 0u");
    expect(state, kinetis_data_program_flash_address(data, 0x20u) == 0x20u,
           "kinetis_data_program_flash_address(data, 0x20u) == 0x20u");

    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1001u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x400u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_destroy(data);
}

static void test_flash_swap_indicator_failures(TestState* state) {
    TestBus bus;
    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);
    bus.fail_write = true;
    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    bus.fail_write = false;
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 0u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);
    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    bus.fail_write = true;
    kinetis_data_test_write_fccob(state, data, 4u, 4u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    bus.fail_write = false;
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 3u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 3u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);
    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    kinetis_data_test_write_fccob(state, data, 4u, 4u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    kinetis_data_reset(data);
    bus.fail_write = true;
    kinetis_data_test_write_fccob(state, data, 4u, 2u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    bus.fail_write = false;
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x1000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 1u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 1u");
    kinetis_data_destroy(data);
}

static void test_flash_partition_codes(TestState* state) {
    static const struct {
        uint8_t code;
        uint8_t unused_fccob3;
        uint32_t data_size;
    } cases[] = {{0x00u, 0xa5u, 0x20000u}, {0x03u, 0u, 0x18000u}, {0x04u, 0u, 0x10000u},
                 {0x05u, 0u, 0u},          {0x08u, 0u, 0u},       {0x0bu, 0u, 0x8000u},
                 {0x0cu, 0u, 0x10000u},    {0x0du, 0u, 0x20000u}};
    const uint8_t phrase[8] = {1u, 3u, 5u, 7u, 9u, 11u, 13u, 15u};
    for (size_t index = 0; index < sizeof(cases) / sizeof(cases[0]); index++) {
        TestBus bus;
        memset(&bus, 0, sizeof(bus));
        memset(bus.flash, 0xff, sizeof(bus.flash));
        KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
        kinetis_data_test_write_fccob(state, data, 3u, cases[index].unused_fccob3);
        kinetis_data_test_write_fccob(state, data, 4u,
                                      cases[index].data_size == 0x20000u ? 0x0fu : 2u);
        kinetis_data_test_write_fccob(state, data, 5u, cases[index].code);
        kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
        expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u,
               "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u");
        kinetis_data_test_set_flash_data(state, data, phrase, sizeof(phrase));
        const uint32_t address = cases[index].data_size == 0u
                                     ? 0x800000u
                                     : 0x800000u + cases[index].data_size - sizeof(phrase);
        kinetis_data_test_flash_command(state, data, 0x07u, address, 40u);
        expect(state,
               ((kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u) ==
                   (cases[index].data_size == 0u),
               "((kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u) == "
               "(cases[index].data_size == 0u)");
        if (cases[index].data_size != 0u) {
            expect(
                state,
                kinetis_data_test_read_value(state, data, 0x10000000u + cases[index].data_size - 8u,
                                             4u) == 0x07050301u,
                "kinetis_data_test_read_value(state, data, 0x10000000u + cases[index].data_size - "
                "8u, 4u) "
                "== 0x07050301u");
            kinetis_data_test_clear_flash_status(state, data);
            kinetis_data_test_flash_command(state, data, 0x07u, 0x800000u + cases[index].data_size,
                                            40u);
            expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
                   "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
        } else {
            kinetis_data_test_clear_flash_status(state, data);
            kinetis_data_test_flash_command(state, data, 0x08u, 0x800000u, 2000u);
            expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
                   "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
        }
        kinetis_data_destroy(data);
    }

    TestBus bus;
    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    kinetis_data_test_write_fccob(state, data, 4u, 0x0fu);
    kinetis_data_test_write_fccob(state, data, 5u, 0x0fu);
    kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "reserved FlexNVM partition code is rejected");
    kinetis_data_destroy(data);
}

int main(void) {
    TestState state = {0};
    kinetis_data_reset(NULL);
    kinetis_data_test_test_profile_boundaries(&state);
    kinetis_data_test_test_api_boundaries(&state);
    kinetis_data_test_test_dma(&state);
    kinetis_data_test_test_dma_advanced(&state);
    kinetis_data_test_test_dmamux_triggers(&state);
    kinetis_data_test_test_dmamux_source_matrix(&state);
    kinetis_data_test_test_dma_arbitration_and_control(&state);
    kinetis_data_test_test_adc(&state);
    kinetis_data_test_test_adc_compare_dma_and_continuous(&state);
    kinetis_data_test_test_dac_cmp_vref(&state);
    kinetis_data_test_test_rng_crc(&state);
    kinetis_data_test_test_flash_flex_copy(&state);
    kinetis_data_test_test_flash_collision_lifecycle(&state);
    kinetis_data_test_test_flash_controller_geometry(&state);
    kinetis_data_test_test_flash_commands_and_failures(&state);
    kinetis_data_test_test_flash_command_semantics(&state);
    test_flash_command_guards(&state);
    test_flash_swap_lifecycle(&state);
    test_flash_swap_indicator_failures(&state);
    test_flash_partition_codes(&state);
    return test_finish(&state);
}
