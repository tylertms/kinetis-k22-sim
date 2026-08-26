#include "device/kinetis/memory/data/internal.h"

void kinetis_data_test_test_flash_flex_copy(TestState* state) {
    TestBus bus = {0};
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    expect(state, kinetis_data_test_read_value(state, data, FTFA, 1) == 0x80u,
           "kinetis_data_test_read_value(state, data, FTFA, 1) == 0x80u");
    kinetis_data_test_write_fccob(state, data, 0u, 0x07u);
    kinetis_data_test_write_fccob(state, data, 1u, 0x00u);
    kinetis_data_test_write_fccob(state, data, 2u, 0x10u);
    kinetis_data_test_write_fccob(state, data, 3u, 0x00u);
    kinetis_data_test_write_fccob(state, data, 4u, 0x12u);
    kinetis_data_test_write_fccob(state, data, 5u, 0x34u);
    kinetis_data_test_write_fccob(state, data, 6u, 0x56u);
    kinetis_data_test_write_fccob(state, data, 7u, 0x78u);
    kinetis_data_test_write_fccob(state, data, 8u, 0x9au);
    kinetis_data_test_write_fccob(state, data, 9u, 0xbcu);
    kinetis_data_test_write_fccob(state, data, 10u, 0xdeu);
    kinetis_data_test_write_fccob(state, data, 11u, 0xf0u);
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x80u);
    expect(state, kinetis_data_test_read_value(state, data, FTFA, 1) == 0,
           "kinetis_data_test_read_value(state, data, FTFA, 1) == 0");
    expect(state, kinetis_data_test_load(bus.flash, 0x1000, 4) == 0x78563412u,
           "kinetis_data_test_load(bus.flash, 0x1000, 4) == 0x78563412u");
    expect(state, kinetis_data_test_load(bus.flash, 0x1004, 4) == 0xf0debc9au,
           "kinetis_data_test_load(bus.flash, 0x1004, 4) == 0xf0debc9au");
    kinetis_data_advance(data, 40);
    expect(state, kinetis_data_test_read_value(state, data, FTFA, 1) == 0x80u,
           "kinetis_data_test_read_value(state, data, FTFA, 1) == 0x80u");
    kinetis_data_test_write_fccob(state, data, 0u, 0xffu);
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x80u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0");
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x20u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) == 0,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) == 0");

    uint8_t configuration[16];
    memset(configuration, 0xff, sizeof(configuration));
    configuration[8] = 0xfeu;
    configuration[0x0c] = 0xfeu;
    expect(state, kinetis_data_set_flash_configuration(data, configuration, sizeof(configuration)),
           "kinetis_data_set_flash_configuration(data, configuration, sizeof(configuration))");
    expect(state, kinetis_data_test_read_value(state, data, 0x408u, 1) == 0xfeu,
           "kinetis_data_test_read_value(state, data, 0x408u, 1) == 0xfeu");
    kinetis_data_reset(data);
    expect(state, kinetis_data_test_read_value(state, data, FTFA + 0x10, 1) == 0xfeu,
           "kinetis_data_test_read_value(state, data, FTFA + 0x10, 1) == 0xfeu");
    kinetis_data_test_write_fccob(state, data, 0u, 0x07u);
    kinetis_data_test_write_fccob(state, data, 1u, 0x00u);
    kinetis_data_test_write_fccob(state, data, 2u, 0x10u);
    kinetis_data_test_write_fccob(state, data, 3u, 0x00u);
    for (uint8_t index = 4u; index < 12u; index++)
        kinetis_data_test_write_fccob(state, data, index, 0xffu);
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x80u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x10u) != 0,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x10u) != 0");

    expect(state, kinetis_data_test_read_value(state, data, 0x10000000u, 4) == UINT32_MAX,
           "kinetis_data_test_read_value(state, data, 0x10000000u, 4) == UINT32_MAX");
    kinetis_data_test_write_value(state, data, 0x10000000u, 4, 0x55aa55aau);
    kinetis_data_test_write_value(state, data, 0x10000000u, 4, 0xffff0000u);
    expect(state, kinetis_data_test_read_value(state, data, 0x10000000u, 4) == 0x55aa0000u,
           "kinetis_data_test_read_value(state, data, 0x10000000u, 4) == 0x55aa0000u");
    kinetis_data_test_write_value(state, data, 0x14000000u, 4, 0xdeadbeefu);
    expect(state, kinetis_data_test_read_value(state, data, 0x14000000u, 4) == 0xdeadbeefu,
           "kinetis_data_test_read_value(state, data, 0x14000000u, 4) == 0xdeadbeefu");

    KinetisData* copy = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    expect(state, kinetis_data_copy(copy, data), "kinetis_data_copy(copy, data)");
    expect(state, kinetis_data_test_read_value(state, copy, 0x10000000u, 4) == 0x55aa0000u,
           "kinetis_data_test_read_value(state, copy, 0x10000000u, 4) == 0x55aa0000u");
    expect(state, kinetis_data_test_read_value(state, copy, 0x14000000u, 4) == 0xdeadbeefu,
           "kinetis_data_test_read_value(state, copy, 0x14000000u, 4) == 0xdeadbeefu");
    kinetis_data_reset(copy);
    expect(state, kinetis_data_test_read_value(state, copy, 0x10000000u, 4) == 0x55aa0000u,
           "kinetis_data_test_read_value(state, copy, 0x10000000u, 4) == 0x55aa0000u");
    expect(state, kinetis_data_test_read_value(state, copy, 0x14000000u, 4) == 0,
           "kinetis_data_test_read_value(state, copy, 0x14000000u, 4) == 0");
    kinetis_data_destroy(copy);
    kinetis_data_destroy(data);
}
void kinetis_data_test_test_flash_collision_lifecycle(TestState* state) {
    TestBus bus = {0};
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    kinetis_data_test_write_value(state, data, FTFA + 1u, 1u, 0xc0u);
    kinetis_data_test_write_fccob(state, data, 0u, 0x07u);
    kinetis_data_test_set_flash_address(state, data, 0x1000u);
    for (uint8_t index = 4u; index < 12u; index++)
        kinetis_data_test_write_fccob(state, data, index, index);
    kinetis_data_test_write_value(state, data, FTFA, 1u, 0x80u);
    expect(state, !bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA],
           "!bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA]");
    expect(state, !kinetis_data_flash_read(data, false, 0x1000u, 1u),
           "!kinetis_data_flash_read(data, false, 0x1000u, 1u)");
    expect(state, !kinetis_data_flash_read(data, false, 0x41000u, 1u),
           "!kinetis_data_flash_read(data, false, 0x41000u, 1u)");
    expect(state, kinetis_data_flash_read(data, true, 0u, 1u),
           "kinetis_data_flash_read(data, true, 0u, 1u)");
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x40u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x40u) != 0u");
    expect(state, bus.interrupt[KINETIS_DATA_INTERRUPT_FLASH_COLLISION],
           "bus.interrupt[KINETIS_DATA_INTERRUPT_FLASH_COLLISION]");
    kinetis_data_test_write_value(state, data, FTFA, 1u, 0x40u);
    expect(state, !bus.interrupt[KINETIS_DATA_INTERRUPT_FLASH_COLLISION],
           "!bus.interrupt[KINETIS_DATA_INTERRUPT_FLASH_COLLISION]");
    kinetis_data_advance(data, 39u);
    expect(state, !bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA],
           "!bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA]");
    kinetis_data_advance(data, 1u);
    expect(state, bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA],
           "bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA]");
    kinetis_data_test_write_value(state, data, FTFA + 1u, 1u, 0u);
    expect(state, !bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA],
           "!bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA]");

    kinetis_data_test_write_fccob(state, data, 0u, 0x07u);
    kinetis_data_test_set_flash_address(state, data, 0x800000u);
    for (uint8_t index = 4u; index < 12u; index++)
        kinetis_data_test_write_fccob(state, data, index, (uint8_t)(index + 0x10u));
    kinetis_data_test_write_value(state, data, FTFA, 1u, 0x80u);
    expect(state, kinetis_data_flash_read(data, false, 0u, 1u),
           "kinetis_data_flash_read(data, false, 0u, 1u)");
    expect(state, !kinetis_data_flash_read(data, true, 0u, 1u),
           "!kinetis_data_flash_read(data, true, 0u, 1u)");
    kinetis_data_advance(data, 40u);
    expect(state, kinetis_data_flash_read(data, true, 0u, 1u),
           "kinetis_data_flash_read(data, true, 0u, 1u)");
    kinetis_data_destroy(data);
}

void kinetis_data_test_set_flash_address(TestState* state, KinetisData* data, uint32_t address) {
    kinetis_data_test_write_fccob(state, data, 1u, (uint8_t)(address >> 16u));
    kinetis_data_test_write_fccob(state, data, 2u, (uint8_t)(address >> 8u));
    kinetis_data_test_write_fccob(state, data, 3u, (uint8_t)address);
}

static void launch_flash(TestState* state, KinetisData* data, uint32_t cycles) {
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x80u);
    kinetis_data_advance(data, cycles);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x80u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x80u) != 0u");
}

void kinetis_data_test_test_flash_controller_geometry(TestState* state) {
    TestBus bus;
    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);
    bus.flash[0x1000] = 0;
    bus.flash[0x1ffc] = 0;
    bus.flash[0x2000] = 0;
    kinetis_data_test_write_fccob(state, data, 0u, 0x09u);
    kinetis_data_test_set_flash_address(state, data, 0x1000u);
    launch_flash(state, data, 2000u);
    expect(state, kinetis_data_test_load(bus.flash, 0x1000u, 4) == UINT32_MAX,
           "kinetis_data_test_load(bus.flash, 0x1000u, 4) == UINT32_MAX");
    expect(state, kinetis_data_test_load(bus.flash, 0x1ffcu, 4) == UINT32_MAX,
           "kinetis_data_test_load(bus.flash, 0x1ffcu, 4) == UINT32_MAX");
    expect(state, bus.flash[0x2000] == 0, "bus.flash[0x2000] == 0");

    kinetis_data_test_write_fccob(state, data, 0u, 0x43u);
    kinetis_data_test_write_fccob(state, data, 1u, 2u);
    const uint8_t once_data[8] = {0x12u, 0x34u, 0x56u, 0x78u, 0x9au, 0xbcu, 0xdeu, 0xf0u};
    for (uint8_t index = 0u; index < sizeof(once_data); index++)
        kinetis_data_test_write_fccob(state, data, (uint8_t)(4u + index), once_data[index]);
    launch_flash(state, data, 40u);
    kinetis_data_test_write_fccob(state, data, 0u, 0x41u);
    kinetis_data_test_write_fccob(state, data, 1u, 2u);
    launch_flash(state, data, 40u);
    for (uint8_t index = 0u; index < sizeof(once_data); index++)
        expect(
            state,
            kinetis_data_test_read_fccob(state, data, (uint8_t)(4u + index)) == once_data[index],
            "kinetis_data_test_read_fccob(state, data, (uint8_t)(4u + index)) == once_data[index]");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN51212);
    bus.flash[0x1000] = 0;
    bus.flash[0x17fc] = 0;
    bus.flash[0x1800] = 0;
    kinetis_data_test_write_fccob(state, data, 0u, 0x09u);
    kinetis_data_test_set_flash_address(state, data, 0x1000u);
    launch_flash(state, data, 2000u);
    expect(state, kinetis_data_test_load(bus.flash, 0x1000u, 4) == UINT32_MAX,
           "kinetis_data_test_load(bus.flash, 0x1000u, 4) == UINT32_MAX");
    expect(state, kinetis_data_test_load(bus.flash, 0x17fcu, 4) == UINT32_MAX,
           "kinetis_data_test_load(bus.flash, 0x17fcu, 4) == UINT32_MAX");
    expect(state, bus.flash[0x1800] == 0, "bus.flash[0x1800] == 0");
    kinetis_data_test_write_fccob(state, data, 0u, 0x07u);
    kinetis_data_test_set_flash_address(state, data, 0x2000u);
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x80u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    bus.flash[0x100u] = 0u;
    bus.flash[0x40000u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x08u, 0u, 2000u);
    expect(state, bus.flash[0x100u] == 0xffu, "single-block flash erases its lower half");
    expect(state, bus.flash[0x40000u] == 0xffu, "single-block flash erases its upper half");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN256CAP12);
    bus.flash[0u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x08u, 4u, 2000u);
    expect(state, bus.flash[0u] == 0xffu, "CAP flash accepts a longword-aligned block erase");
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x00u, 4u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u,
           "CAP flash accepts a longword-aligned block check");
    kinetis_data_destroy(data);
}

void kinetis_data_test_flash_command(TestState* state, KinetisData* data, uint8_t command,
                                     uint32_t address, uint32_t cycles) {
    kinetis_data_test_write_fccob(state, data, 0u, command);
    kinetis_data_test_set_flash_address(state, data, address);
    launch_flash(state, data, cycles);
}

void kinetis_data_test_flash_command_without_address(TestState* state, KinetisData* data,
                                                     uint8_t command, uint32_t cycles) {
    kinetis_data_test_write_fccob(state, data, 0u, command);
    launch_flash(state, data, cycles);
}

void kinetis_data_test_test_flash_commands_and_failures(TestState* state) {
    TestBus bus;
    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);

    bus.flash[0x100u] = 0u;
    bus.flash[0x7fffcu] = 0u;
    bus.flash[0x80000u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x08u, 0x100u, 2000u);
    expect(state, bus.flash[0x100u] == 0xffu, "bus.flash[0x100u] == 0xffu");
    expect(state, bus.flash[0x7fffcu] == 0xffu, "bus.flash[0x7fffcu] == 0xffu");
    expect(state, bus.flash[0x80000u] == 0u, "bus.flash[0x80000u] == 0u");

    bus.flash[0u] = 0u;
    bus.flash[0xfffffu] = 0u;
    kinetis_data_test_flash_command(state, data, 0x44u, 0u, 2000u);
    expect(state, bus.flash[0u] == 0xffu, "bus.flash[0u] == 0xffu");
    expect(state, bus.flash[0xfffffu] == 0xffu, "bus.flash[0xfffffu] == 0xffu");

    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_write_fccob(state, data, 5u, 1u);
    kinetis_data_test_write_fccob(state, data, 6u, 0u);
    kinetis_data_test_flash_command(state, data, 0x01u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u");
    bus.flash[0x1000u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x01u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) != 0u");
    kinetis_data_test_write_value(state, data, FTFA, 1, 1u);
    memset(bus.flash, 0xff, sizeof(bus.flash));
    kinetis_data_test_write_fccob(state, data, 1u, 0u);
    kinetis_data_test_flash_command(state, data, 0x40u, 0u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u");

    kinetis_data_test_store(bus.flash, 0x2000u, 4, 0x12345678u);
    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_write_fccob(state, data, 8u, 0x78u);
    kinetis_data_test_write_fccob(state, data, 9u, 0x56u);
    kinetis_data_test_write_fccob(state, data, 10u, 0x34u);
    kinetis_data_test_write_fccob(state, data, 11u, 0x12u);
    kinetis_data_test_flash_command(state, data, 0x02u, 0x2000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u");
    kinetis_data_test_write_fccob(state, data, 8u, 0x87u);
    kinetis_data_test_write_fccob(state, data, 9u, 0x65u);
    kinetis_data_test_write_fccob(state, data, 10u, 0x43u);
    kinetis_data_test_write_fccob(state, data, 11u, 0x21u);
    kinetis_data_test_flash_command(state, data, 0x02u, 0x2000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) != 0u");
    kinetis_data_test_write_value(state, data, FTFA, 1, 1u);

    bus.fail_read = true;
    for (uint8_t index = 4u; index < 12u; index++)
        kinetis_data_test_write_fccob(state, data, index, (uint8_t)(index * 17u));
    kinetis_data_test_flash_command(state, data, 0x07u, 0x3000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u");
    bus.fail_read = false;
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x20u);
    bus.fail_write = true;
    kinetis_data_test_flash_command(state, data, 0x07u, 0x3000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u");
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x20u);
    kinetis_data_test_flash_command(state, data, 0x09u, 0x4000u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u");
    bus.fail_write = false;
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x20u);

    kinetis_data_test_write_fccob(state, data, 4u, 1u);
    kinetis_data_test_flash_command(state, data, 0x03u, 8u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x30u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x30u) == 0u");
    kinetis_data_test_flash_command(state, data, 0x45u, 0u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u");
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x20u);
    kinetis_data_test_write_fccob(state, data, 0u, 0x43u);
    kinetis_data_test_write_fccob(state, data, 1u, 1u);
    for (uint8_t index = 4u; index < 12u; index++)
        kinetis_data_test_write_fccob(state, data, index, (uint8_t)(index * 19u));
    launch_flash(state, data, 40u);
    launch_flash(state, data, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) != 0u");
    kinetis_data_test_write_value(state, data, FTFA, 1, 0x20u);
    kinetis_data_test_write_value(state, data, FTFA + 1u, 1, 0x80u);
    kinetis_data_test_flash_command(state, data, 0x03u, 8u, 40u);
    expect(state, bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA],
           "bus.interrupt[KINETIS_DATA_INTERRUPT_FTFA]");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN25612);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_write_fccob(state, data, 5u, 1u);
    kinetis_data_test_write_fccob(state, data, 6u, 0u);
    bus.flash[0x1008u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x01u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 1u) == 0u");
    bus.flash[0x1008u] = 0xffu;
    kinetis_data_test_write_fccob(state, data, 5u, 2u);
    kinetis_data_test_flash_command(state, data, 0x01u, 0x7f8u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) == 0u");
    bus.flash[0x800u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x09u, 0x808u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1) & 0x20u) == 0u");
    expect(state, bus.flash[0x800u] == 0xffu, "bus.flash[0x800u] == 0xffu");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create_without_program(state, &bus, KINETIS_PROFILE_MK22FN51212);
    kinetis_data_test_write_fccob(state, data, 0u, 0x06u);
    kinetis_data_test_set_flash_address(state, data, 0x1000u);
    kinetis_data_test_write_fccob(state, data, 4u, 0x78u);
    kinetis_data_test_write_fccob(state, data, 5u, 0x56u);
    kinetis_data_test_write_fccob(state, data, 6u, 0x34u);
    kinetis_data_test_write_fccob(state, data, 7u, 0x12u);
    launch_flash(state, data, 40u);
    expect(state, kinetis_data_test_load(bus.flash, 0x1000u, 4) == 0x12345678u,
           "kinetis_data_test_load(bus.flash, 0x1000u, 4) == 0x12345678u");
    kinetis_data_destroy(data);
}

void kinetis_data_test_clear_flash_status(TestState* state, KinetisData* data) {
    kinetis_data_test_write_value(state, data, FTFA, 1u, 0x70u);
}

void kinetis_data_test_set_flash_data(TestState* state, KinetisData* data, const uint8_t* bytes,
                                      uint8_t length) {
    for (uint8_t index = 0u; index < length; index++)
        kinetis_data_test_write_fccob(state, data, (uint8_t)(4u + index), bytes[index]);
}

void kinetis_data_test_test_flash_command_semantics(TestState* state) {
    TestBus bus;
    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    KinetisData* data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN51212);
    const uint8_t longword[4] = {0x11u, 0x22u, 0x33u, 0x44u};
    kinetis_data_test_set_flash_data(state, data, longword, sizeof(longword));
    kinetis_data_test_flash_command(state, data, 0x06u, 0x2000u, 40u);
    expect(state, kinetis_data_test_load(bus.flash, 0x2000u, 4u) == 0x44332211u,
           "kinetis_data_test_load(bus.flash, 0x2000u, 4u) == 0x44332211u");
    kinetis_data_test_flash_command(state, data, 0x06u, 0x2000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command(state, data, 0x06u, 0x2001u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command(state, data, 0x07u, 0x3000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);

    kinetis_data_test_write_fccob(state, data, 1u, 0u);
    kinetis_data_test_set_flash_data(state, data, longword, sizeof(longword));
    kinetis_data_test_flash_command(state, data, 0x43u, 0u, 40u);
    kinetis_data_test_flash_command(state, data, 0x41u, 0u, 40u);
    for (uint8_t index = 0u; index < sizeof(longword); index++)
        expect(
            state,
            kinetis_data_test_read_fccob(state, data, (uint8_t)(4u + index)) == longword[index],
            "kinetis_data_test_read_fccob(state, data, (uint8_t)(4u + index)) == longword[index]");
    kinetis_data_test_flash_command(state, data, 0x43u, 0u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 8u, 1u);
    kinetis_data_test_flash_command(state, data, 0x03u, 0u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 4u) == 0x01u,
           "kinetis_data_test_read_fccob(state, data, 4u) == 0x01u");
    expect(state, kinetis_data_test_read_fccob(state, data, 6u) == 0x46u,
           "kinetis_data_test_read_fccob(state, data, 6u) == 0x46u");
    expect(state, kinetis_data_test_read_fccob(state, data, 7u) == 0x54u,
           "kinetis_data_test_read_fccob(state, data, 7u) == 0x54u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);
    uint8_t erased_configuration[16];
    memset(erased_configuration, 0xff, sizeof(erased_configuration));
    expect(state,
           kinetis_data_set_flash_configuration(data, erased_configuration,
                                                sizeof(erased_configuration)),
           "kinetis_data_set_flash_configuration(data, erased_configuration, "
           "sizeof(erased_configuration))");
    kinetis_data_reset(data);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x00u, 0x00000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) == 0u");
    bus.flash[0x7fffcu] = 0u;
    kinetis_data_test_flash_command(state, data, 0x00u, 0x00000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_write_fccob(state, data, 5u, 2u);
    kinetis_data_test_write_fccob(state, data, 6u, 0u);
    bus.flash[0x1010u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x01u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_write_fccob(state, data, 5u, 0u);
    kinetis_data_test_flash_command(state, data, 0x01u, 0x1000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);

    for (uint8_t index = 0u; index < 16u; index++)
        kinetis_data_test_write_value(state, data, 0x14000000u + index, 1u,
                                      (uint8_t)(0x80u + index));
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_write_fccob(state, data, 5u, 1u);
    kinetis_data_test_flash_command(state, data, 0x0bu, 0x3000u, 40u);
    for (uint8_t index = 0u; index < 16u; index++)
        expect(state, bus.flash[0x3000u + index] == (uint8_t)(0x80u + index),
               "bus.flash[0x3000u + index] == (uint8_t)(0x80u + index)");
    kinetis_data_test_flash_command(state, data, 0x0bu, 0x3000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_write_fccob(state, data, 5u, 0u);
    kinetis_data_test_flash_command(state, data, 0x0bu, 0x4000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 8u);
    kinetis_data_test_flash_command(state, data, 0x46u, 0x4000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 5u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 5u) == 0u");
    expect(state, kinetis_data_test_read_fccob(state, data, 6u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 6u) == 0u");
    expect(state, kinetis_data_test_read_fccob(state, data, 7u) == 0u,
           "kinetis_data_test_read_fccob(state, data, 7u) == 0u");
    kinetis_data_test_write_fccob(state, data, 1u, 0xffu);
    kinetis_data_test_flash_command_without_address(state, data, 0x81u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "MK22FN1M0 rejects Set FlexRAM Function");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    uint8_t flex_configuration[16];
    memset(flex_configuration, 0xff, sizeof(flex_configuration));
    expect(
        state,
        kinetis_data_set_flash_configuration(data, flex_configuration, sizeof(flex_configuration)),
        "kinetis_data_set_flash_configuration(data, flex_configuration, "
        "sizeof(flex_configuration))");
    kinetis_data_reset(data);
    kinetis_data_test_write_fccob(state, data, 3u, 0u);
    kinetis_data_test_write_fccob(state, data, 4u, 2u);
    kinetis_data_test_write_fccob(state, data, 5u, 3u);
    kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 1u,
           "(kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 1u");
    expect(state, kinetis_data_test_read_value(state, data, 0x10000000u, 4u) == UINT32_MAX,
           "kinetis_data_test_read_value(state, data, 0x10000000u, 4u) == UINT32_MAX");
    kinetis_data_test_write_value(state, data, 0x14000000u, 4u, 0x5aa5c33cu);
    kinetis_data_test_write_fccob(state, data, 1u, 0xffu);
    kinetis_data_test_flash_command_without_address(state, data, 0x81u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u");
    for (uint8_t index = 0u; index < 16u; index++)
        kinetis_data_test_write_value(state, data, 0x14000000u + index, 1u, (uint8_t)index);
    bus.fail_write = true;
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_write_fccob(state, data, 5u, 1u);
    kinetis_data_test_flash_command(state, data, 0x0bu, 0x5000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    bus.fail_write = false;
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_reset(data);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 1u,
           "(kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 1u");
    expect(state, kinetis_data_test_read_value(state, data, 0x14000000u, 4u) == 0x5aa5c33cu,
           "EEPROM data is restored after reset");
    kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 1u, 0xffu);
    kinetis_data_test_flash_command_without_address(state, data, 0x81u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 2u,
           "(kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 2u");
    expect(state, kinetis_data_test_read_value(state, data, 0x14000000u, 4u) == UINT32_MAX,
           "kinetis_data_test_read_value(state, data, 0x14000000u, 4u) == UINT32_MAX");
    kinetis_data_test_write_fccob(state, data, 1u, 0u);
    kinetis_data_test_flash_command_without_address(state, data, 0x81u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 1u,
           "(kinetis_data_test_read_value(state, data, FTFA + 1u, 1u) & 3u) == 1u");
    expect(state, kinetis_data_test_read_value(state, data, 0x14000000u, 4u) == 0x5aa5c33cu,
           "EEPROM data survives FlexRAM mode changes");
    kinetis_data_test_write_fccob(state, data, 1u, 1u);
    kinetis_data_test_flash_command_without_address(state, data, 0x81u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    const uint8_t phrase[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    kinetis_data_test_set_flash_data(state, data, phrase, sizeof(phrase));
    kinetis_data_test_flash_command(state, data, 0x07u, 0x800000u, 40u);
    expect(state, kinetis_data_test_read_value(state, data, 0x10000000u, 4u) == 0x04030201u,
           "kinetis_data_test_read_value(state, data, 0x10000000u, 4u) == 0x04030201u");
    expect(state, kinetis_data_test_read_value(state, data, 0x10000004u, 4u) == 0x08070605u,
           "kinetis_data_test_read_value(state, data, 0x10000004u, 4u) == 0x08070605u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    kinetis_data_test_write_fccob(state, data, 3u, 0u);
    kinetis_data_test_write_fccob(state, data, 4u, 9u);
    kinetis_data_test_write_fccob(state, data, 5u, 3u);
    kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
    uint32_t eeprom_value = 0u;
    expect(state,
           kinetis_data_write(data, 0x1400001fu, 1u, 0x96u) &&
               kinetis_data_read(data, 0x1400001fu, 1u, &eeprom_value) && eeprom_value == 0x96u,
           "configured EEPROM range is accessible");
    expect(state,
           !kinetis_data_write(data, 0x14000020u, 1u, 0u) &&
               !kinetis_data_read(data, 0x14000020u, 1u, &eeprom_value),
           "FlexRAM outside the configured EEPROM range is inaccessible");
    KinetisData* copy = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    expect(state, kinetis_data_copy(copy, data), "EEPROM state is copied");
    kinetis_data_reset(copy);
    expect(state, kinetis_data_test_read_value(state, copy, 0x1400001fu, 1u) == 0x96u,
           "copied EEPROM data survives reset");
    kinetis_data_destroy(copy);
    kinetis_data_reset(data);
    expect(state, kinetis_data_test_read_value(state, data, 0x1400001fu, 1u) == 0x96u,
           "small EEPROM data survives reset");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FX51212);
    expect(
        state,
        kinetis_data_set_flash_configuration(data, flex_configuration, sizeof(flex_configuration)),
        "kinetis_data_set_flash_configuration(data, flex_configuration, "
        "sizeof(flex_configuration))");
    kinetis_data_reset(data);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x03u, 0u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 4u) == 0xffu,
           "kinetis_data_test_read_fccob(state, data, 4u) == 0xffu");
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x00u, 0x800000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) == 0u");
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x03u, 0x800000u, 40u);
    expect(state, kinetis_data_test_read_fccob(state, data, 4u) == 0xffu,
           "kinetis_data_test_read_fccob(state, data, 4u) == 0xffu");
    kinetis_data_test_write_fccob(state, data, 3u, 0u);
    kinetis_data_test_write_fccob(state, data, 4u, 0x02u);
    kinetis_data_test_write_fccob(state, data, 5u, 0x04u);
    kinetis_data_test_flash_command_without_address(state, data, 0x80u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) == 0u");
    kinetis_data_test_write_fccob(state, data, 1u, 0u);
    kinetis_data_test_flash_command_without_address(state, data, 0x40u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "Read 1s All Blocks checks data flash IFR");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_fccob(state, data, 4u, 0u);
    kinetis_data_test_flash_command(state, data, 0x00u, 0x800000u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_write_value(state, data, 0x10000000u, 1u, 0u);
    kinetis_data_test_write_fccob(state, data, 1u, 0u);
    kinetis_data_test_flash_command_without_address(state, data, 0x40u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 1u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_flash_command(state, data, 0x08u, 0x800000u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    expect(state, kinetis_data_test_read_value(state, data, 0x10000000u, 1u) == 0u,
           "kinetis_data_test_read_value(state, data, 0x10000000u, 1u) == 0u");
    kinetis_data_test_clear_flash_status(state, data);
    bus.flash[0u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x08u, 0u, 2000u);
    expect(state, bus.flash[0u] == 0xffu, "bus.flash[0u] == 0xffu");
    kinetis_data_test_write_value(state, data, 0x10000000u, 1u, 0u);
    kinetis_data_test_flash_command(state, data, 0x44u, 0u, 2000u);
    expect(state, kinetis_data_test_read_value(state, data, 0x10000000u, 1u) == 0xffu,
           "kinetis_data_test_read_value(state, data, 0x10000000u, 1u) == 0xffu");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN51212);
    bus.flash[0u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x08u, 0u, 2000u);
    expect(state, bus.flash[0u] == 0xffu, "bus.flash[0u] == 0xffu");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN51212);
    kinetis_data_test_write_fccob(state, data, 1u, 0x10u);
    const uint8_t once_phrase[8] = {9u, 8u, 7u, 6u, 5u, 4u, 3u, 2u};
    kinetis_data_test_set_flash_data(state, data, once_phrase, sizeof(once_phrase));
    kinetis_data_test_flash_command_without_address(state, data, 0x43u, 40u);
    kinetis_data_test_write_fccob(state, data, 1u, 0x10u);
    kinetis_data_test_flash_command_without_address(state, data, 0x41u, 40u);
    for (uint8_t index = 0u; index < sizeof(once_phrase); index++)
        expect(state,
               kinetis_data_test_read_fccob(state, data, (uint8_t)(4u + index)) ==
                   once_phrase[index],
               "kinetis_data_test_read_fccob(state, data, (uint8_t)(4u + index)) == "
               "once_phrase[index]");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN1M012);
    uint8_t protected_configuration[16];
    memset(protected_configuration, 0xff, sizeof(protected_configuration));
    protected_configuration[8] = 0xfdu;
    protected_configuration[12] = 0xfeu;
    expect(state,
           kinetis_data_set_flash_configuration(data, protected_configuration,
                                                sizeof(protected_configuration)),
           "kinetis_data_set_flash_configuration(data, protected_configuration, "
           "sizeof(protected_configuration))");
    kinetis_data_reset(data);
    bus.flash[0u] = 0u;
    kinetis_data_test_flash_command(state, data, 0x08u, 0u, 2000u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x10u) != 0u");
    expect(state, bus.flash[0u] == 0u, "bus.flash[0u] == 0u");
    kinetis_data_destroy(data);

    memset(&bus, 0, sizeof(bus));
    memset(bus.flash, 0xff, sizeof(bus.flash));
    data = kinetis_data_test_create(state, &bus, KINETIS_PROFILE_MK22FN51212);
    uint8_t configuration[16];
    for (uint8_t index = 0u; index < sizeof(configuration); index++)
        configuration[index] = (uint8_t)(index + 1u);
    configuration[8] = 0xffu;
    configuration[9] = 0xffu;
    configuration[10] = 0xffu;
    configuration[11] = 0xffu;
    configuration[12] = 0x80u;
    expect(state, kinetis_data_set_flash_configuration(data, configuration, sizeof(configuration)),
           "kinetis_data_set_flash_configuration(data, configuration, sizeof(configuration))");
    kinetis_data_reset(data);
    kinetis_data_test_set_flash_data(state, data, configuration, 8u);
    kinetis_data_test_flash_command(state, data, 0x45u, 0u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA + 2u, 1u) & 3u) == 2u,
           "(kinetis_data_test_read_value(state, data, FTFA + 2u, 1u) & 3u) == 2u");
    kinetis_data_reset(data);
    uint8_t constant_key[8];
    memset(constant_key, 0xff, sizeof(constant_key));
    kinetis_data_test_set_flash_data(state, data, constant_key, sizeof(constant_key));
    kinetis_data_test_flash_command(state, data, 0x45u, 0u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_test_clear_flash_status(state, data);
    kinetis_data_test_set_flash_data(state, data, configuration, 8u);
    kinetis_data_test_flash_command(state, data, 0x45u, 0u, 40u);
    expect(state, (kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u,
           "(kinetis_data_test_read_value(state, data, FTFA, 1u) & 0x20u) != 0u");
    kinetis_data_destroy(data);
}
