#include "device/kinetis/memory/internal.h"

#include "test.h"

static void set_command(KinetisData* data, uint8_t command, uint32_t address) {
    data->flash[7u] = command;
    data->flash[6u] = (uint8_t)(address >> 16u);
    data->flash[5u] = (uint8_t)(address >> 8u);
    data->flash[4u] = (uint8_t)address;
}

int main(void) {
    TestState state = {0u, 0u, 0u};
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN51212);
    expect(&state, profile != NULL, "find flash fault test profile");
    KinetisData* data = profile == NULL ? NULL : kinetis_data_create(profile, (KinetisDataBus){0});
    expect(&state, data != NULL, "create flash controller without a read callback");
    if (data != NULL) {
        set_command(data, 0x02u, UINT32_C(0x2000));
        data->flash[11u] = 1u;
        expect(&state, kinetis_data_internal_flash_write(data, FLASH_BASE, 1u, 0x80u),
               "launch flash verify command");
        expect(&state, (data->flash[0] & 1u) != 0u, "flash verify reports a memory read failure");
        kinetis_data_destroy(data);
    }
    return test_finish(&state);
}
