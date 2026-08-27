#include "device/kinetis/memory/data/internal.h"

int main(void) {
    TestState state = {0};

    kinetis_data_test_test_flash_command_census(&state);
    kinetis_data_test_test_flash_state_census(&state);
    kinetis_data_test_test_state_census(&state);
    return test_finish(&state);
}
