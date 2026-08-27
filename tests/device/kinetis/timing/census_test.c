#include "device/kinetis/timing/support.h"

int main(void) {
    TestState state = {0};
    Observations observations = {0};
    KinetisTiming timing;
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN51212);

    expect(&state,
           kinetis_timing_init(&timing, profile, 8000000u, 32768u,
                               kinetis_timing_test_signals(&observations)),
           "initialize timing census");
    kinetis_timing_test_test_ftm_census(&state, &timing);
    kinetis_timing_test_test_state_census(&state, &timing);
    return test_finish(&state);
}
