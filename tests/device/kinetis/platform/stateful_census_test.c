#include "kinetis.h"

#include <inttypes.h>
#include <stdio.h>

#include "device/kinetis/internal.h"
#include "test.h"

typedef struct {
    uint32_t random_state;
    uint32_t read_successes;
    uint32_t write_successes;
    uint32_t signal_count;
    uint64_t fingerprint;
} StatefulCensus;

static uint32_t next_random(StatefulCensus* census) {
    uint32_t random_value = census->random_state;
    random_value ^= random_value << 13u;
    random_value ^= random_value >> 17u;
    random_value ^= random_value << 5u;
    census->random_state = random_value;
    return random_value;
}

static void mix_fingerprint(StatefulCensus* census, uint32_t input_value) {
    census->fingerprint = (census->fingerprint ^ input_value) * UINT64_C(1099511628211);
}

static void exercise_signal(Kinetis* device, StatefulCensus* census, uint32_t random_value) {
    bool operation_succeeded = false;
    bool ftm_output_high = false;
    uint16_t dac_output = 0u;
    switch (random_value % 16u) {
    case 0u:
        operation_succeeded = kinetis_set_adc_input(
            device, (uint8_t)(random_value >> 8u) % 3u, (KinetisAdcMux)((random_value >> 24u) % 3u),
            (uint8_t)(random_value >> 12u) % 32u, (uint16_t)random_value);
        break;
    case 1u:
        operation_succeeded =
            kinetis_set_cmp_input(device, (uint8_t)(random_value >> 8u) % 4u,
                                  (uint8_t)(random_value >> 12u) % 9u, (uint8_t)random_value);
        break;
    case 2u:
        operation_succeeded = kinetis_set_lptmr_input(device, (uint8_t)(random_value >> 8u) % 3u,
                                                      (random_value & 1u) != 0u);
        break;
    case 3u:
        operation_succeeded = kinetis_set_llwu_pin(device, (uint8_t)(random_value >> 8u) % 18u,
                                                   (random_value & 1u) != 0u);
        break;
    case 4u:
        operation_succeeded =
            kinetis_trigger_llwu_module(device, (uint8_t)(random_value >> 8u) % 10u);
        break;
    case 5u:
        operation_succeeded =
            kinetis_set_ftm_input(device, (uint8_t)(random_value >> 8u) % 6u,
                                  (uint8_t)(random_value >> 12u) % 10u, (random_value & 1u) != 0u);
        break;
    case 6u:
        operation_succeeded =
            kinetis_set_ftm_fault(device, (uint8_t)(random_value >> 8u) % 6u,
                                  (uint8_t)(random_value >> 12u) % 6u, (random_value & 1u) != 0u);
        break;
    case 7u:
        operation_succeeded = kinetis_trigger_ftm_hardware(
            device, (uint8_t)(random_value >> 8u) % 6u, (uint8_t)(random_value >> 12u) % 5u);
        break;
    case 8u:
        operation_succeeded =
            kinetis_get_ftm_output(device, (uint8_t)(random_value >> 8u) % 6u,
                                   (uint8_t)(random_value >> 12u) % 10u, &ftm_output_high);
        mix_fingerprint(census, ftm_output_high);
        break;
    case 9u:
        operation_succeeded =
            kinetis_get_dac_output(device, (uint8_t)(random_value >> 8u) % 3u, &dac_output);
        mix_fingerprint(census, dac_output);
        break;
    case 10u:
        operation_succeeded =
            kinetis_gpio_drive(device, (uint8_t)(random_value >> 8u) % 7u,
                               (uint8_t)(random_value >> 12u) % 34u, (random_value & 1u) != 0u);
        break;
    case 11u:
        operation_succeeded = kinetis_gpio_release(device, (uint8_t)(random_value >> 8u) % 7u,
                                                   (uint8_t)(random_value >> 12u) % 34u);
        break;
    case 12u:
        operation_succeeded =
            kinetis_serial_receive(device, (KinetisSerialEndpoint)(random_value % 15u),
                                   (uint16_t)random_value, (uint8_t)(random_value >> 16u));
        break;
    case 13u:
        operation_succeeded =
            kinetis_set_usb_charger(device, (KinetisUsbCharger)(random_value % 6u));
        break;
    case 14u:
        operation_succeeded = kinetis_set_usb_pullup(device, (random_value & 1u) != 0u);
        break;
    default:
        operation_succeeded = kinetis_set_ewm_input(device, (random_value & 1u) != 0u);
        break;
    }
    census->signal_count += operation_succeeded;
    mix_fingerprint(census, operation_succeeded);
}

static void exercise_profile(TestState* state, StatefulCensus* census, KinetisProfile profile) {
    static const uint8_t access_sizes[] = {0u, 1u, 2u, 3u, 4u, 5u};
    KinetisConfiguration configuration = kinetis_configuration(profile);
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "create stateful census simulator");
    if (device == NULL)
        return;
    for (uint32_t iteration = 0u; iteration < 30000u; iteration++) {
        const uint32_t random_value = next_random(census);
        const KinetisRegisterDescriptor* descriptor =
            &device->manifest->registers[random_value % device->manifest->register_count];
        const uint32_t address = descriptor->address + ((random_value >> 25u) == 0u ? 1u : 0u);
        const uint8_t access_size =
            access_sizes[(random_value >> 16u) % (sizeof(access_sizes) / sizeof(access_sizes[0]))];
        const CortexM4Access access_type = (CortexM4Access)((random_value >> 20u) % 5u);
        const bool write_succeeded = kinetis_memory_write(device, address, access_size, access_type,
                                                          random_value ^ UINT32_C(0xa5a55a5a));
        uint32_t read_value = UINT32_MAX;
        const bool read_succeeded =
            kinetis_memory_read(device, address, access_size, access_type, &read_value);
        census->write_successes += write_succeeded;
        census->read_successes += read_succeeded;
        mix_fingerprint(census, address);
        mix_fingerprint(census, access_size);
        mix_fingerprint(census, write_succeeded);
        mix_fingerprint(census, read_succeeded);
        mix_fingerprint(census, read_value);
        exercise_signal(device, census, random_value);
        if ((iteration & 15u) == 0u) {
            kinetis_advance(device, (random_value & 31u) + 1u);
        }
        KinetisEvent event;
        for (uint8_t event_index = 0u; event_index < 4u && kinetis_next_event(device, &event);
             event_index++) {
            mix_fingerprint(census, event.type ^ event.source ^ event.value);
        }
    }
    kinetis_destroy(device);
}

int main(void) {
    TestState state = {0u, 0u, 0u};
    StatefulCensus census = {UINT32_C(0x6d2b79f5), 0u, 0u, 0u, UINT64_C(14695981039346656037)};
    for (KinetisProfile profile = KINETIS_PROFILE_MK22FN12810; profile < KINETIS_PROFILE_COUNT;
         profile++) {
        exercise_profile(&state, &census, profile);
    }
    const bool census_matches = census.read_successes == 40615u &&
                                census.write_successes == 41344u && census.signal_count == 94715u &&
                                census.fingerprint == UINT64_C(8544810778568488024);
    if (!census_matches) {
        fprintf(stderr,
                "[census] reads=%" PRIu32 " writes=%" PRIu32 " signals=%" PRIu32
                " fingerprint=%" PRIu64 "\n",
                census.read_successes, census.write_successes, census.signal_count,
                census.fingerprint);
    }
    expect(&state, census_matches, "stateful census matches");
    return test_finish(&state);
}
