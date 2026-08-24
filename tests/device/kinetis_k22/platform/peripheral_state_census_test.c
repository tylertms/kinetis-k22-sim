#include "device/kinetis_k22/internal.h"

#include <inttypes.h>
#include <stdio.h>

#include "test.h"

typedef struct {
    uint32_t random_state;
    uint32_t accepted_accesses;
    uint32_t rejected_accesses;
    uint64_t fingerprint;
} PeripheralStateCensus;

static uint32_t next_random(PeripheralStateCensus* census) {
    uint32_t random_value = census->random_state;
    random_value ^= random_value << 13u;
    random_value ^= random_value >> 17u;
    random_value ^= random_value << 5u;
    census->random_state = random_value;
    return random_value;
}

static void mix_fingerprint(PeripheralStateCensus* census, uint32_t input_value) {
    census->fingerprint = (census->fingerprint ^ input_value) * UINT64_C(1099511628211);
}

static void randomize_state(KinetisK22* device, PeripheralStateCensus* census,
                            uint32_t random_value) {
    const uint32_t next_value = next_random(census);
    kinetis_k22_internal_raw_store(device, K22_AIPS0 + 0x20u + ((random_value >> 20u) & 0x1cu), 4u,
                                   random_value);
    kinetis_k22_internal_raw_store(device, K22_AIPS1 + 0x20u + ((next_value >> 20u) & 0x1cu), 4u,
                                   next_value);

    const uint32_t axbs_base = K22_AXBS + ((random_value >> 8u) % 5u) * 0x100u;
    kinetis_k22_internal_raw_store(device, axbs_base + 0x10u, 4u, next_value);

    for (uint8_t register_offset = 0u; register_offset < 12u; register_offset++) {
        kinetis_k22_internal_raw_store(device, K22_CMT + register_offset, 1u,
                                       (uint8_t)next_random(census));
    }

    device->cmt_cycles = random_value & 0x3ffu;
    device->cmt_period_ticks = device->cmt_cycles + (next_value & 0x3ffu) + 1u;
    device->cmt_bus_remainder = random_value & 0xffffu;
    device->cmt_mark_ticks = next_value & 0x3ffu;
    device->cmt_carrier_high_ticks = random_value & 0xffu;
    device->cmt_carrier_period_ticks = (next_value & 0xffu) + 1u;
    device->cmt_carrier_offset_ticks = random_value & 0x3ffu;
    device->cmt_output_delay_ticks = next_value & 0x3ffu;
    device->cmt_eoc_read = (random_value & 1u) != 0u;
    device->cmt_running = (random_value & 2u) != 0u;
    device->cmt_stop_pending = (random_value & 4u) != 0u;
    device->cmt_fsk_secondary = (random_value & 8u) != 0u;
    device->cmt_extended_space = (random_value & 16u) != 0u;
    device->cmt_dma_pending = (random_value & 32u) != 0u;

    device->timing.sim_scgc3 = random_value;
    device->timing.sim_scgc4 = next_value;
    device->timing.sim_scgc5 = random_value ^ next_value;
    device->timing.sim_scgc6 = ~random_value;
    device->timing.sim_scgc7 = ~next_value;
}

int main(void) {
    TestState state = {0u, 0u, 0u};
    PeripheralStateCensus census = {UINT32_C(0xbb67ae85), 0u, 0u, UINT64_C(14695981039346656037)};
    KinetisK22* device =
        kinetis_k22_create(kinetis_k22_configuration(KINETIS_K22_PROFILE_MK22FN1M012));
    expect(&state, device != NULL, "create peripheral state census simulator");
    if (device == NULL)
        return test_finish(&state);
    for (uint32_t iteration = 0u; iteration < 100000u; iteration++) {
        const uint32_t random_value = next_random(&census);
        randomize_state(device, &census, random_value);
        const K22RegisterDescriptor* descriptor =
            &device->manifest->registers[(random_value >> 8u) % device->manifest->register_count];
        K22PeripheralLocation location;
        const bool resolved = k22_profile_resolve_peripheral(
            device->profile, descriptor->address, (uint8_t)(descriptor->width / 8u), &location);
        if (!resolved) {
            expect(&state, false, "resolve peripheral state census register");
            continue;
        }
        const uint8_t access_size = (uint8_t[]){1u, 2u, 4u}[random_value % 3u];
        const CortexM4Access access = (CortexM4Access)((random_value >> 20u) % 5u);
        const bool write_access = (random_value & 1u) != 0u;
        const uint32_t peripheral_address =
            K22_PERIPHERAL_BASE + ((random_value >> 4u) % K22_PERIPHERAL_SIZE);
        const uint32_t axbs_address = K22_AXBS + ((random_value >> 12u) % 0x520u);
        const bool aips_access_allowed = kinetis_k22_internal_aips_access_allowed(
            device, peripheral_address, access, write_access);
        const bool axbs_write_allowed =
            kinetis_k22_internal_axbs_write_allowed(device, axbs_address);
        const bool peripheral_clock_enabled = kinetis_k22_internal_peripheral_clock_enabled(
            device, (K22PeripheralId)((random_value >> 24u) % (K22_PERIPHERAL_COUNT + 2u)));
        const bool debug_clock_enabled = kinetis_k22_internal_enable_debug_clock(
            device, (K22PeripheralId)((random_value >> 16u) % (K22_PERIPHERAL_COUNT + 2u)));
        uint32_t read_value = UINT32_MAX;
        const bool read_succeeded = kinetis_k22_internal_semantic_read(
            device, location.id, descriptor->address, access_size, &read_value);
        const bool write_succeeded =
            kinetis_k22_internal_semantic_write(device, location.id, descriptor->address,
                                                access_size, random_value ^ UINT32_C(0xa5a55a5a));
        const bool serial_endpoint_available = kinetis_k22_internal_serial_endpoint_available(
            device, (KinetisK22SerialEndpoint)((random_value >> 12u) %
                                               (KINETIS_K22_SERIAL_ENDPOINT_COUNT + 2u)));
        kinetis_k22_internal_cmt_advance(device, (random_value & 31u) + 1u);
        if ((iteration & 15u) == 0u) {
            kinetis_k22_sync_clock_gates(device);
            kinetis_k22_refresh_signals(device);
        }
        const uint32_t access_results = aips_access_allowed | (axbs_write_allowed << 1u) |
                                        (peripheral_clock_enabled << 2u) |
                                        (debug_clock_enabled << 3u) | (read_succeeded << 4u) |
                                        (write_succeeded << 5u) | (serial_endpoint_available << 6u);
        const uint32_t accepted_accesses =
            aips_access_allowed + axbs_write_allowed + peripheral_clock_enabled +
            debug_clock_enabled + read_succeeded + write_succeeded + serial_endpoint_available;
        census.accepted_accesses += accepted_accesses;
        census.rejected_accesses += 7u - accepted_accesses;
        mix_fingerprint(&census, descriptor->address);
        mix_fingerprint(&census, read_value);
        mix_fingerprint(&census, access_results);
        mix_fingerprint(&census, (uint32_t)device->cmt_cycles ^ device->timing.sim_scgc6);
    }
    const bool census_matches = census.accepted_accesses == 411294u &&
                                census.rejected_accesses == 288706u &&
                                census.fingerprint == UINT64_C(585895875121782539);
    if (!census_matches) {
        fprintf(stderr,
                "[census] accepted=%" PRIu32 " rejected=%" PRIu32 " fingerprint=%" PRIu64 "\n",
                census.accepted_accesses, census.rejected_accesses, census.fingerprint);
    }
    expect(&state, census_matches, "peripheral state census matches");
    kinetis_k22_destroy(device);
    return test_finish(&state);
}
