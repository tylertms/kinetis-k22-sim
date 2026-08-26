#include "device/kinetis/communication/sdhc.h"

#include <string.h>

#include "test.h"

enum {
    SDHC_BASE = 0x400b1000u,
};

typedef struct {
    uint8_t memory[4096];
    uint32_t random;
    uint32_t reads;
    uint32_t writes;
    uint64_t fingerprint;
} SdhcCensus;

static bool bus_read(void* context, uint32_t address, uint8_t size, uint32_t* read_value) {
    SdhcCensus* census = context;
    if (read_value == NULL || address > sizeof(census->memory) ||
        size > sizeof(census->memory) - address)
        return false;
    *read_value = 0u;
    memcpy(read_value, census->memory + address, size);
    return true;
}

static bool bus_write(void* context, uint32_t address, uint8_t size, uint32_t write_value) {
    SdhcCensus* census = context;
    if (address > sizeof(census->memory) || size > sizeof(census->memory) - address)
        return false;
    memcpy(census->memory + address, &write_value, size);
    return true;
}

static uint32_t next_random(SdhcCensus* census) {
    uint32_t random_value = census->random;
    random_value ^= random_value << 13u;
    random_value ^= random_value >> 17u;
    random_value ^= random_value << 5u;
    census->random = random_value;
    return random_value;
}

static void mix(SdhcCensus* census, uint32_t mix_value) {
    census->fingerprint = (census->fingerprint ^ mix_value) * UINT64_C(1099511628211);
}

static void randomize_state(KinetisSdhc* sdhc, SdhcCensus* census) {
    for (uint8_t register_index = 0u; register_index < 64u; register_index++) {
        sdhc->registers[register_index] = next_random(census);
    }
    const uint32_t initial_random = next_random(census);
    const uint32_t secondary_random = next_random(census);
    sdhc->transfer_address = initial_random % (sdhc->card_size + 1u);
    sdhc->transfer_remaining = secondary_random % (sdhc->card_size - sdhc->transfer_address + 1u);
    sdhc->block_length = (initial_random >> 8u) & 0x1fffu;
    sdhc->relative_address = secondary_random;
    sdhc->present = (initial_random & 1u) != 0u;
    sdhc->write_protected = (initial_random & 2u) != 0u;
    sdhc->clock_enabled = (initial_random & 4u) != 0u;
    sdhc->application_command = (initial_random & 8u) != 0u;
    sdhc->selected = (initial_random & 16u) != 0u;
    sdhc->transfer_read = (initial_random & 32u) != 0u;
    sdhc->transfer_multiple = (initial_random & 64u) != 0u;
}

int main(void) {
    TestState state = {0u, 0u, 0u};
    SdhcCensus census = {{0u}, UINT32_C(0xa4093822), 0u, 0u, UINT64_C(14695981039346656037)};
    KinetisSdhc sdhc;
    expect(&state, kinetis_sdhc_init(&sdhc, (KinetisSdhcBus){&census, bus_read, bus_write}),
           "initialize SDHC state census");
    uint8_t card[2048] = {0u};
    expect(&state, kinetis_sdhc_state_insert(&sdhc, card, sizeof(card), false),
           "insert SDHC state census card");
    for (uint32_t iteration_index = 0u; iteration_index < 100000u; iteration_index++) {
        randomize_state(&sdhc, &census);
        const uint32_t random_value = next_random(&census);
        const uint32_t register_address = SDHC_BASE + ((random_value >> 8u) % 66u) * 4u;
        const uint8_t access_size = (uint8_t)(random_value % 6u);
        uint32_t read_value = UINT32_MAX;
        const bool read_succeeded =
            kinetis_sdhc_read(&sdhc, register_address, access_size, &read_value);
        const bool write_succeeded = kinetis_sdhc_write(&sdhc, register_address, access_size,
                                                        random_value ^ UINT32_C(0x5aa5a55a));
        census.reads += read_succeeded;
        census.writes += write_succeeded;
        mix(&census, read_succeeded | (write_succeeded << 1u));
        mix(&census, read_value);
        mix(&census, sdhc.registers[(random_value >> 16u) % 64u]);
        mix(&census, kinetis_sdhc_irq(&sdhc));
    }
    expect(&state,
           census.reads == 2864u && census.writes == 1723u &&
               census.fingerprint == UINT64_C(8440844145325666953),
           "SDHC state census matches");
    kinetis_sdhc_destroy(&sdhc);
    return test_finish(&state);
}
