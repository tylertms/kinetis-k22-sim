#include "device/kinetis/internal.h"

#include "allocation_failure.h"
#include "test.h"

typedef struct {
    uint32_t accepted_reads;
    uint32_t accepted_writes;
    uint64_t fingerprint;
} DeviceCensus;

static Kinetis* create_device(TestState* state) {
    KinetisConfiguration configuration = kinetis_default_configuration();
    configuration.profile = KINETIS_PROFILE_MK22FN1M012;
    configuration.package = KINETIS_PACKAGE_LQ_144_LQFP;
    configuration.flash_size = 4096u;
    configuration.sram_size = 65536u;
    Kinetis* device = kinetis_create(configuration);
    expect(state, device != NULL, "create device boundary simulator");
    return device;
}

static void mix(DeviceCensus* census, uint32_t input_value) {
    census->fingerprint = (census->fingerprint ^ input_value) * UINT64_C(1099511628211);
}

static void access_census(Kinetis* device, DeviceCensus* census) {
    static const uint32_t test_addresses[] = {
        0u,          0x3ffu,      0x400u,      0xfffu,      0x1fffffffu,
        0x20000000u, 0x20000001u, 0x20007fffu, 0x40000000u, 0x4001f000u,
        0x400fffffu, 0x42000000u, 0x43ffffffu, 0x60000000u, UINT32_MAX,
    };
    for (size_t address_index = 0u;
         address_index < sizeof(test_addresses) / sizeof(test_addresses[0]); address_index++) {
        for (uint8_t access_size = 0u; access_size <= 5u; access_size++) {
            for (uint8_t access_kind = CORTEX_M4_ACCESS_INSTRUCTION;
                 access_kind <= CORTEX_M4_ACCESS_DEBUG; access_kind++) {
                uint32_t read_value = UINT32_C(0xa5a55a5a);
                const bool read_succeeded =
                    kinetis_memory_read(device, test_addresses[address_index], access_size,
                                        (CortexM4Access)access_kind, &read_value);
                const bool write_succeeded =
                    kinetis_memory_write(device, test_addresses[address_index], access_size,
                                         (CortexM4Access)access_kind, UINT32_C(0x5aa5a55a));
                census->accepted_reads += read_succeeded;
                census->accepted_writes += write_succeeded;
                mix(census, test_addresses[address_index]);
                mix(census, access_size);
                mix(census, access_kind);
                mix(census, read_succeeded);
                mix(census, write_succeeded);
                mix(census, read_value);
            }
            uint32_t dma_read_value = 0u;
            const bool dma_read_succeeded = kinetis_dma_read(device, test_addresses[address_index],
                                                             access_size, &dma_read_value);
            const bool dma_write_succeeded = kinetis_dma_write(
                device, test_addresses[address_index], access_size, UINT32_C(0x12345678));
            census->accepted_reads += dma_read_succeeded;
            census->accepted_writes += dma_write_succeeded;
            mix(census, dma_read_succeeded);
            mix(census, dma_write_succeeded);
            mix(census, dma_read_value);
        }
    }
}

static void copy_guard_cases(TestState* state, Kinetis* destination, Kinetis* source) {
    const K22Profile* profile = source->profile;
    const K22PackageSelection* package = source->package;
    const size_t flash_size = source->configuration.flash_size;
    const size_t sram_size = source->configuration.sram_size;

    source->profile = k22_profile_get(K22_PROFILE_MK22FN51212);
    expect(state, !kinetis_copy(destination, source), "copy rejects a different profile");
    source->profile = profile;
    source->package = k22_package_select(source->profile, K22_PACKAGE_DC_121_XFBGA);
    expect(state, !kinetis_copy(destination, source), "copy rejects a different package");
    source->package = package;
    source->configuration.flash_size++;
    expect(state, !kinetis_copy(destination, source), "copy rejects a different flash size");
    source->configuration.flash_size = flash_size;
    source->configuration.sram_size++;
    expect(state, !kinetis_copy(destination, source), "copy rejects a different SRAM size");
    source->configuration.sram_size = sram_size;
    expect(state, kinetis_copy(destination, source), "compatible devices copy");
}

static void uninitialized_sram_cases(TestState* state, Kinetis* destination, Kinetis* source) {
    const uint32_t address = source->sram_base;
    uint32_t value = 0u;
    expect(state,
           kinetis_memory_read(source, address, 4u, CORTEX_M4_ACCESS_DATA, &value) &&
               kinetis_get_uninitialized_sram_read_count(source) == 4u &&
               kinetis_get_first_uninitialized_sram_read(source) == address,
           "data access reports each uninitialized SRAM byte");
    expect(state,
           kinetis_memory_read(source, address, 4u, CORTEX_M4_ACCESS_DEBUG, &value) &&
               kinetis_get_uninitialized_sram_read_count(source) == 4u,
           "debug access does not report uninitialized SRAM");

    kinetis_clear_uninitialized_sram_reads(source);
    kinetis_clear_uninitialized_sram_reads(NULL);
    expect(state,
           kinetis_get_uninitialized_sram_read_count(source) == 0u &&
               kinetis_get_first_uninitialized_sram_read(source) == UINT32_MAX,
           "uninitialized SRAM diagnostics clear");
    expect(state,
           kinetis_memory_write(source, address, 2u, CORTEX_M4_ACCESS_DATA, 0x1234u) &&
               kinetis_memory_read(source, address, 4u, CORTEX_M4_ACCESS_DATA, &value) &&
               kinetis_get_uninitialized_sram_read_count(source) == 2u &&
               kinetis_get_first_uninitialized_sram_read(source) == address + 2u,
           "SRAM writes initialize only the written bytes");
    expect(state,
           kinetis_copy(destination, source) &&
               kinetis_get_uninitialized_sram_read_count(destination) == 2u &&
               kinetis_get_first_uninitialized_sram_read(destination) == address + 2u,
           "device copy preserves SRAM initialization diagnostics");
    const uint32_t vectors[2] = {address + 0x1000u, 0x101u};
    expect(state,
           kinetis_load(source, 0u, vectors, sizeof(vectors)) && kinetis_reset(source) &&
               kinetis_get_uninitialized_sram_read_count(source) == 0u &&
               kinetis_get_first_uninitialized_sram_read(source) == UINT32_MAX,
           "cold reset clears SRAM initialization diagnostics");
}

static void flexbus_cases(TestState* state, Kinetis* device) {
    static const uint8_t window_data[] = {1u, 2u, 3u, 4u};
    uint8_t read_data[4] = {0u};
    expect(state,
           kinetis_flexbus_attach(device, 0x60000000u, window_data, sizeof(window_data), true),
           "attach a read-only FlexBus window");
    expect(state, kinetis_flexbus_read(device, 0u, read_data, sizeof(read_data)),
           "read the complete FlexBus window");
    expect(state, !kinetis_flexbus_read(device, sizeof(window_data) + 1u, read_data, 0u),
           "FlexBus read rejects an offset beyond the window");
    expect(state, !kinetis_flexbus_read(device, sizeof(window_data), read_data, 1u),
           "FlexBus read rejects a range beyond the window");
    kinetis_flexbus_detach(device);
}

static void register_state_cases(TestState* state, Kinetis* first, Kinetis* second) {
    uint32_t address = 0u;
    uint32_t first_value = 0u;
    uint32_t second_value = 0u;
    expect(state,
           !kinetis_register_state_equal(NULL, second, &address, &first_value, &second_value),
           "register comparison rejects a null device");
    expect(state, !kinetis_register_state_equal(first, second, NULL, &first_value, &second_value),
           "register comparison requires difference outputs");
    expect(state, kinetis_reset(first) && kinetis_reset(second),
           "reset devices for register comparison");
    expect(state,
           kinetis_register_state_equal(first, second, &address, &first_value, &second_value),
           "equal register state agrees");
    expect(
        state,
        kinetis_memory_write(first, 0x40048038u, 4u, CORTEX_M4_ACCESS_DEBUG, 0x200u) &&
            !kinetis_register_state_equal(first, second, &address, &first_value, &second_value) &&
            address == 0x40048038u && first_value == 0x200u && second_value == 0u,
        "register comparison reports a configured-state difference");
    expect(state, kinetis_copy(second, first), "restore matching register state");

    first->timing.pit[0].current = 10u;
    second->timing.pit[0].current = 20u;
    first->timing.pit[0].flag = true;
    second->timing.pit[0].flag = false;
    expect(state,
           kinetis_register_state_equal(first, second, &address, &first_value, &second_value),
           "register comparison ignores live timer position");

    expect(state,
           kinetis_memory_write(first, 0x4005200eu, 2u, CORTEX_M4_ACCESS_DEBUG, 0xc520u) &&
               kinetis_memory_write(first, 0x4005200eu, 2u, CORTEX_M4_ACCESS_DEBUG, 0xd928u) &&
               kinetis_memory_write(first, 0x40052000u, 2u, CORTEX_M4_ACCESS_DEBUG, 0x40d7u) &&
               kinetis_memory_write(second, 0x4005200eu, 2u, CORTEX_M4_ACCESS_DEBUG, 0xc520u) &&
               kinetis_memory_write(second, 0x4005200eu, 2u, CORTEX_M4_ACCESS_DEBUG, 0xd928u) &&
               kinetis_memory_write(second, 0x40052000u, 2u, CORTEX_M4_ACCESS_DEBUG, 0x40d7u),
           "stage matching watchdog updates");
    kinetis_advance(first, 600u);
    expect(state,
           kinetis_register_state_equal(first, second, &address, &first_value, &second_value),
           "register comparison projects a committed watchdog update");
}

#ifdef K22_TEST_ALLOCATION_FAILURE
static void copy_allocation_cases(TestState* state, Kinetis* destination, Kinetis* source) {
    static const uint8_t window_data[] = {1u, 2u, 3u, 4u};
    static const uint8_t card[512] = {0u};
    expect(state,
           kinetis_flexbus_attach(source, 0x60000000u, window_data, sizeof(window_data), false),
           "attach source FlexBus window");
    expect(state, k22_sdhc_insert(&source->sdhc, card, sizeof(card), false),
           "insert source SDHC card");
    uint32_t failure_count = 0u;
    bool copy_succeeded = false;
    for (size_t allocation_index = 0u; allocation_index < 32u && !copy_succeeded;
         allocation_index++) {
        test_fail_allocation_after(allocation_index);
        copy_succeeded = kinetis_copy(destination, source);
        test_allow_allocations();
        failure_count += !copy_succeeded;
    }
    expect(state, copy_succeeded && failure_count != 0u,
           "device copy reports allocation failures before succeeding");
    kinetis_flexbus_detach(source);
}
#endif

int main(void) {
    TestState state = {0u, 0u, 0u};
    DeviceCensus census = {0u, 0u, UINT64_C(14695981039346656037)};
    Kinetis* destination_device = create_device(&state);
    Kinetis* source_device = create_device(&state);
    if (destination_device != NULL && source_device != NULL) {
        uninitialized_sram_cases(&state, destination_device, source_device);
        access_census(destination_device, &census);
        copy_guard_cases(&state, destination_device, source_device);
        register_state_cases(&state, destination_device, source_device);
#ifdef K22_TEST_ALLOCATION_FAILURE
        copy_allocation_cases(&state, destination_device, source_device);
#endif
        flexbus_cases(&state, destination_device);
        expect(&state,
               census.accepted_reads == 90u && census.accepted_writes == 87u &&
                   census.fingerprint == UINT64_C(3273249558290571156),
               "device census matches");
    }
    kinetis_destroy(source_device);
    kinetis_destroy(destination_device);
    return test_finish(&state);
}
