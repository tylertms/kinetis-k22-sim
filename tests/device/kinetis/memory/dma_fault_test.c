#include "device/kinetis/memory/internal.h"

#include <string.h>

#include "test.h"

typedef struct {
    uint8_t memory[256];
    uint32_t reads;
} BusState;

static bool bus_read(void* context, uint32_t address, uint8_t size, uint32_t* value) {
    BusState* state = context;
    if (address == UINT32_C(0x7fffffff) && size == 1u) {
        state->reads++;
        *value = 0u;
        return true;
    }
    if (address < UINT32_C(0x20000000) ||
        address - UINT32_C(0x20000000) > sizeof(state->memory) - size)
        return false;
    state->reads++;
    *value = kinetis_data_internal_load_bytes(state->memory, address - UINT32_C(0x20000000), size);
    return true;
}

static bool bus_write(void* context, uint32_t address, uint8_t size, uint32_t value) {
    BusState* state = context;
    if (address < UINT32_C(0x20000000) ||
        address - UINT32_C(0x20000000) > sizeof(state->memory) - size)
        return false;
    kinetis_data_internal_store_bytes(state->memory, address - UINT32_C(0x20000000), size, value);
    return true;
}

static void prepare_descriptor(KinetisData* data) {
    uint8_t* descriptor = data->dma + UINT32_C(0x1000);
    memset(descriptor, 0, DMA_TCD_SIZE);
    kinetis_data_internal_store_bytes(descriptor, 0u, 4u, UINT32_C(0x20000000));
    kinetis_data_internal_store_bytes(descriptor, 8u, 4u, 1u);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x10), 4u, UINT32_C(0x20000004));
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x16), 2u, 1u);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x1e), 2u, 1u);
}

static uint32_t error_status(const KinetisData* data) {
    return kinetis_data_internal_load_bytes(data->dma, 4u, 4u);
}

static void test_missing_bus_callbacks(TestState* state, const KinetisDeviceProfile* profile) {
    KinetisData* data = kinetis_data_create(profile, (KinetisDataBus){0});
    expect(state, data != NULL, "create DMA without bus callbacks");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA source read requires a bus callback");
    expect(state, error_status(data) == UINT32_C(0x80000002), "failed DMA source read records SBE");
    expect(state, (kinetis_data_internal_load_bytes(data->dma, UINT32_C(0x2c), 2u) & 1u) != 0u,
           "failed DMA source read records a channel error");
    kinetis_data_destroy(data);

    BusState bus = {0u};
    data = kinetis_data_create(profile, (KinetisDataBus){&bus, bus_read, NULL, NULL, NULL, NULL});
    expect(state, data != NULL, "create DMA without a bus write callback");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA destination write requires a bus callback");
    expect(state, bus.reads == 1u, "DMA reads the source before the destination failure");
    expect(state, error_status(data) == UINT32_C(0x80000001),
           "failed DMA destination write records DBE");
    kinetis_data_destroy(data);
}

static void test_error_replacement(TestState* state, const KinetisDeviceProfile* profile) {
    KinetisData* data = kinetis_data_create(profile, (KinetisDataBus){0});
    expect(state, data != NULL, "create DMA for error replacement");
    if (data == NULL)
        return;

    kinetis_data_internal_dma_error(data, 3u, 1u << 7u);
    kinetis_data_internal_dma_error(data, 1u, 1u);
    expect(state, error_status(data) == UINT32_C(0x80000101),
           "DMA ES describes only the latest error");
    expect(state,
           kinetis_data_internal_load_bytes(data->dma, UINT32_C(0x2c), 2u) == UINT32_C(0x000a),
           "DMA ERR accumulates channel errors");
    kinetis_data_destroy(data);
}

static void test_bus_error_progress(TestState* state, const KinetisDeviceProfile* profile) {
    BusState bus = {0u};
    KinetisData* data =
        kinetis_data_create(profile, (KinetisDataBus){&bus, bus_read, bus_write, NULL, NULL, NULL});
    expect(state, data != NULL, "create DMA for bus error progress");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    uint8_t* descriptor = data->dma + UINT32_C(0x1000);
    kinetis_data_internal_store_bytes(descriptor, 0u, 4u, UINT32_C(0x200000fc));
    kinetis_data_internal_store_bytes(descriptor, 4u, 2u, 4u);
    kinetis_data_internal_store_bytes(descriptor, 6u, 2u, UINT32_C(0x0202));
    kinetis_data_internal_store_bytes(descriptor, 8u, 4u, 8u);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x10), 4u, UINT32_C(0x20000000));
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x14), 2u, 4u);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA reports a source fault after one completed transfer");
    expect(state, kinetis_data_internal_load_bytes(descriptor, 0u, 4u) == UINT32_C(0x20000100),
           "source fault preserves completed source progress");
    expect(state,
           kinetis_data_internal_load_bytes(descriptor, UINT32_C(0x10), 4u) == UINT32_C(0x20000004),
           "source fault preserves completed destination progress");

    prepare_descriptor(data);
    descriptor = data->dma + UINT32_C(0x1000);
    kinetis_data_internal_store_bytes(descriptor, 4u, 2u, 4u);
    kinetis_data_internal_store_bytes(descriptor, 6u, 2u, UINT32_C(0x0204));
    kinetis_data_internal_store_bytes(descriptor, 8u, 4u, 16u);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x10), 4u, UINT32_C(0x20000100));
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x14), 2u, 16u);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA reports a destination fault after completing source reads");
    expect(state, kinetis_data_internal_load_bytes(descriptor, 0u, 4u) == UINT32_C(0x20000010),
           "destination fault preserves completed source progress");
    expect(state,
           kinetis_data_internal_load_bytes(descriptor, UINT32_C(0x10), 4u) == UINT32_C(0x20000100),
           "destination fault does not advance the failed destination");
    kinetis_data_destroy(data);
}

static void test_configuration_errors(TestState* state, const KinetisDeviceProfile* profile) {
    BusState bus = {0u};
    KinetisData* data =
        kinetis_data_create(profile, (KinetisDataBus){&bus, bus_read, bus_write, NULL, NULL, NULL});
    expect(state, data != NULL, "create DMA for configuration errors");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 6u, 2u, 3u << 8u);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "reserved DMA source size is rejected");
    expect(state, error_status(data) == UINT32_C(0x80000080),
           "reserved DMA source size records SAE");

    prepare_descriptor(data);
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 6u, 2u, UINT32_C(0x0100));
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 0u, 4u, UINT32_C(0x20000001));
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 4u, 2u, 1u);
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 8u, 4u, 2u);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "misaligned DMA source configuration is rejected");
    expect(state, error_status(data) == UINT32_C(0x800000c0),
           "misaligned DMA source records SAE and SOE");

    prepare_descriptor(data);
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), UINT32_C(0x16), 2u,
                                      UINT32_C(0x8001));
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "mismatched DMA iteration linking is rejected");
    expect(state, error_status(data) == UINT32_C(0x80000008),
           "mismatched DMA iteration linking records NCE");
    kinetis_data_destroy(data);
}

static void test_minor_byte_count_formats(TestState* state, const KinetisDeviceProfile* profile) {
    BusState bus = {0u};
    KinetisData* data =
        kinetis_data_create(profile, (KinetisDataBus){&bus, bus_read, bus_write, NULL, NULL, NULL});
    expect(state, data != NULL, "create DMA for minor byte count formats");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 0u, 4u, UINT32_C(0x10000000));
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 8u, 4u, UINT32_C(0x40000000));
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "minor-loop-disabled DMA uses all 32 count bits");
    expect(state, error_status(data) == UINT32_C(0x80000002),
           "full-width minor count reaches the source bus");

    prepare_descriptor(data);
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 0u, 4u, UINT32_C(0x10000000));
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 8u, 4u, 0u);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "zero full-width minor count starts a 4 GiB transfer");
    expect(state, error_status(data) == UINT32_C(0x80000002),
           "zero full-width minor count reaches the source bus");

    prepare_descriptor(data);
    kinetis_data_internal_store_bytes(data->dma, 0u, 4u, UINT32_C(0x80));
    kinetis_data_internal_store_bytes(data->dma + UINT32_C(0x1000), 8u, 4u, UINT32_C(0x400));
    expect(state, kinetis_data_internal_dma_service_channel(data, 0u),
           "mapped minor loop without offsets uses the 30-bit count");
    expect(state, bus.reads == UINT32_C(0x400), "mapped minor loop transfers 1024 bytes");
    kinetis_data_destroy(data);
}

static void test_scatter_gather(TestState* state, const KinetisDeviceProfile* profile) {
    BusState bus = {0u};
    KinetisData* data =
        kinetis_data_create(profile, (KinetisDataBus){&bus, bus_read, bus_write, NULL, NULL, NULL});
    expect(state, data != NULL, "create DMA for scatter gather");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    uint8_t* descriptor = data->dma + UINT32_C(0x1000);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x18), 4u, UINT32_C(0x20000004));
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x1c), 2u, UINT32_C(0x0010));
    expect(state, kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA completes before an invalid scatter address");
    expect(state, error_status(data) == UINT32_C(0x80000004),
           "unaligned scatter address records SGE");

    prepare_descriptor(data);
    descriptor = data->dma + UINT32_C(0x1000);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x18), 4u, UINT32_C(0x20000080));
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x1c), 2u, UINT32_C(0x013a));
    kinetis_data_internal_store_bytes(data->dma, UINT32_C(0x0c), 2u, 1u);
    expect(state, kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA loads an aligned scatter descriptor");
    expect(state, (kinetis_data_internal_load_bytes(data->dma, UINT32_C(0x0c), 2u) & 1u) == 0u,
           "completed scatter descriptor disables its request");
    expect(state, (kinetis_data_internal_load_bytes(data->dma, UINT32_C(0x24), 2u) & 1u) != 0u,
           "completed scatter descriptor raises its interrupt");
    expect(state, (data->dma_requests & 2u) != 0u,
           "completed scatter descriptor links its major channel");
    expect(state, kinetis_data_internal_load_bytes(descriptor, UINT32_C(0x1c), 2u) == 0u,
           "new scatter descriptor retains its control flags");
    kinetis_data_destroy(data);
}

static void test_burst_sizes(TestState* state, const KinetisDeviceProfile* profile) {
    BusState bus = {0u};
    for (uint8_t index = 0u; index < 32u; index++)
        bus.memory[index] = (uint8_t)(index + 1u);
    KinetisData* data =
        kinetis_data_create(profile, (KinetisDataBus){&bus, bus_read, bus_write, NULL, NULL, NULL});
    expect(state, data != NULL, "create DMA for burst sizes");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    uint8_t* descriptor = data->dma + UINT32_C(0x1000);
    kinetis_data_internal_store_bytes(descriptor, 4u, 2u, 32u);
    kinetis_data_internal_store_bytes(descriptor, 6u, 2u, UINT32_C(0x0504));
    kinetis_data_internal_store_bytes(descriptor, 8u, 4u, 32u);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x10), 4u, UINT32_C(0x20000040));
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x14), 2u, 16u);
    expect(state, kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA accepts 32-byte source and 16-byte destination sizes");
    expect(state, memcmp(bus.memory, bus.memory + 64u, 32u) == 0,
           "DMA copies a 32-byte burst without truncation");
    kinetis_data_destroy(data);
}

static void test_maximum_modulo(TestState* state, const KinetisDeviceProfile* profile) {
    BusState bus = {0u};
    KinetisData* data =
        kinetis_data_create(profile, (KinetisDataBus){&bus, bus_read, bus_write, NULL, NULL, NULL});
    expect(state, data != NULL, "create DMA for maximum modulo");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    uint8_t* descriptor = data->dma + UINT32_C(0x1000);
    kinetis_data_internal_store_bytes(descriptor, 0u, 4u, UINT32_C(0x7fffffff));
    kinetis_data_internal_store_bytes(descriptor, 4u, 2u, 1u);
    kinetis_data_internal_store_bytes(descriptor, 6u, 2u, 31u << 11u);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x16), 2u, 2u);
    kinetis_data_internal_store_bytes(descriptor, UINT32_C(0x1e), 2u, 2u);

    expect(state, kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA accepts the maximum source modulo");
    expect(state, kinetis_data_internal_load_bytes(descriptor, 0u, 4u) == 0u,
           "maximum source modulo wraps at 2 GiB");
    kinetis_data_destroy(data);
}

static void test_channel_boundaries(TestState* state, const KinetisDeviceProfile* profile) {
    KinetisData* data = kinetis_data_create(profile, (KinetisDataBus){0});
    expect(state, data != NULL, "create DMA for channel boundaries");
    if (data == NULL)
        return;

    kinetis_data_internal_dma_queue_always_enabled(data, data->dma_channel_count);
    expect(state, data->dma_requests == 0u, "out-of-range always-enabled channel is ignored");
    kinetis_data_internal_store_bytes(data->dma, 0u, 4u, 4u);
    expect(state, kinetis_data_internal_dma_select_channel(data) == UINT8_MAX,
           "round-robin DMA selection reports an empty request set");

    uint32_t value = 0u;
    const uint32_t outside_descriptor =
        DMA_BASE + UINT32_C(0x1000) + (uint32_t)data->dma_channel_count * DMA_TCD_SIZE;
    expect(state, !kinetis_data_internal_dma_read(data, outside_descriptor, 1u, &value),
           "out-of-range DMA descriptor read is rejected");
    expect(state, !kinetis_data_internal_dma_write(data, outside_descriptor, 1u, 0u),
           "out-of-range DMA descriptor write is rejected");

    kinetis_data_destroy(data);
}

int main(void) {
    TestState state = {0u, 0u, 0u};
    const KinetisDeviceProfile* profile = kinetis_profile_get(KINETIS_PROFILE_MK22FN12810);
    expect(&state, profile != NULL, "find DMA test profile");
    if (profile != NULL) {
        test_missing_bus_callbacks(&state, profile);
        test_error_replacement(&state, profile);
        test_bus_error_progress(&state, profile);
        test_configuration_errors(&state, profile);
        test_minor_byte_count_formats(&state, profile);
        test_scatter_gather(&state, profile);
        test_burst_sizes(&state, profile);
        test_maximum_modulo(&state, profile);
        test_channel_boundaries(&state, profile);
    }
    return test_finish(&state);
}
