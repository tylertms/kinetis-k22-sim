#include "device/kinetis/memory/internal.h"

#include <string.h>

#include "test.h"

typedef struct {
    uint32_t reads;
} BusState;

static bool bus_read(void* context, uint32_t address, uint8_t size, uint32_t* value) {
    BusState* state = context;
    state->reads++;
    (void)address;
    (void)size;
    *value = UINT32_C(0x5aa5a55a);
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

static void test_missing_bus_callbacks(TestState* state, const KinetisDeviceProfile* profile) {
    KinetisData* data = kinetis_data_create(profile, (KinetisDataBus){0});
    expect(state, data != NULL, "create DMA without bus callbacks");
    if (data == NULL)
        return;

    prepare_descriptor(data);
    expect(state, !kinetis_data_internal_dma_service_channel(data, 0u),
           "DMA source read requires a bus callback");
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
        test_channel_boundaries(&state, profile);
    }
    return test_finish(&state);
}
