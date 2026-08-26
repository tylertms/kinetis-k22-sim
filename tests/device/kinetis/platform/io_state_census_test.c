#include "device/kinetis/gpio/io.h"

#include "test.h"

enum {
    PORTA = 0x40049000u,
    GPIOA = 0x400ff000u,
    USB0 = 0x40072000u,
    CAN0 = 0x40024000u,
    I2S0 = 0x4002f000u,
    FLEXBUS = 0x4000c000u,
    SYSMPU = 0x4000d000u,
};

#define MCM UINT32_C(0xe0080008)

typedef struct {
    uint32_t random_state;
    uint32_t read_successes;
    uint32_t write_successes;
    uint32_t event_count;
    uint64_t fingerprint;
} IoCensus;

static uint32_t next_random(IoCensus* census) {
    uint32_t random_value = census->random_state;
    random_value ^= random_value << 13u;
    random_value ^= random_value >> 17u;
    random_value ^= random_value << 5u;
    census->random_state = random_value;
    return random_value;
}

static void mix_fingerprint(IoCensus* census, uint32_t input_value) {
    census->fingerprint = (census->fingerprint ^ input_value) * UINT64_C(1099511628211);
}

static void record_event(void* context, const KinetisIoEvent* event) {
    IoCensus* census = context;
    census->event_count++;
    mix_fingerprint(census, event->type ^ event->source ^ event->value ^ event->auxiliary);
}

static void randomize_state(KinetisIo* io, IoCensus* census, uint32_t random_value) {
    const uint8_t port = (uint8_t)(random_value % KINETIS_IO_PORT_COUNT);
    const uint8_t pin = (uint8_t)((random_value >> 8u) % KINETIS_IO_PIN_COUNT);
    const uint32_t next_value = next_random(census);
    io->port_pcr[port][pin] = random_value;
    io->port_isfr[port] = next_value;
    io->port_dfer[port] = random_value ^ next_value;
    io->port_dfwr[port] = (uint8_t)random_value;

    io->gpio_pdor[port] = random_value;
    io->gpio_pddr[port] = next_value;
    io->gpio_external[port] = random_value >> 1u;
    io->gpio_external_drive[port] = next_value >> 1u;
    io->gpio_filtered[port] = next_random(census);
    io->gpio_pending[port] = next_random(census);
    io->gpio_filter_age[port][pin] = (uint8_t)next_value;

    io->usb[(random_value >> 16u) % sizeof(io->usb)] = (uint8_t)next_value;
    io->usb[0x84u] = (uint8_t)random_value;
    io->usb[0x94u] = (uint8_t)(next_value | 1u);
    io->usb_cycle_remainder = next_value % 1000u;

    io->can[(random_value >> 16u) % (sizeof(io->can) / sizeof(io->can[0]))] = next_value;
    io->can[0] = random_value & ~(1u << 31u);
    io->can[0x28u / 4u] = next_value;
    io->can[0x30u / 4u] = random_value;

    io->i2s[(random_value >> 16u) % (sizeof(io->i2s) / sizeof(io->i2s[0]))] = next_value;
    io->i2s[0] = random_value | UINT32_C(0x80000000);
    io->i2s[0x80u / 4u] = next_value | UINT32_C(0x80000000);
    io->i2s_receive_read = (uint8_t)(random_value % KINETIS_IO_FIFO_CAPACITY);
    io->i2s_receive_write = (uint8_t)(next_value % KINETIS_IO_FIFO_CAPACITY);
    io->i2s_receive_count = (uint8_t)(random_value % (KINETIS_IO_FIFO_CAPACITY + 1u));
    io->i2s_transmit_read = (uint8_t)(next_value % KINETIS_IO_FIFO_CAPACITY);
    io->i2s_transmit_write = (uint8_t)(random_value % KINETIS_IO_FIFO_CAPACITY);
    io->i2s_transmit_count = (uint8_t)(next_value % (KINETIS_IO_FIFO_CAPACITY + 1u));

    io->flexbus[(random_value >> 16u) % (sizeof(io->flexbus) / sizeof(io->flexbus[0]))] =
        next_value;
    io->mcm[(random_value >> 20u) % (sizeof(io->mcm) / sizeof(io->mcm[0]))] = next_value;
    io->sysmpu[(random_value >> 12u) % (sizeof(io->sysmpu) / sizeof(io->sysmpu[0]))] = next_value;
    const uint8_t region = (uint8_t)((random_value >> 24u) % 12u);
    const uint32_t descriptor = 0x400u / 4u + (uint32_t)region * 4u;
    io->sysmpu[0] = random_value | 1u;
    io->sysmpu[descriptor] = next_value & 0xffff0000u;
    io->sysmpu[descriptor + 1u] = io->sysmpu[descriptor] + (random_value & 0xffffu);
    io->sysmpu[descriptor + 2u] = random_value;
    io->sysmpu[descriptor + 3u] = 1u;
}

static uint32_t address_for(uint32_t random_value) {
    static const uint32_t bases[] = {PORTA, GPIOA, USB0, CAN0, I2S0, FLEXBUS, MCM, SYSMPU};
    static const uint32_t spans[] = {0xd0u, 0x18u, 0x160u, 0x8c0u, 0x108u, 0x64u, 0x3cu, 0x830u};
    const uint8_t region = (uint8_t)(random_value % (sizeof(bases) / sizeof(bases[0])));
    return bases[region] + ((random_value >> 8u) % spans[region]);
}

int main(void) {
    TestState state = {0u, 0u, 0u};
    IoCensus census = {UINT32_C(0x243f6a88), 0u, 0u, 0u, UINT64_C(14695981039346656037)};
    KinetisIoConfiguration configuration =
        kinetis_io_default_configuration(kinetis_profile_get(KINETIS_PROFILE_MK22FN1M012));
    for (uint8_t port = 0u; port < KINETIS_IO_PORT_COUNT; port++) {
        configuration.package_pin_mask[port] = UINT32_MAX;
    }
    configuration.event_handler = record_event;
    configuration.event_context = &census;
    KinetisIo io;
    expect(&state, kinetis_io_init(&io, configuration), "initialize I/O state census");
    for (KinetisPeripheralId peripheral = 0; peripheral < KINETIS_PERIPHERAL_COUNT; peripheral++) {
        kinetis_io_set_clock(&io, peripheral, true);
    }
    for (uint32_t iteration = 0u; iteration < 100000u; iteration++) {
        const uint32_t random_value = next_random(&census);
        randomize_state(&io, &census, random_value);
        const uint32_t address = address_for(random_value);
        const uint8_t access_size = (uint8_t)(random_value % 6u);
        uint32_t read_value = UINT32_MAX;
        const bool read_succeeded = kinetis_io_read(&io, address, access_size, &read_value);
        const bool write_succeeded =
            kinetis_io_write(&io, address, access_size, random_value ^ UINT32_C(0xa5a55a5a));
        const uint8_t port = (uint8_t)((random_value >> 8u) % KINETIS_IO_PORT_COUNT);
        const uint8_t pin = (uint8_t)((random_value >> 16u) % KINETIS_IO_PIN_COUNT);
        const bool pin_drive_succeeded =
            kinetis_io_drive_pin(&io, port, pin, (random_value & 1u) != 0u);
        const bool pin_release_succeeded = kinetis_io_release_pin(&io, port, pin);
        KinetisCanFrame frame = {random_value,
                                 (uint8_t)(random_value % 10u),
                                 {0u},
                                 (random_value & 1u) != 0u,
                                 (random_value & 2u) != 0u};
        frame.data[0] = (uint8_t)random_value;
        const bool can_receive_succeeded = kinetis_io_can_receive(&io, &frame);
        const bool i2s_receive_succeeded = kinetis_io_i2s_receive(&io, random_value);
        uint32_t transmit_sample = 0u;
        const bool i2s_transmit_succeeded = kinetis_io_i2s_transmit(&io, &transmit_sample);
        const bool sysmpu_access_allowed = kinetis_io_sysmpu_access(
            &io, random_value, (uint8_t)((random_value >> 20u) % 9u), (random_value & 4u) != 0u,
            (KinetisSysMpuAccess)((random_value >> 24u) % 4u));
        const bool flexbus_transfer_succeeded =
            kinetis_io_flexbus_transfer(&io, random_value, access_size, (random_value & 8u) != 0u,
                                        random_value ^ UINT32_C(0x5a5aa5a5));
        kinetis_io_advance(&io, (random_value & 0x7ffu) + 1u);
        census.read_successes += read_succeeded;
        census.write_successes += write_succeeded;
        mix_fingerprint(&census, read_value);
        mix_fingerprint(&census, transmit_sample);
        mix_fingerprint(&census,
                        read_succeeded | (write_succeeded << 1u) | (pin_drive_succeeded << 2u) |
                            (pin_release_succeeded << 3u) | (can_receive_succeeded << 4u) |
                            (i2s_receive_succeeded << 5u) | (i2s_transmit_succeeded << 6u) |
                            (sysmpu_access_allowed << 7u) | (flexbus_transfer_succeeded << 8u));
        mix_fingerprint(&census, kinetis_io_pin_input(&io, port));
        mix_fingerprint(&census, kinetis_io_irq_asserted(&io, (uint8_t)(random_value >> 24u)));
    }
    expect(&state,
           census.read_successes == 7833u && census.write_successes == 6818u &&
               census.event_count == 744342u && census.fingerprint == UINT64_C(8816404939693753862),
           "I/O state census matches");
    return test_finish(&state);
}
