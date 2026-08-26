#ifndef KINETIS_SIM_SDHC_H
#define KINETIS_SIM_SDHC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*KinetisSdhcBusRead)(void* context, uint32_t bus_address, uint8_t access_size,
                                   uint32_t* read_value);
typedef bool (*KinetisSdhcBusWrite)(void* context, uint32_t bus_address, uint8_t access_size,
                                    uint32_t write_value);

typedef struct {
    void* context;
    KinetisSdhcBusRead read;
    KinetisSdhcBusWrite write;
} KinetisSdhcBus;

typedef struct {
    KinetisSdhcBus bus;
    uint32_t registers[64];
    uint8_t* card;
    size_t card_size;
    size_t transfer_address;
    size_t transfer_remaining;
    uint32_t block_length;
    uint32_t relative_address;
    bool present;
    bool write_protected;
    bool clock_enabled;
    bool application_command;
    bool selected;
    bool transfer_read;
    bool transfer_multiple;
} KinetisSdhc;

bool kinetis_sdhc_init(KinetisSdhc* sdhc, KinetisSdhcBus bus);
void kinetis_sdhc_destroy(KinetisSdhc* sdhc);
void kinetis_sdhc_reset(KinetisSdhc* sdhc);
bool kinetis_sdhc_copy(KinetisSdhc* destination, const KinetisSdhc* source, KinetisSdhcBus bus);
void kinetis_sdhc_set_clock(KinetisSdhc* sdhc, bool enabled);
bool kinetis_sdhc_state_insert(KinetisSdhc* sdhc, const void* card_data, size_t card_size,
                               bool write_protected);
void kinetis_sdhc_state_eject(KinetisSdhc* sdhc);
bool kinetis_sdhc_state_read_card(const KinetisSdhc* sdhc, size_t card_offset, void* card_data,
                                  size_t byte_count);
bool kinetis_sdhc_read(KinetisSdhc* sdhc, uint32_t register_address, uint8_t access_size,
                       uint32_t* read_value);
bool kinetis_sdhc_write(KinetisSdhc* sdhc, uint32_t register_address, uint8_t access_size,
                        uint32_t write_value);
bool kinetis_sdhc_irq(const KinetisSdhc* sdhc);

#endif
