#include "kinetis_k22.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "architecture/cortex_m4/internal.h"
#include "test.h"

typedef struct {
    uint64_t examined_count;
    uint64_t permitted_count;
    uint64_t executed_count;
    uint64_t fingerprint;
} Census;

static uint64_t mix(uint64_t fingerprint, uint32_t value) {
    return (fingerprint ^ value) * UINT64_C(1099511628211);
}

static void prepare_cpu(CortexM4* cpu, uint8_t cpu_state) {
    static const uint32_t register_values[][4] = {
        {0u, 1u, 0x20000100u, 0x7fffffffu},
        {UINT32_MAX, 0x80000000u, 0x20000200u, 0x55555555u},
        {0xaaaaaaaau, 0x12345678u, 0x20000300u, 0x80000001u},
    };
    for (uint8_t register_index = 0u; register_index < CORTEX_M4_REGISTER_COUNT; register_index++) {
        cpu->registers[register_index] = register_values[cpu_state][register_index & 3u];
    }

    cpu->registers[13] = 0x20001000u;
    cpu->registers[15] = 0x100u;
    cpu->msp = 0x20001000u;
    cpu->psp = 0x20001800u;

    cpu->xpsr = CORTEX_M4_XPSR_T | (cpu_state == 1u ? UINT32_C(0xf0000000) : 0u);
    cpu->it_state = cpu_state == 2u ? 0x0cu : 0u;
    cpu->control = cpu_state == 2u ? CORTEX_M4_CONTROL_NPRIV : 0u;
    cpu->ccr = cpu_state == 1u ? 0x18u : 0u;
    cpu->primask = cpu_state == 1u;
    cpu->basepri = cpu_state == 2u ? 0x80u : 0u;
    cpu->faultmask = cpu_state == 2u;
    cpu->exclusive_valid = cpu_state == 2u;
    cpu->exclusive_address = 0x20000200u;
    cpu->exclusive_size = 4u;
    cpu->cpacr = cpu_state == 0u ? 0u : 0x00f00000u;
    cpu->fpscr = cpu_state == 2u ? 0x03c00000u : 0u;

    for (uint8_t fp_register_index = 0u; fp_register_index < CORTEX_M4_FP_REGISTER_COUNT;
         fp_register_index++) {
        cpu->fp_registers[fp_register_index] = register_values[cpu_state][fp_register_index & 3u];
    }

    cpu->cfsr = 0u;
    cpu->hfsr = 0u;
    cpu->system_pending = 0u;
    cpu->stop = CORTEX_M4_STOP_RUNNING;
}

static void record_execution(Census* census, CortexM4* cpu, bool instruction_executed,
                             uint32_t encoding) {
    census->executed_count += instruction_executed;
    census->fingerprint = mix(census->fingerprint, encoding);
    census->fingerprint = mix(census->fingerprint, instruction_executed);
    census->fingerprint = mix(census->fingerprint, cpu->registers[0]);
    census->fingerprint = mix(census->fingerprint, cpu->registers[15]);
    census->fingerprint = mix(census->fingerprint, cpu->xpsr);
    census->fingerprint = mix(census->fingerprint, cpu->cfsr);
}

static Census census_thumb16(CortexM4* cpu) {
    Census census = {0u, 0u, 0u, UINT64_C(14695981039346656037)};
    for (uint8_t cpu_state = 0u; cpu_state < 3u; cpu_state++) {
        for (uint32_t opcode = 0u; opcode <= UINT16_MAX; opcode++) {
            prepare_cpu(cpu, cpu_state);
            census.examined_count++;
            if (cortex_m4_check_instruction_constraints(cpu, (uint16_t)opcode, 0u, false) !=
                CORTEX_M4_INSTRUCTION_EXECUTE) {
                continue;
            }
            census.permitted_count++;
            record_execution(&census, cpu, cortex_m4_execute_thumb16(cpu, (uint16_t)opcode),
                             opcode);
        }
    }
    return census;
}

static Census census_thumb32(CortexM4* cpu, uint8_t shard) {
    Census census = {0u, 0u, 0u, UINT64_C(14695981039346656037)};
    const uint32_t first_encoding = 0xe800u + (uint32_t)shard * 0x100u;
    const uint32_t last_encoding = first_encoding + 0xffu;
    for (uint8_t cpu_state = 0u; cpu_state < 3u; cpu_state++) {
        for (uint32_t first_halfword = first_encoding; first_halfword <= last_encoding;
             first_halfword++) {
            for (uint32_t second_halfword = 0u; second_halfword <= UINT16_MAX; second_halfword++) {
                prepare_cpu(cpu, cpu_state);
                census.examined_count++;
                if (cortex_m4_check_instruction_constraints(cpu, (uint16_t)first_halfword,
                                                            (uint16_t)second_halfword, true) !=
                    CORTEX_M4_INSTRUCTION_EXECUTE) {
                    continue;
                }
                census.permitted_count++;
                record_execution(&census, cpu,
                                 cortex_m4_execute_thumb32(cpu, (uint16_t)first_halfword,
                                                           (uint16_t)second_halfword),
                                 (first_halfword << 16u) | second_halfword);
            }
        }
    }
    return census;
}

int main(int argc, char** argv) {
    char* end = NULL;
    const unsigned long parsed_shard = argc == 2 ? strtoul(argv[1], &end, 10) : 0u;
    TestState test_state = {0};
    const bool valid_shard =
        argc == 1 || (argc == 2 && end != argv[1] && *end == '\0' && parsed_shard < 24u);
    expect(&test_state, valid_shard, "encoding census shard is valid");
    if (!valid_shard) {
        return test_finish(&test_state);
    }
    const uint8_t shard = (uint8_t)parsed_shard;
    static const uint64_t expected_permitted[24] = {
        34459455u, 31070047u, 42017792u, 41716736u, 50331648u, 50331648u, 50331648u, 50331648u,
        39900890u, 39557330u, 43139072u, 40799644u, 39931904u, 39587840u, 43139072u, 43614208u,
        34774896u, 45435808u, 50196096u, 49892040u, 50331648u, 50331648u, 50331648u, 50331648u,
    };
    static const uint64_t expected_executed[24] = {
        11165931u, 18487135u, 11515392u, 10687488u, 320256u,   2486784u,  2891040u,  0u,
        25994022u, 25650462u, 18776064u, 17231371u, 26013696u, 25669632u, 18776064u, 15302656u,
        16672624u, 7687072u,  1104264u,  3482136u,  0u,        0u,        0u,        0u,
    };
    static const uint64_t expected_fingerprints[24] = {
        UINT64_C(12359392008932972023), UINT64_C(3399418810324431349),
        UINT64_C(6814390124731425717),  UINT64_C(16073962114491256845),
        UINT64_C(229153025585920568),   UINT64_C(8806714289990595429),
        UINT64_C(5579980413187923948),  UINT64_C(13945169685019501349),
        UINT64_C(14429519315073962497), UINT64_C(9360902978296142409),
        UINT64_C(3022176430939378469),  UINT64_C(8930834599203716791),
        UINT64_C(14938244973230212133), UINT64_C(18390324586635231269),
        UINT64_C(18404655547084280613), UINT64_C(14976156243861903901),
        UINT64_C(4316239447711128326),  UINT64_C(5857029729619730063),
        UINT64_C(4570402664723792546),  UINT64_C(14807501672243050140),
        UINT64_C(2258742799113528101),  UINT64_C(12554654631497704229),
        UINT64_C(13714994920826217253), UINT64_C(2274096894812758821),
    };
    KinetisK22Configuration configuration = kinetis_k22_default_configuration();
    configuration.flash_size = 4096u;
    configuration.sram_size = 65536u;
    KinetisK22* device = kinetis_k22_create(configuration);
    expect(&test_state, device != NULL, "device != NULL");
    const uint32_t vectors[2] = {0x20001000u, 0x101u};
    expect(&test_state, kinetis_k22_load(device, 0u, vectors, sizeof(vectors)),
           "kinetis_k22_load(device, 0u, vectors, sizeof(vectors))");
    expect(&test_state, kinetis_k22_reset(device), "kinetis_k22_reset(device)");
    CortexM4* cpu = kinetis_k22_cpu(device);
    const Census thumb16 = census_thumb16(cpu);
    const Census thumb32 = census_thumb32(cpu, shard);
    expect(&test_state,
           thumb16.examined_count == 196608u && thumb16.permitted_count == 185351u &&
               thumb16.executed_count == 146023u &&
               thumb16.fingerprint == UINT64_C(14300317076329787867),
           "thumb16 census matches");
    const bool thumb32_matches = thumb32.examined_count == 50331648u &&
                                 thumb32.permitted_count == expected_permitted[shard] &&
                                 thumb32.executed_count == expected_executed[shard] &&
                                 thumb32.fingerprint == expected_fingerprints[shard];
    if (!thumb32_matches) {
        fprintf(stderr,
                "[census] shard=%u examined=%" PRIu64 " permitted=%" PRIu64 " executed=%" PRIu64
                " fingerprint=%" PRIu64 "\n",
                shard, thumb32.examined_count, thumb32.permitted_count, thumb32.executed_count,
                thumb32.fingerprint);
    }
    expect(&test_state, thumb32_matches, "thumb32 census matches");
    kinetis_k22_destroy(device);
    return test_finish(&test_state);
}
