#include "cortex_m4.h"

#include <stdlib.h>
#include <string.h>

struct CortexM4Coverage {
    uint32_t address;
    size_t size;
    uint64_t instructions;
    uint64_t skipped;
    uint64_t outside_range;
    uint64_t branches_taken;
    uint64_t branches_not_taken;
    size_t unique_instructions;
    size_t unique_skipped;
    size_t observed_branch_sites;
    size_t observed_branch_outcomes;
    size_t branch_sites_with_both_outcomes;
    uint8_t slots[];
};

static size_t coverage_slot(const CortexM4Coverage* coverage, uint32_t address) {
    if ((address & 1u) != 0u || address < coverage->address ||
        (size_t)(address - coverage->address) >= coverage->size) {
        return SIZE_MAX;
    }
    return (address - coverage->address) / 2u;
}

CortexM4Coverage* cortex_m4_coverage_create(uint32_t address, size_t size) {
    if ((address & 1u) != 0u || size == 0u || (size & 1u) != 0u || size > UINT32_MAX ||
        (uint64_t)address + size > UINT64_C(0x100000000)) {
        return NULL;
    }
    const size_t slot_count = size / 2u;
    if (slot_count > SIZE_MAX - sizeof(CortexM4Coverage)) {
        return NULL;
    }
    CortexM4Coverage* coverage = calloc(1, sizeof(*coverage) + slot_count);
    if (coverage != NULL) {
        coverage->address = address;
        coverage->size = size;
    }
    return coverage;
}

void cortex_m4_coverage_destroy(CortexM4Coverage* coverage) { free(coverage); }

void cortex_m4_coverage_clear(CortexM4Coverage* coverage) {
    if (coverage == NULL) {
        return;
    }
    const uint32_t address = coverage->address;
    const size_t size = coverage->size;
    memset(coverage, 0, sizeof(*coverage) + size / 2u);
    coverage->address = address;
    coverage->size = size;
}

void cortex_m4_coverage_record(CortexM4Coverage* coverage, uint32_t address, bool executed) {
    const size_t slot = coverage_slot(coverage, address);
    if (slot == SIZE_MAX) {
        coverage->outside_range++;
    } else {
        const uint8_t flag = executed ? CORTEX_M4_COVERAGE_EXECUTED : CORTEX_M4_COVERAGE_SKIPPED;
        if ((coverage->slots[slot] & flag) == 0u) {
            coverage->slots[slot] |= flag;
            if (executed) {
                coverage->unique_instructions++;
            } else {
                coverage->unique_skipped++;
            }
        }
    }
    coverage->instructions += executed;
    coverage->skipped += !executed;
}

void cortex_m4_coverage_record_branch(CortexM4Coverage* coverage, uint32_t address, bool taken) {
    const size_t slot = coverage_slot(coverage, address);
    if (slot == SIZE_MAX) {
        return;
    }
    const uint8_t outcome =
        taken ? CORTEX_M4_COVERAGE_BRANCH_TAKEN : CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN;
    const uint8_t opposite =
        taken ? CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN : CORTEX_M4_COVERAGE_BRANCH_TAKEN;
    if ((coverage->slots[slot] &
         (CORTEX_M4_COVERAGE_BRANCH_TAKEN | CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN)) == 0u) {
        coverage->observed_branch_sites++;
    }
    if ((coverage->slots[slot] & outcome) == 0u) {
        coverage->slots[slot] |= outcome;
        coverage->observed_branch_outcomes++;
        if ((coverage->slots[slot] & opposite) != 0u) {
            coverage->branch_sites_with_both_outcomes++;
        }
    }
    coverage->branches_taken += taken;
    coverage->branches_not_taken += !taken;
}

CortexM4CoverageResult cortex_m4_coverage_result(const CortexM4Coverage* coverage) {
    if (coverage == NULL) {
        return (CortexM4CoverageResult){0};
    }
    CortexM4CoverageResult result = {
        .instructions = coverage->instructions,
        .skipped = coverage->skipped,
        .outside_range = coverage->outside_range,
        .conditional_branches = coverage->branches_taken + coverage->branches_not_taken,
        .branches_taken = coverage->branches_taken,
        .branches_not_taken = coverage->branches_not_taken,
        .unique_instructions = coverage->unique_instructions,
        .unique_skipped = coverage->unique_skipped,
        .observed_branch_sites = coverage->observed_branch_sites,
        .observed_branch_outcomes = coverage->observed_branch_outcomes,
        .branch_sites_with_both_outcomes = coverage->branch_sites_with_both_outcomes,
    };
    return result;
}

uint8_t cortex_m4_coverage_flags(const CortexM4Coverage* coverage, uint32_t address) {
    if (coverage == NULL) {
        return 0u;
    }
    const size_t slot = coverage_slot(coverage, address);
    return slot == SIZE_MAX ? 0u : coverage->slots[slot];
}
