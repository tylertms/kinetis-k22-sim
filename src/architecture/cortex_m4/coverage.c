#include "cortex_m4.h"

#include <stdlib.h>

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
    size_t covered_instructions;
    size_t total_instructions;
    size_t covered_branch_sites;
    size_t total_branch_sites;
    uint8_t slots[];
};

enum {
    COVERAGE_DEFINED = 1u << 4,
    COVERAGE_CONDITIONAL_BRANCH = 1u << 5,
    COVERAGE_DEFINITION_MASK = COVERAGE_DEFINED | COVERAGE_CONDITIONAL_BRANCH,
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
    coverage->instructions = 0u;
    coverage->skipped = 0u;
    coverage->outside_range = 0u;
    coverage->branches_taken = 0u;
    coverage->branches_not_taken = 0u;
    coverage->unique_instructions = 0u;
    coverage->unique_skipped = 0u;
    coverage->observed_branch_sites = 0u;
    coverage->observed_branch_outcomes = 0u;
    coverage->branch_sites_with_both_outcomes = 0u;
    coverage->covered_instructions = 0u;
    coverage->covered_branch_sites = 0u;
    for (size_t slot = 0u; slot < coverage->size / 2u; slot++) {
        coverage->slots[slot] &= COVERAGE_DEFINITION_MASK;
    }
}

bool cortex_m4_coverage_define_instruction(CortexM4Coverage* coverage, uint32_t address,
                                           bool conditional_branch) {
    const size_t slot = coverage == NULL ? SIZE_MAX : coverage_slot(coverage, address);
    if (slot == SIZE_MAX) {
        return false;
    }
    if ((coverage->slots[slot] & COVERAGE_DEFINED) == 0u) {
        coverage->slots[slot] |= COVERAGE_DEFINED;
        coverage->total_instructions++;
        if ((coverage->slots[slot] & CORTEX_M4_COVERAGE_EXECUTED) != 0u) {
            coverage->covered_instructions++;
        }
    }
    if (conditional_branch && (coverage->slots[slot] & COVERAGE_CONDITIONAL_BRANCH) == 0u) {
        coverage->slots[slot] |= COVERAGE_CONDITIONAL_BRANCH;
        coverage->total_branch_sites++;
        if ((coverage->slots[slot] &
             (CORTEX_M4_COVERAGE_BRANCH_TAKEN | CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN)) != 0u) {
            coverage->covered_branch_sites++;
        }
    }
    return true;
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
                coverage->covered_instructions += (coverage->slots[slot] & COVERAGE_DEFINED) != 0u;
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
        coverage->covered_branch_sites +=
            (coverage->slots[slot] & COVERAGE_CONDITIONAL_BRANCH) != 0u;
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
        .covered_instructions = coverage->covered_instructions,
        .total_instructions = coverage->total_instructions,
        .instruction_coverage_percent = coverage->total_instructions == 0u
                                            ? 0.0
                                            : 100.0 * (double)coverage->covered_instructions /
                                                  (double)coverage->total_instructions,
        .covered_branch_sites = coverage->covered_branch_sites,
        .total_branch_sites = coverage->total_branch_sites,
        .branch_coverage_percent = coverage->total_branch_sites == 0u
                                       ? 0.0
                                       : 100.0 * (double)coverage->covered_branch_sites /
                                             (double)coverage->total_branch_sites,
    };
    return result;
}

uint8_t cortex_m4_coverage_flags(const CortexM4Coverage* coverage, uint32_t address) {
    if (coverage == NULL) {
        return 0u;
    }
    const size_t slot = coverage_slot(coverage, address);
    return slot == SIZE_MAX ? 0u : coverage->slots[slot] & 0x0fu;
}
