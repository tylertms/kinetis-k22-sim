#include "cortex_m4.h"

#include <stdlib.h>
#include <string.h>

struct CortexM4Coverage {
    uint32_t address;
    size_t size;
    size_t slot_count;
    uint64_t instructions;
    uint64_t skipped;
    uint64_t outside_range;
    uint64_t conditional_branches;
    uint64_t branches_taken;
    uint64_t branches_not_taken;
    size_t unique_instructions;
    size_t unique_skipped;
    size_t unique_branch_sites;
    size_t unique_branch_outcomes;
    size_t fully_covered_branch_sites;
    uint8_t slots[];
};

static uint8_t* coverage_slot(CortexM4Coverage* coverage, uint32_t address) {
    if (coverage == NULL || (address & 1u) != 0u || address < coverage->address ||
        (size_t)(address - coverage->address) >= coverage->size) {
        return NULL;
    }
    return &coverage->slots[(address - coverage->address) / 2u];
}

static const uint8_t* const_coverage_slot(const CortexM4Coverage* coverage, uint32_t address) {
    if (coverage == NULL || (address & 1u) != 0u || address < coverage->address ||
        (size_t)(address - coverage->address) >= coverage->size) {
        return NULL;
    }
    return &coverage->slots[(address - coverage->address) / 2u];
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
        coverage->slot_count = slot_count;
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
    const size_t slot_count = coverage->slot_count;
    memset(coverage, 0, sizeof(*coverage) + slot_count);
    coverage->address = address;
    coverage->size = size;
    coverage->slot_count = slot_count;
}

void cortex_m4_coverage_record(void* context, uint32_t address, uint32_t opcode, bool executed) {
    CortexM4Coverage* coverage = context;
    if (coverage == NULL) {
        return;
    }
    uint8_t* slot = coverage_slot(coverage, address);
    if (slot == NULL) {
        coverage->outside_range++;
    } else if (executed) {
        if ((*slot & CORTEX_M4_COVERAGE_EXECUTED) == 0u) {
            *slot |= CORTEX_M4_COVERAGE_EXECUTED;
            coverage->unique_instructions++;
        }
    } else {
        if ((*slot & CORTEX_M4_COVERAGE_SKIPPED) == 0u) {
            *slot |= CORTEX_M4_COVERAGE_SKIPPED;
            coverage->unique_skipped++;
        }
    }
    if (executed) {
        coverage->instructions++;
    } else {
        coverage->skipped++;
    }
    (void)opcode;
}

void cortex_m4_coverage_record_branch(CortexM4Coverage* coverage, uint32_t address, bool taken) {
    uint8_t* slot = coverage_slot(coverage, address);
    if (slot == NULL) {
        return;
    }
    const uint8_t outcome =
        taken ? CORTEX_M4_COVERAGE_BRANCH_TAKEN : CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN;
    const uint8_t opposite =
        taken ? CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN : CORTEX_M4_COVERAGE_BRANCH_TAKEN;
    if ((*slot & (CORTEX_M4_COVERAGE_BRANCH_TAKEN | CORTEX_M4_COVERAGE_BRANCH_NOT_TAKEN)) == 0u) {
        coverage->unique_branch_sites++;
    }
    if ((*slot & outcome) == 0u) {
        *slot |= outcome;
        coverage->unique_branch_outcomes++;
        if ((*slot & opposite) != 0u) {
            coverage->fully_covered_branch_sites++;
        }
    }
    coverage->conditional_branches++;
    if (taken) {
        coverage->branches_taken++;
    } else {
        coverage->branches_not_taken++;
    }
}

CortexM4CoverageResult cortex_m4_coverage_result(const CortexM4Coverage* coverage) {
    if (coverage == NULL) {
        return (CortexM4CoverageResult){0};
    }
    return (CortexM4CoverageResult){
        coverage->instructions,
        coverage->skipped,
        coverage->outside_range,
        coverage->conditional_branches,
        coverage->branches_taken,
        coverage->branches_not_taken,
        coverage->unique_instructions,
        coverage->unique_skipped,
        coverage->unique_branch_sites,
        coverage->unique_branch_outcomes,
        coverage->fully_covered_branch_sites,
    };
}

uint8_t cortex_m4_coverage_flags(const CortexM4Coverage* coverage, uint32_t address) {
    const uint8_t* slot = const_coverage_slot(coverage, address);
    return slot == NULL ? 0u : *slot;
}
