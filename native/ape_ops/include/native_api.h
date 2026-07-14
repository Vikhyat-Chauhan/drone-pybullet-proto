/*
 * native_api.h — ctypes-friendly entry points for the three real APE
 * planners. Params are passed by pointer (never by value) and results
 * written into a caller-owned out-pointer, keeping the ABI unambiguous
 * across the ctypes boundary.
 */
#pragma once

#include <stdint.h>
#include "ape_types.h"

/* APE1 has no persistent memory by design (type-level enforcement --
 * see ape_types.h's ape_search_state_t doc); APE2/APE3 take an optional
 * search-state pointer (NULL for avoidance-path calls that don't carry
 * one). */
void ape_native_plan_ape1(const ape_params_t *params, ape_result_t *out);
void ape_native_plan_ape2(const ape_params_t *params, ape_search_state_t *state, ape_result_t *out);
void ape_native_plan_ape3(const ape_params_t *params, ape_search_state_t *state, ape_result_t *out);

/* Zeroes state->cells and (re)sets its grid geometry -- call once per
 * mission (nav_algorithm.py's begin_mission()) before the first plan
 * call that uses this state. */
void ape_native_search_state_reset(ape_search_state_t *state, int32_t grid_w, int32_t grid_h,
                                    float cell_size_m, float origin_x, float origin_y);

/*
 * Struct-layout ABI cross-check: ape_native.py compares these against
 * ctypes.sizeof(ApeParams)/ctypes.sizeof(ApeResult)/ctypes.sizeof(ApeSearchState)
 * at import time and refuses to proceed on mismatch, rather than
 * silently misreading fields across the ctypes boundary.
 */
int32_t ape_native_sizeof_params(void);
int32_t ape_native_sizeof_result(void);
int32_t ape_native_sizeof_search_state(void);
