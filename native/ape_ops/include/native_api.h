/*
 * native_api.h — ctypes-friendly entry points for the three real APE
 * planners. Params are passed by pointer (never by value) and results
 * written into a caller-owned out-pointer, keeping the ABI unambiguous
 * across the ctypes boundary.
 */
#pragma once

#include <stdint.h>
#include "ape_types.h"

void ape_native_plan_ape1(const ape_params_t *params, ape_result_t *out);
void ape_native_plan_ape2(const ape_params_t *params, ape_result_t *out);
void ape_native_plan_ape3(const ape_params_t *params, ape_result_t *out);

/*
 * Struct-layout ABI cross-check: ape_native.py compares these against
 * ctypes.sizeof(ApeParams)/ctypes.sizeof(ApeResult) at import time and
 * refuses to proceed on mismatch, rather than silently misreading
 * fields across the ctypes boundary.
 */
int32_t ape_native_sizeof_params(void);
int32_t ape_native_sizeof_result(void);
