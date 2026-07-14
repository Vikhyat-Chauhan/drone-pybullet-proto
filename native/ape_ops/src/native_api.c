#include "native_api.h"
#include "ape1_bug.h"
#include "ape2_dwa.h"
#include "ape3_vfh.h"
#include <string.h>

void ape_native_plan_ape1(const ape_params_t *params, ape_result_t *out) {
    *out = ape1_bug_plan(params);
}

/* APE2 = DWA (ape2_dwa.c, medium compute), APE3 = VFH (ape3_vfh.c, most
 * compute) -- dispatch matches the source-file identity, which is also
 * how the gem5 cycle table (gem5_measured_latencies.py, keyed by
 * "ape2"/"ape3") is measured. */
void ape_native_plan_ape2(const ape_params_t *params, ape_search_state_t *state, ape_result_t *out) {
    *out = ape2_dwa_plan(params, state);
}

void ape_native_plan_ape3(const ape_params_t *params, ape_search_state_t *state, ape_result_t *out) {
    *out = ape3_vfh_plan(params, state);
}

void ape_native_search_state_reset(ape_search_state_t *state, int32_t grid_w, int32_t grid_h,
                                    float cell_size_m, float origin_x, float origin_y) {
    if (grid_w < 1) grid_w = 1;
    if (grid_h < 1) grid_h = 1;
    while ((int64_t)grid_w * (int64_t)grid_h > APE_GRID_MAX_CELLS) {
        /* Defensive clamp -- caller (Python) is expected to size within
         * bounds; shrink the taller dimension until it fits rather than
         * overrun the fixed-size cells[] buffer. */
        if (grid_h > grid_w) grid_h--; else grid_w--;
    }
    memset(state->cells, 0, sizeof(state->cells));
    state->grid_w = grid_w;
    state->grid_h = grid_h;
    state->cell_size_m = cell_size_m;
    state->origin_x = origin_x;
    state->origin_y = origin_y;
    state->initialized = 1;
}

int32_t ape_native_sizeof_params(void) { return (int32_t)sizeof(ape_params_t); }
int32_t ape_native_sizeof_result(void) { return (int32_t)sizeof(ape_result_t); }
int32_t ape_native_sizeof_search_state(void) { return (int32_t)sizeof(ape_search_state_t); }
