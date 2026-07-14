/*
 * main.c — gem5 cycle-accurate benchmark harness for the real native APE
 * planners (ape1_bug_plan / ape2_dwa_plan / ape3_vfh_plan — see
 * native/ape_ops/src/). Measures ONLY the planner call itself: the
 * m5_reset_stats()/m5_dump_stats() ROI bracket below wraps a fixed
 * ITERATIONS-call loop and nothing else, so process/loader/libc startup
 * is excluded from the measured cycle count.
 *
 * Which planner this binary measures is selected at compile time via
 * -DAPE_BENCH_TARGET=1|2|3, producing three separate static binaries
 * (bench_ape1/bench_ape2/bench_ape3) — see ../Makefile. A second axis,
 * -DAPE_BENCH_MODE=0|1 (0=avoidance/target-known, the default; 1=search/
 * target-undetected), produces the search_ape1/search_ape2/search_ape3
 * variants that measure the search-and-rescue exploration code path
 * added to each planner (see native/ape_ops/src/ape{1_bug,2_dwa,3_vfh}.c)
 * -- a genuinely different workload per planner (APE2/APE3 also touch
 * their persistent search-state grid), not just a different input to the
 * same code.
 *
 * Fixture: one fixed "open corridor" LiDAR-style scan, built once before
 * the ROI loop starts (setup cost is deliberately outside m5_reset_stats).
 * Geometry/config values mirror this repo's live defaults (Lidar2D in
 * main.py: num_rays=48, fov_deg=300, max_range=15.0; GoToConfig/
 * AvoidCfg/RiskCfg/EventDecisionCfg/AlgoTuning in nav_algorithm.py) so
 * the measured op count reflects a realistic invocation, not an
 * arbitrary one. All three planners' op counts are independent of scan
 * *content* (see each planner's own header comment), so the exact
 * range values only need to be plausible, not tuned per-planner. The
 * search-state grid (APE_BENCH_MODE=1) is pre-populated with a mix of
 * unknown/free/occupied cells before the ROI loop, matching a plausible
 * mid-mission map rather than an empty-grid edge case that would
 * understate real search compute cost.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gem5/m5ops.h>

#include "ape_types.h"
#include "ape1_bug.h"
#include "ape2_dwa.h"
#include "ape3_vfh.h"

#ifndef APE_BENCH_TARGET
#error "APE_BENCH_TARGET must be defined to 1, 2, or 3"
#endif

#ifndef APE_BENCH_MODE
#define APE_BENCH_MODE 0   /* 0 = avoidance (target known), 1 = search */
#endif

#define ITERATIONS 200

/* Search-state grid fixture (only used when APE_BENCH_MODE==1) --
 * dimensions match nav_algorithm.py's SearchCfg defaults for each
 * planner's tier (APE2 coarse/partial, APE3 fine/full). */
#define SEARCH_ORIGIN_X   -110.0f
#define SEARCH_ORIGIN_Y   -60.0f
#define SEARCH_GRID_W_APE2 64
#define SEARCH_GRID_H_APE2 32
#define SEARCH_CELL_M_APE2 3.75f
#define SEARCH_GRID_W_APE3 12
#define SEARCH_GRID_H_APE3 8
#define SEARCH_CELL_M_APE3 18.3f

#define NUM_RAYS   48
#define FOV_DEG    300.0f
#define N_LAYERS   5
#define VERTICAL_ANGLE_MIN       (-0.0872665f)   /* -5 deg */
#define VERTICAL_ANGLE_INCREMENT (0.0436332f)    /* (2*5deg)/(n_layers-1) */
#define RANGE_MIN  0.05f
#define RANGE_MAX  60.0f
#define OPEN_CORRIDOR_RANGE_M 15.0f   /* clear corridor: far, uniform returns */

static float g_ranges[N_LAYERS * NUM_RAYS];

static void build_fixture(ape_params_t *p) {
    float fov_rad = FOV_DEG * (float)M_PI / 180.0f;
    float angle_min = -fov_rad / 2.0f;
    float angle_increment = fov_rad / (float)(NUM_RAYS - 1);

    for (int32_t layer = 0; layer < N_LAYERS; layer++) {
        for (int32_t i = 0; i < NUM_RAYS; i++) {
            /* Mostly-open corridor with a couple of nearer returns off to
             * one side, so the sector-min/histogram/DWA-clearance logic
             * all have real (non-degenerate) work to do, matching a
             * plausible in-flight scan rather than an all-max-range
             * edge case. */
            float angle_deg = (angle_min + (float)i * angle_increment) * 180.0f / (float)M_PI;
            float r = OPEN_CORRIDOR_RANGE_M;
            if (angle_deg > 40.0f && angle_deg < 70.0f) {
                r = 4.0f;   /* nearer obstacle, right-of-center */
            }
            g_ranges[layer * NUM_RAYS + i] = r;
        }
    }

    memset(p, 0, sizeof(*p));
    p->ranges = g_ranges;
    p->n_ranges = NUM_RAYS;
    p->n_layers = N_LAYERS;
    p->angle_min = angle_min;
    p->angle_increment = angle_increment;
    p->vertical_angle_min = VERTICAL_ANGLE_MIN;
    p->vertical_angle_increment = VERTICAL_ANGLE_INCREMENT;
    p->range_min = RANGE_MIN;
    p->range_max = RANGE_MAX;

    p->v_cmd = 6.0f;
#if APE_BENCH_MODE == 1
    p->yaw_err = 0.0f;          /* placeholder, ignored by search-mode code */
    p->target_detected = 0;
    p->drone_x = SEARCH_ORIGIN_X + 5.0f * SEARCH_CELL_M_APE3;  /* mid-grid-ish */
    p->drone_y = SEARCH_ORIGIN_Y + 3.0f * SEARCH_CELL_M_APE3;
    p->drone_yaw = 0.3f;
#else
    p->yaw_err = 0.15f;
    p->target_detected = 1;
#endif

    /* GoToConfig (nav_algorithm.py) */
    p->max_v = 15.0f;
    p->max_wz = 1.4f;
    p->max_vz = 3.5f;
    p->kp_yaw = 2.0f;

    /* RiskCfg */
    p->vehicle_radius_m = 0.7f;
    p->max_decel_mps2 = 4.5f;
    p->stop_margin_m = 2.0f;
    p->curvature_k = 0.9f;

    /* AvoidCfg */
    p->safe_m = 5.0f;
    p->front_deg = 5.0f;
    p->side_deg = 30.0f;

    /* EventDecisionCfg */
    p->v_cap_frac = 0.75f;
    p->sidestep_deg = 110.0f;
    p->sidestep_speed_frac = 0.35f;
    p->sudden_obj_radius_m = 1.2f;
    p->sudden_obj_clearance_m = 0.3f;

    /* AlgoTuning: DWA (mirrors nav_algorithm.py's ApeAlgoCfg live
     * defaults -- sized to keep this tier's real measured cost "medium",
     * see that dataclass's docstring for why). */
    p->dwa_n_v = 3;
    p->dwa_n_w = 3;
    p->dwa_dt = 0.3f;
    p->dwa_horizon_s = 0.6f;
    p->dwa_w_clear = 0.4f;
    p->dwa_w_heading = 0.4f;
    p->dwa_w_speed = 0.2f;

    /* AlgoTuning: VFH */
    p->vfh_n_sectors = 36;
    p->vfh_threshold = 0.3f;
    p->vfh_smax_sectors = 6.0f;

    /* AlgoTuning: moving-threat handling. Populated with all
     * APE_MAX_THREATS slots active (a plausible worst case -- multiple
     * concurrent threats, ThreatManager's max_active_threats default is
     * 3) so the measured cycle count reflects real threat-handling work,
     * not the cheaper zero-threat path. */
    p->dwa_w_threat = 0.5f;
    p->vfh_w_threat = 0.5f;
    p->vfh_threat_horizon_s = 2.0f;
    for (int32_t i = 0; i < APE_MAX_THREATS; i++) {
        p->threats[i].active = 1;
        p->threats[i].range_m = 6.0f + (float)i * 2.0f;
        p->threats[i].bearing_rad = -0.4f + (float)i * 0.4f;
        p->threats[i].closing_speed_mps = 10.0f - (float)i * 2.0f;
        p->threats[i].radius_m = 0.8f;
    }
    p->n_threats = APE_MAX_THREATS;
}

#if APE_BENCH_MODE == 1 && (APE_BENCH_TARGET == 2 || APE_BENCH_TARGET == 3)
static ape_search_state_t g_search_state;

/* Pre-populates the grid with a plausible mid-mission mix of free/
 * unknown/occupied cells (not an empty grid, which would understate real
 * search compute -- e.g. APE3's frontier lookup is cheapest when every
 * cell is already known). Deterministic, no RNG needed for a fixed
 * benchmark fixture. */
static void build_search_state(ape_search_state_t *s, int32_t grid_w, int32_t grid_h, float cell_m) {
    memset(s->cells, 0, sizeof(s->cells));
    s->grid_w = grid_w;
    s->grid_h = grid_h;
    s->cell_size_m = cell_m;
    s->origin_x = SEARCH_ORIGIN_X;
    s->origin_y = SEARCH_ORIGIN_Y;
    s->initialized = 1;
    for (int32_t i = 0; i < grid_w * grid_h; i++) {
        s->cells[i] = (uint8_t)(i % 3);  /* cycles UNKNOWN(0)/FREE(1)/OCCUPIED(2) */
    }
}
#endif

int main(void) {
    ape_params_t params;
    build_fixture(&params);

#if APE_BENCH_MODE == 1 && APE_BENCH_TARGET == 2
    build_search_state(&g_search_state, SEARCH_GRID_W_APE2, SEARCH_GRID_H_APE2, SEARCH_CELL_M_APE2);
#elif APE_BENCH_MODE == 1 && APE_BENCH_TARGET == 3
    build_search_state(&g_search_state, SEARCH_GRID_W_APE3, SEARCH_GRID_H_APE3, SEARCH_CELL_M_APE3);
#endif

    volatile float sink = 0.0f;

    m5_reset_stats(0, 0);
    for (int32_t i = 0; i < ITERATIONS; i++) {
#if APE_BENCH_TARGET == 1
        ape_result_t r = ape1_bug_plan(&params);
#elif APE_BENCH_TARGET == 2
        /* APE2 = DWA (native_api.c's ape_native_plan_ape2). */
#if APE_BENCH_MODE == 1
        ape_result_t r = ape2_dwa_plan(&params, &g_search_state);
#else
        ape_result_t r = ape2_dwa_plan(&params, NULL);
#endif
#elif APE_BENCH_TARGET == 3
        /* APE3 = VFH (native_api.c's ape_native_plan_ape3). */
#if APE_BENCH_MODE == 1
        ape_result_t r = ape3_vfh_plan(&params, &g_search_state);
#else
        ape_result_t r = ape3_vfh_plan(&params, NULL);
#endif
#else
#error "APE_BENCH_TARGET must be 1, 2, or 3"
#endif
        /* Prevent the compiler from proving the loop body is dead. */
        sink += r.v + r.wz + r.vz + r.score;
    }
    m5_dump_stats(0, 0);

    printf("bench_ape%d mode=%d: iterations=%d sink=%f\n",
           APE_BENCH_TARGET, APE_BENCH_MODE, ITERATIONS, (double)sink);
    return 0;
}
