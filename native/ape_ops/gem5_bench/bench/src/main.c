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
 * (bench_ape1/bench_ape2/bench_ape3) — see ../Makefile.
 *
 * Fixture: one fixed "open corridor" LiDAR-style scan, built once before
 * the ROI loop starts (setup cost is deliberately outside m5_reset_stats).
 * Geometry/config values mirror this repo's live defaults (Lidar2D in
 * sim/lidar.py: num_rays=48, fov_deg=300, max_range=15.0; GoToConfig/
 * AvoidCfg/RiskCfg/EventDecisionCfg/AlgoTuning in nav/algorithm.py) so
 * the measured op count reflects a realistic invocation, not an
 * arbitrary one. All three planners' op counts are independent of scan
 * *content* (see each planner's own header comment), so the exact
 * range values only need to be plausible, not tuned per-planner.
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

#define ITERATIONS 200

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
    p->yaw_err = 0.15f;

    /* GoToConfig (nav/algorithm.py) */
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

    /* AlgoTuning: DWA (mirrors nav/algorithm.py's ApeAlgoCfg live
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
}

int main(void) {
    ape_params_t params;
    build_fixture(&params);

    volatile float sink = 0.0f;

    m5_reset_stats(0, 0);
    for (int32_t i = 0; i < ITERATIONS; i++) {
#if APE_BENCH_TARGET == 1
        ape_result_t r = ape1_bug_plan(&params);
#elif APE_BENCH_TARGET == 2
        /* APE2 = VFH (native_api.c's ape_native_plan_ape2). */
        ape_result_t r = ape3_vfh_plan(&params);
#elif APE_BENCH_TARGET == 3
        /* APE3 = DWA (native_api.c's ape_native_plan_ape3). */
        ape_result_t r = ape2_dwa_plan(&params);
#else
#error "APE_BENCH_TARGET must be 1, 2, or 3"
#endif
        /* Prevent the compiler from proving the loop body is dead. */
        sink += r.v + r.wz + r.vz + r.score;
    }
    m5_dump_stats(0, 0);

    printf("bench_ape%d: iterations=%d sink=%f\n", APE_BENCH_TARGET, ITERATIONS, (double)sink);
    return 0;
}
