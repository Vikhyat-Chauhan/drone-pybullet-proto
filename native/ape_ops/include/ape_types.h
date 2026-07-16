/*
 * ape_types.h — shared parameter/result contract for the three real APE
 * planners (ape1_bug.c / ape2_dwa.c / ape3_vfh.c).
 *
 * This struct is the ONLY channel between Python and the native planners
 * (ape_native.py, repo root, mirrors it field-for-field as a
 * ctypes.Structure). Config values are passed fresh on every call from
 * Python's existing dataclasses (GoToConfig/AvoidCfg/RiskCfg/
 * EventDecisionCfg in nav_algorithm.py) rather than duplicated as C
 * constants — this is deliberate, to avoid the exact class of drift bug
 * (stale hand-mirrored values) found and fixed twice earlier in this
 * project's history (see docs/POWER_MODEL.md).
 *
 * ABI NOTE: field order/types here must match ApeParams/ApeResult in
 * ape_native.py exactly. Verified at runtime via ape_native_sizeof_params()/
 * ape_native_sizeof_result() cross-checked against ctypes.sizeof() — do not
 * rely on visual inspection alone when changing this file.
 */
#pragma once

#include <stdint.h>

typedef struct {
    /* Raw scan data, flat array: layer-major, then angle-minor
     * (ranges[layer * n_ranges + angle_idx]). n_layers == 1 for the
     * APE1/APE2 tiers (horizontal-plane only); up to 5 for the APE3 tier
     * (multi-layer, fed from the PointCloud2-derived per-layer
     * conversion — see algorithm.py's _build_ape_params). Which
     * algorithm (DWA/VFH) runs in which tier is decided by native_api.c's
     * dispatch, independent of this layer-count contract. */
    const float *ranges;
    int32_t n_ranges;          /* per-layer angle sample count */
    int32_t n_layers;
    float angle_min;
    float angle_increment;
    float vertical_angle_min;
    float vertical_angle_increment;
    float range_min;
    float range_max;

    /* Scalar nav state, from the event snapshot */
    float v_cmd;
    float yaw_err;              /* rad, wrapped to [-pi, pi] */

    /* Config, sourced fresh from Python dataclasses every call */
    float max_v, max_wz, max_vz;
    float kp_yaw;
    float vehicle_radius_m;
    float max_decel_mps2, stop_margin_m;
    float safe_m, front_deg, side_deg;
    float v_cap_frac;
    float sidestep_deg, sidestep_speed_frac;
    float sudden_obj_radius_m, sudden_obj_clearance_m;
    float curvature_k;

    /* Bug-style reactive planner (ape1_bug.c) */
    int32_t bug_oversample_n;  /* repeats each sector-min lookup N times
                                   (averaged; scan is frozen for the tick,
                                   so this scales compute cost only, not
                                   the result) */

    /* Dynamic Window Approach (dwa.c) */
    int32_t dwa_n_v, dwa_n_w;   /* candidate grid resolution */
    float dwa_dt, dwa_horizon_s;
    float dwa_w_clear, dwa_w_heading, dwa_w_speed;

    /* Vector Field Histogram (vfh.c) */
    int32_t vfh_n_sectors;      /* polar histogram resolution */
    float vfh_threshold;        /* obstacle-density threshold for "blocked" */
    float vfh_smax_sectors;     /* max valley width considered "wide" */

    /* No-fly zones, in the same ego/body frame (x-forward, y-left) as the
     * planners' own kinematic simulation -- flat, 4 floats/rect:
     * (xmin, ymin, xmax, ymax), AABB of the zone already rotated+translated
     * into ego frame by the caller. Only vfh.c (RHTP) consumes these today;
     * bug.c/dwa.c ignore them. */
    const float *nofly_rects_ego;
    int32_t n_nofly_rects;
} ape_params_t;

typedef struct {
    float v, wz, vz;
    float score;
    int32_t ok;   /* 0 if no admissible action was found */
} ape_result_t;
