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

/* Bounded set of currently sensor-visible moving threats (see
 * sim_adapters.ThreatSensorProvider) fed into every plan call. Fixed
 * size, always iterated in full (branch on .active) rather than a
 * variable-length array -- preserves the content-independent fixed
 * trip-count discipline the other loops in this file's planners rely on
 * for a single offline gem5 measurement to stay valid. */
#define APE_MAX_THREATS 3

typedef struct {
    int32_t active;
    float range_m;
    float bearing_rad;         /* body frame, 0=ahead, + = left (matches yaw_err convention) */
    float closing_speed_mps;   /* + = approaching */
    float radius_m;
} ape_threat_t;

typedef struct {
    /* Raw scan data, flat array: layer-major, then angle-minor
     * (ranges[layer * n_ranges + angle_idx]). n_layers == 1 for
     * APE1/APE2 (horizontal-plane only); up to 5 for APE3 (multi-layer,
     * fed from the PointCloud2-derived per-layer conversion — see
     * nav_algorithm_T.py's _CloudSub / _build_ape_params). */
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
    /* 0 while the search-and-rescue target is still unknown -- yaw_err
     * carries no real goal bearing in that case (nav_algorithm.py passes
     * a harmless placeholder), so each planner must ignore its
     * goal-directed scoring term and fall back to its own
     * exploration/search heuristic instead. 1 once detected, meaning
     * yaw_err is real and today's goal-seeking behavior applies
     * unchanged. */
    int32_t target_detected;

    /* Absolute drone pose, world frame -- only needed for indexing an
     * ape_search_state_t grid (search-and-rescue memory, see below);
     * avoidance never needed absolute position, only body-frame yaw_err
     * and the body-frame scan, so this is new alongside target_detected. */
    float drone_x, drone_y, drone_yaw;

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

    /* APE2 / Dynamic Window Approach */
    int32_t dwa_n_v, dwa_n_w;   /* candidate grid resolution */
    float dwa_dt, dwa_horizon_s;
    float dwa_w_clear, dwa_w_heading, dwa_w_speed;

    /* APE3 / Vector Field Histogram */
    int32_t vfh_n_sectors;      /* polar histogram resolution */
    float vfh_threshold;        /* obstacle-density threshold for "blocked" */
    float vfh_smax_sectors;     /* max valley width considered "wide" */

    /* Moving threats -- sensor-visible only (ThreatSensorProvider),
     * sorted by ascending range, truncated to APE_MAX_THREATS. */
    ape_threat_t threats[APE_MAX_THREATS];
    int32_t n_threats;
    float dwa_w_threat;         /* APE2/DWA: moving-threat rollout-collision weight */
    float vfh_w_threat;         /* APE3/VFH: moving-threat valley-penalty weight */
    float vfh_threat_horizon_s; /* APE3/VFH: time-to-collision beyond which a threat is ignored */
} ape_params_t;

typedef struct {
    float v, wz, vz;
    float score;
    int32_t ok;   /* 0 if no admissible action was found */
} ape_result_t;

/* Search-and-rescue persistent memory, owned by Python (nav_algorithm.py,
 * one instance per APE2/APE3 per mission, allocated/reset in
 * begin_mission()) and passed by pointer into every ape_native_plan_ape2/
 * _ape3 call for the lifetime of that mission -- APE1 never receives one
 * (no memory, by design; see native_api.h). Fixed-size (no malloc inside
 * native code, matching this project's "no hidden dynamic alloc on the
 * flight controller" posture): a caller-chosen grid_w x grid_h window
 * within the APE_GRID_MAX_CELLS cap, anchored at (origin_x, origin_y)
 * with cell_size_m per cell. APE2 (ape2_dwa.c, full memory) treats
 * cells[] as an occupancy grid (0=unknown, 1=free, 2=occupied); APE3
 * (ape3_vfh.c, partial memory) treats it as a saturating visited-count
 * bitmap. Reset via ape_native_search_state_reset(). */
#define APE_GRID_MAX_CELLS 4096

typedef struct {
    int32_t initialized;
    int32_t grid_w, grid_h;       /* 0 = unused/unset */
    float cell_size_m;
    float origin_x, origin_y;     /* world-frame coords of cell (0,0)'s corner */
    uint8_t cells[APE_GRID_MAX_CELLS];
} ape_search_state_t;
