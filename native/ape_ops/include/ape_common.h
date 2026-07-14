/*
 * ape_common.h — small shared helpers used by all three APE planners
 * (ape1_bug.c / ape2_dwa.c / ape3_vfh.c). Kept deliberately tiny: index
 * math, sector-min lookups, wrap/clamp, and the stopping-distance speed
 * cap — all pure, side-effect-free, O(sector size) or O(1).
 */
#pragma once

#include "ape_types.h"

/* Wraps an angle (radians) to [-pi, pi]. */
float ape_wrap_pi(float a);

float ape_clampf(float v, float lo, float hi);

/*
 * Minimum finite, positive range within [center_deg - half_deg,
 * center_deg + half_deg] for the given layer (0-based index into
 * p->ranges). Returns p->range_max + 1.0f ("clear") if no valid sample
 * falls in the window or the scan is empty/invalid.
 */
float ape_sector_min(const ape_params_t *p, int32_t layer, float center_deg, float half_deg);

/*
 * Caps v_des so the vehicle can still stop before dmin, given
 * max_decel_mps2 and stop_margin_m. Returns 0 if dmin is already at or
 * inside the stop margin.
 */
float ape_stopping_limited_speed(float v_des, float dmin, float max_decel_mps2, float stop_margin_m);
