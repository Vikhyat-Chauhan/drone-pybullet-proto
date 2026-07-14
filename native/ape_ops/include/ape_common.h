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

/*
 * Maps a world-frame point into an ape_search_state_t cell index
 * (row-major, cells[y*grid_w + x]). Returns -1 if the state is
 * uninitialized/unsized or the point falls outside the grid -- callers
 * must check for -1 before indexing cells[].
 */
int32_t ape_grid_index(const ape_search_state_t *s, float x, float y);

/*
 * Time (seconds) until a sensed threat's range would reach zero if its
 * range keeps shrinking at the currently-sensed closing_speed_mps -- a
 * straight-line, constant-rate-of-closure model, the only motion the
 * scalar range/bearing/closing_speed ABI (ape_threat_t) supports without
 * carrying a full 2D obstacle velocity vector. Returns a large sentinel
 * (not a literal "never" guarantee) when the threat isn't closing.
 */
float ape_threat_time_to_collision(float range_m, float closing_speed_mps);

/*
 * Half-angle (radians) a threat of the given radius subtends at the
 * given range, inflated by the vehicle's own radius -- used to smear a
 * threat's danger across nearby histogram sectors/candidate headings.
 */
float ape_threat_angular_half_width(float range_m, float radius_m, float vehicle_radius_m);
