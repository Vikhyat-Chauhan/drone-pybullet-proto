/*
 * ape2_dwa.h — APE2: Dynamic Window Approach (Fox, Burgard & Thrun,
 * "The Dynamic Window Approach to Collision Avoidance", IEEE Robotics &
 * Automation Magazine, 1997), scoped to a single 2D LiDAR scan (layer 0,
 * horizontal plane only) rather than a full 2D costmap.
 *
 * Samples a fixed dwa_n_v x dwa_n_w grid of (v, w) candidates, forward-
 * simulates each with a unicycle kinematic model, scores by weighted
 * clearance + goal-heading alignment + speed, picks the best. The full
 * grid is always evaluated (no early exit on data) so its cost is
 * independent of scan content — see ape_ops's gem5 methodology notes.
 */
#pragma once

#include "ape_types.h"

/*
 * state: search-and-rescue occupancy memory (full tier -- see
 * ape_types.h's ape_search_state_t doc). May be NULL (e.g. avoidance-
 * path calls that don't carry search state); when target_detected is 0
 * and state is non-NULL, candidate scoring's heading term rewards
 * frontier cells (unknown in the grid) over already-seen ones instead of
 * chasing a goal bearing. Updated in place from the current scan every
 * call.
 */
ape_result_t ape2_dwa_plan(const ape_params_t *p, ape_search_state_t *state);
