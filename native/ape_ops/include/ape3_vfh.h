/*
 * ape3_vfh.h — APE3: Vector Field Histogram (Borenstein & Koren, "The
 * Vector Field Histogram — Fast Obstacle Avoidance for Mobile Robots",
 * IEEE Trans. Robotics & Automation, 1991), extended to combine all
 * vertical LiDAR layers (multi-layer consensus: a sector is blocked if
 * any layer sees a close obstacle there) and followed by a single-pass
 * valley search that selects and scores the best candidate heading —
 * the "search over a local structure" step intended to restore the
 * paper's "Map Search" framing for APE3, the best-quality/most-expensive
 * tier.
 *
 * Histogram build and valley search are both single fixed-size linear
 * passes (no early exit, no recursion) so the op count is independent
 * of scan content — see ape_ops's gem5 methodology notes.
 */
#pragma once

#include "ape_types.h"

/*
 * state: search-and-rescue visited-cell memory (partial tier -- coarser
 * than ape2_dwa's occupancy grid, see ape_types.h's ape_search_state_t
 * doc). May be NULL. When target_detected is 0 and state is non-NULL,
 * valley scoring gets a least-visited bonus biasing exploration away
 * from already-covered ground. Updated in place (current drone cell
 * marked visited) every call.
 */
ape_result_t ape3_vfh_plan(const ape_params_t *p, ape_search_state_t *state);
