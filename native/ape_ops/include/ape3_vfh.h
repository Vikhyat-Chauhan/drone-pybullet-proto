/*
 * ape3_vfh.h — Vector Field Histogram (Borenstein & Koren, "The Vector Field
 * Histogram — Fast Obstacle Avoidance for Mobile Robots", IEEE Trans.
 * Robotics & Automation, 1991), followed by a single-pass valley search
 * that selects and scores the best candidate heading -- the "search
 * over a local structure" step intended to restore the paper's "Map
 * Search" framing.
 *
 * This is the cheap/single-layer tier (see native_api.c's dispatch
 * comment): the histogram is built from layer 0 only. The one place all
 * vertical LiDAR layers are still consulted is the final front-clearance
 * stopping check, a cheap O(n_layers) pass used as the immediate safety
 * cap on speed, not for the histogram/valley search itself.
 *
 * Histogram build and valley search are both single fixed-size linear
 * passes (no early exit, no recursion) so the op count is independent
 * of scan content — see ape_ops's gem5 methodology notes.
 */
#pragma once

#include "ape_types.h"

ape_result_t ape3_vfh_plan(const ape_params_t *p);