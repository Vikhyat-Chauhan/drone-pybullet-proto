/*
 * ape1_bug.h — APE1: minimal reactive potential-field / Bug-style
 * planner. Cheapest of the three tiers by design: closed-form, three
 * small sector-min lookups (front/left/right), no candidate grid, no
 * search. Operates on layer 0 (horizontal plane) only.
 */
#pragma once

#include "ape_types.h"

ape_result_t ape1_bug_plan(const ape_params_t *p);
