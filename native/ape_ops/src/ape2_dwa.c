/*
 * ape2_dwa.c — real Dynamic Window Approach. See ape2_dwa.h for the
 * reference and scoping notes.
 *
 * Op-count discipline: the (v,w) grid and per-candidate simulation step
 * count are both fixed by dwa_n_v/dwa_n_w/dwa_horizon_s/dwa_dt (config,
 * not data) — every candidate is fully simulated and scored regardless
 * of scan content, no early exit. This keeps the op count independent
 * of live sensor data, which is what makes a single offline gem5
 * measurement of this function valid for every future invocation.
 */
#include "ape2_dwa.h"
#include "ape_common.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MAX_SIM_STEPS   16   /* hard cap; dwa_horizon_s/dwa_dt clamped to this */
#define OBSTACLE_LOOKUP_HALF_DEG 3.0f
#define RAYMARCH_STEPS  6    /* bounded per-ray occupancy-grid update steps */

#define CELL_UNKNOWN  0
#define CELL_FREE     1
#define CELL_OCCUPIED 2

/* Forward-simulates one (v,w) candidate for the configured horizon,
 * returning the clearance (meters traveled before predicted collision,
 * capped at the full horizon distance), the final heading (rad, body
 * frame, starting at 0), the final body-frame (x, y), and
 * out_threat_margin_sq -- the smallest predicted *squared* clearance
 * margin (dist^2 - combined_radius^2, can go negative) between the
 * candidate's simulated position and any active moving threat's
 * constant-velocity-extrapolated position, over the whole rollout.
 * Deliberately left squared (no sqrtf in this hot loop -- it runs
 * n_v*n_w*n_steps*APE_MAX_THREATS times per plan call and a literal
 * per-iteration sqrt made this planner's measured cost balloon past
 * VFH's, inverting the intended cost ordering) -- the sign is exactly
 * what a collision reject needs, and a monotonic proxy is enough for
 * scoring. This is the classic Velocity-Obstacles idea (Fiorini &
 * Shiller 1998) applied inside DWA's existing forward simulation: a
 * threat's predicted position at step s is its currently-sensed range
 * shrinking linearly at closing_speed_mps along its currently-sensed
 * (fixed) bearing -- the straight-line-intercept model the scalar
 * range/bearing/closing_speed ABI supports (see
 * ape_threat_time_to_collision's doc). Extra cost: n_steps *
 * APE_MAX_THREATS per candidate, a fixed trip count regardless of scan
 * or threat content. */
static void simulate_candidate(const ape_params_t *p, float v, float w,
                                int32_t n_steps, float dt,
                                const float *threat_cos, const float *threat_sin,
                                float *out_clearance, float *out_final_theta,
                                float *out_final_obstacle_range,
                                float *out_final_x, float *out_final_y,
                                float *out_threat_margin_sq) {
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    float clearance = 0.0f;
    int32_t collided = 0;
    float obstacle_range = p->range_max;
    float threat_margin_sq = 1.0e12f;

    for (int32_t s = 0; s < n_steps; s++) {
        float prev_x = x, prev_y = y;
        x += v * cosf(theta) * dt;
        y += v * sinf(theta) * dt;
        theta += w * dt;

        float dist_from_origin = sqrtf(x * x + y * y);
        float bearing_deg = atan2f(y, x) * 180.0f / (float)M_PI;
        obstacle_range = ape_sector_min(p, 0, bearing_deg, OBSTACLE_LOOKUP_HALF_DEG);

        if (!collided) {
            if (obstacle_range - p->vehicle_radius_m < dist_from_origin) {
                /* Collision predicted this step: clearance is the
                 * distance traveled up to (not including) this step. */
                float step_dist = sqrtf((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y));
                clearance += 0.0f * step_dist; /* don't credit the colliding step */
                collided = 1;
            } else {
                float step_dist = sqrtf((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y));
                clearance += step_dist;
            }
        }

        for (int32_t ti = 0; ti < APE_MAX_THREATS; ti++) {
            const ape_threat_t *th = &p->threats[ti];
            if (!th->active) continue;
            float t_now = (float)(s + 1) * dt;
            float pred_range = th->range_m - th->closing_speed_mps * t_now;
            if (pred_range < 0.0f) pred_range = 0.0f;
            float tx = pred_range * threat_cos[ti];
            float ty = pred_range * threat_sin[ti];
            float dx = x - tx, dy = y - ty;
            float radius_sum = th->radius_m + p->vehicle_radius_m;
            float margin_sq = (dx * dx + dy * dy) - radius_sum * radius_sum;
            if (margin_sq < threat_margin_sq) threat_margin_sq = margin_sq;
        }
    }

    *out_clearance = clearance;
    *out_final_theta = theta;
    *out_final_obstacle_range = obstacle_range;
    *out_final_x = x;
    *out_final_y = y;
    *out_threat_margin_sq = threat_margin_sq;
}

/* Full search memory: marches every ray of the current scan into the
 * occupancy grid in RAYMARCH_STEPS fixed steps (bounded, content-
 * independent cost -- n_ranges * RAYMARCH_STEPS regardless of what the
 * scan returns), marking traversed cells free and each ray's endpoint
 * occupied if it hit something before range_max. This is what makes
 * APE3 the "full memory" tier -- APE2 (ape3_vfh.c) only marks the
 * drone's own current cell, never builds a map of the surroundings. */
static void update_occupancy_grid(const ape_params_t *p, ape_search_state_t *state) {
    float ca_yaw = cosf(p->drone_yaw), sa_yaw = sinf(p->drone_yaw);
    for (int32_t i = 0; i < p->n_ranges; i++) {
        float d = p->ranges[i];
        int32_t hit = (isfinite(d) && d > 0.0f && d < p->range_max);
        float d_capped = hit ? d : p->range_max;

        float angle = p->angle_min + (float)i * p->angle_increment;
        float ca = cosf(angle), sa = sinf(angle);
        /* body-frame ray direction rotated into world frame by drone_yaw */
        float wx = ca * ca_yaw - sa * sa_yaw;
        float wy = ca * sa_yaw + sa * ca_yaw;

        for (int32_t s = 1; s <= RAYMARCH_STEPS; s++) {
            float dist = d_capped * (float)s / (float)RAYMARCH_STEPS;
            float px = p->drone_x + dist * wx;
            float py = p->drone_y + dist * wy;
            int32_t cell = ape_grid_index(state, px, py);
            if (cell < 0) continue;
            if (hit && s == RAYMARCH_STEPS) {
                state->cells[cell] = CELL_OCCUPIED;
            } else if (state->cells[cell] != CELL_OCCUPIED) {
                state->cells[cell] = CELL_FREE;
            }
        }
    }
}

ape_result_t ape2_dwa_plan(const ape_params_t *p, ape_search_state_t *state) {
    ape_result_t r = {0};

    if (state != NULL) update_occupancy_grid(p, state);

    int32_t n_v = (p->dwa_n_v > 1) ? p->dwa_n_v : 1;
    int32_t n_w = (p->dwa_n_w > 1) ? p->dwa_n_w : 1;
    float dt = (p->dwa_dt > 1e-3f) ? p->dwa_dt : 0.3f;
    int32_t n_steps = (int32_t)(p->dwa_horizon_s / dt + 0.5f);
    if (n_steps < 1) n_steps = 1;
    if (n_steps > MAX_SIM_STEPS) n_steps = MAX_SIM_STEPS;

    float horizon_dist = p->max_v * dt * (float)n_steps;
    if (horizon_dist < 1e-3f) horizon_dist = 1e-3f;

    /* Precompute each active threat's bearing cos/sin once (constant
     * across every candidate's rollout, since bearing_rad is a fixed
     * sensed value) instead of re-deriving it n_v*n_w*n_steps times
     * inside simulate_candidate -- keeps the moving-threat check's cost
     * to one trig pair per threat regardless of grid/horizon size. */
    float threat_cos[APE_MAX_THREATS], threat_sin[APE_MAX_THREATS];
    for (int32_t ti = 0; ti < APE_MAX_THREATS; ti++) {
        threat_cos[ti] = cosf(p->threats[ti].bearing_rad);
        threat_sin[ti] = sinf(p->threats[ti].bearing_rad);
    }

    float best_g = -1e30f;
    float best_v = 0.0f, best_w = 0.0f, best_clearance = 0.0f;

    for (int32_t iv = 0; iv < n_v; iv++) {
        float v = p->max_v * (float)iv / (float)(n_v - 1 > 0 ? n_v - 1 : 1);
        for (int32_t iw = 0; iw < n_w; iw++) {
            float w = -p->max_wz + 2.0f * p->max_wz * (float)iw / (float)(n_w - 1 > 0 ? n_w - 1 : 1);

            float clearance, final_theta, final_obstacle_range, final_x, final_y, threat_margin_sq;
            simulate_candidate(p, v, w, n_steps, dt, threat_cos, threat_sin, &clearance, &final_theta,
                                &final_obstacle_range, &final_x, &final_y, &threat_margin_sq);

            float clearance_score = ape_clampf(clearance / horizon_dist, 0.0f, 1.0f);
            /* threat_score in [-1, 1]; a negative predicted squared
             * margin (the rollout comes within the combined radii of a
             * threat's extrapolated path) drives this toward -1 and
             * additionally hard-rejects the candidate below -- this is
             * what makes APE2 sometimes fail to react to a fast/sudden
             * threat: the full n_v*n_w grid (now with this per-candidate
             * threat check folded into the same rollout) still has to
             * finish within its gem5-measured budget, and can
             * occasionally lose that race against a threat whose
             * deadline is tight. */
            float threat_scale_sq = (p->vehicle_radius_m + 1.0f) * (p->vehicle_radius_m + 1.0f);
            float threat_score = ape_clampf(threat_margin_sq / threat_scale_sq, -1.0f, 1.0f);
            /* Search mode (target undetected): no goal to chase, so
             * heading_score instead rewards candidates whose simulated
             * end position lands on a frontier (unknown) cell in the
             * occupancy grid over already-seen ground -- real
             * information-gain-driven exploration, reusing the rollout
             * simulate_candidate() already does. Falls back to the
             * cheaper "most open direction" proxy when no grid is
             * attached (e.g. this planner was called without search
             * state, or the endpoint falls outside the grid). */
            float heading_score;
            if (p->target_detected) {
                float heading_err = fabsf(ape_wrap_pi(final_theta - p->yaw_err));
                heading_score = 1.0f - ape_clampf(heading_err / (float)M_PI, 0.0f, 1.0f);
            } else {
                int32_t fcell = -1;
                if (state != NULL) {
                    float ca_yaw = cosf(p->drone_yaw), sa_yaw = sinf(p->drone_yaw);
                    float world_fx = p->drone_x + final_x * ca_yaw - final_y * sa_yaw;
                    float world_fy = p->drone_y + final_x * sa_yaw + final_y * ca_yaw;
                    fcell = ape_grid_index(state, world_fx, world_fy);
                }
                if (fcell >= 0) {
                    uint8_t c = state->cells[fcell];
                    heading_score = (c == CELL_UNKNOWN) ? 1.0f : (c == CELL_OCCUPIED ? 0.0f : 0.3f);
                } else {
                    heading_score = ape_clampf(final_obstacle_range / p->range_max, 0.0f, 1.0f);
                }
            }
            float speed_score = ape_clampf(v / (p->max_v > 1e-3f ? p->max_v : 1.0f), 0.0f, 1.0f);

            float g = p->dwa_w_heading * heading_score
                    + p->dwa_w_clear   * clearance_score
                    + p->dwa_w_speed   * speed_score
                    + p->dwa_w_threat  * threat_score;
            if (threat_margin_sq < 0.0f) g -= 1000.0f;  /* hard-reject predicted threat collisions */

            if (g > best_g) {
                best_g = g;
                best_v = v;
                best_w = w;
                best_clearance = clearance;
            }
        }
    }

    float wz = ape_clampf(best_w, -p->max_wz, p->max_wz);
    float curv_k = (p->curvature_k > 0.05f) ? p->curvature_k : 0.05f;
    float curv_cap = p->max_v / (1.0f + curv_k * fabsf(wz));
    float v_event_cap = p->v_cap_frac * p->max_v;

    float v = best_v;
    if (v_event_cap < v) v = v_event_cap;
    if (curv_cap < v) v = curv_cap;

    float d_front = ape_sector_min(p, 0, 0.0f, (p->front_deg > 5.0f) ? p->front_deg : 5.0f);
    v = ape_stopping_limited_speed(v, d_front, p->max_decel_mps2, p->stop_margin_m);

    r.v = v;
    r.wz = wz;
    r.vz = 0.0f;
    r.score = best_g;
    r.ok = (best_clearance > 0.0f) ? 1 : 0;
    return r;
}
