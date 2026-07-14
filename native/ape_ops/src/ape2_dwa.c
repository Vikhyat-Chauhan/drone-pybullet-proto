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

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MAX_SIM_STEPS   16   /* hard cap; dwa_horizon_s/dwa_dt clamped to this */
#define OBSTACLE_LOOKUP_HALF_DEG 3.0f

/* Forward-simulates one (v,w) candidate for the configured horizon,
 * returning the clearance (meters traveled before predicted collision,
 * capped at the full horizon distance) and the final heading (rad,
 * body frame, starting at 0). */
static void simulate_candidate(const ape_params_t *p, float v, float w,
                                int32_t n_steps, float dt,
                                float *out_clearance, float *out_final_theta) {
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    float clearance = 0.0f;
    int32_t collided = 0;

    for (int32_t s = 0; s < n_steps; s++) {
        float prev_x = x, prev_y = y;
        x += v * cosf(theta) * dt;
        y += v * sinf(theta) * dt;
        theta += w * dt;

        float dist_from_origin = sqrtf(x * x + y * y);
        float bearing_deg = atan2f(y, x) * 180.0f / (float)M_PI;
        float obstacle_range = ape_sector_min(p, 0, bearing_deg, OBSTACLE_LOOKUP_HALF_DEG);

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
    }

    *out_clearance = clearance;
    *out_final_theta = theta;
}

ape_result_t ape2_dwa_plan(const ape_params_t *p) {
    ape_result_t r = {0};

    int32_t n_v = (p->dwa_n_v > 1) ? p->dwa_n_v : 1;
    int32_t n_w = (p->dwa_n_w > 1) ? p->dwa_n_w : 1;
    float dt = (p->dwa_dt > 1e-3f) ? p->dwa_dt : 0.3f;
    int32_t n_steps = (int32_t)(p->dwa_horizon_s / dt + 0.5f);
    if (n_steps < 1) n_steps = 1;
    if (n_steps > MAX_SIM_STEPS) n_steps = MAX_SIM_STEPS;

    float horizon_dist = p->max_v * dt * (float)n_steps;
    if (horizon_dist < 1e-3f) horizon_dist = 1e-3f;

    float best_g = -1e30f;
    float best_v = 0.0f, best_w = 0.0f, best_clearance = 0.0f;

    for (int32_t iv = 0; iv < n_v; iv++) {
        float v = p->max_v * (float)iv / (float)(n_v - 1 > 0 ? n_v - 1 : 1);
        for (int32_t iw = 0; iw < n_w; iw++) {
            float w = -p->max_wz + 2.0f * p->max_wz * (float)iw / (float)(n_w - 1 > 0 ? n_w - 1 : 1);

            float clearance, final_theta;
            simulate_candidate(p, v, w, n_steps, dt, &clearance, &final_theta);

            float clearance_score = ape_clampf(clearance / horizon_dist, 0.0f, 1.0f);
            float heading_err = fabsf(ape_wrap_pi(final_theta - p->yaw_err));
            float heading_score = 1.0f - ape_clampf(heading_err / (float)M_PI, 0.0f, 1.0f);
            float speed_score = ape_clampf(v / (p->max_v > 1e-3f ? p->max_v : 1.0f), 0.0f, 1.0f);

            float g = p->dwa_w_heading * heading_score
                    + p->dwa_w_clear   * clearance_score
                    + p->dwa_w_speed   * speed_score;

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
