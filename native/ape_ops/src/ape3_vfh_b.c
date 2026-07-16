/*
 * ape3_vfh.c — real Vector Field Histogram (Borenstein & Koren, "The Vector
 * Field Histogram — Fast Obstacle Avoidance for Mobile Robots", IEEE
 * Trans. Robotics & Automation, 1991), with a single-pass valley search.
 * See ape3_vfh.h for references and scoping notes.
 *
 * Op-count discipline: histogram build is one fixed pass over layer 0's
 * n_ranges rays only (this tier's cheap/single-layer scan — see
 * native_api.c's dispatch comment; every ray contributes some computation
 * regardless of its value); valley search is one fixed linear pass over
 * vfh_n_sectors bins. Neither loop's iteration count depends on scan
 * content, only on config (n_ranges/vfh_n_sectors) — keeping a single
 * offline gem5 measurement valid for every future invocation. The final
 * stopping check below is the one place this planner still consults every
 * LiDAR layer directly (cheap, O(n_layers), for the immediate safety cap).
 */
#include "ape3_vfh.h"
#include "ape_common.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MAX_SECTORS 128

/* Borenstein & Koren certainty weighting: c = a - b*d, clamped to
 * [0, a]. a/b chosen so certainty reaches 0 at CERTAINTY_ZERO_RANGE_M —
 * a modeler's tuning choice, not a datasheet figure. */
#define CERTAINTY_A            1.0f
#define CERTAINTY_ZERO_RANGE_M 8.0f
#define CERTAINTY_B            (CERTAINTY_A / CERTAINTY_ZERO_RANGE_M)

ape_result_t ape3_vfh_plan(const ape_params_t *p) {
    ape_result_t r = {0};

    int32_t n_sectors = p->vfh_n_sectors;
    if (n_sectors < 2) n_sectors = 2;
    if (n_sectors > MAX_SECTORS) n_sectors = MAX_SECTORS;

    float scan_span = p->n_ranges * p->angle_increment;   /* total angular span, rad */
    float sector_span = scan_span / (float)n_sectors;

    float density[MAX_SECTORS];
    float min_range[MAX_SECTORS];
    for (int32_t s = 0; s < n_sectors; s++) {
        density[s] = 0.0f;
        min_range[s] = p->range_max + 1.0f;
    }

    /* --- Histogram build: one fixed pass over layer 0's rays only --- */
    {
        const float *row = p->ranges;   /* layer 0 — this tier's single-layer scan */
        for (int32_t i = 0; i < p->n_ranges; i++) {
            float d = row[i];
            float valid = (isfinite(d) && d > 0.0f) ? 1.0f : 0.0f;
            float d_clamped = valid ? d : (CERTAINTY_ZERO_RANGE_M + 1.0f);

            float angle = p->angle_min + (float)i * p->angle_increment;
            int32_t s = (int32_t)((angle - p->angle_min) / sector_span);
            if (s < 0) s = 0;
            if (s >= n_sectors) s = n_sectors - 1;

            float certainty = CERTAINTY_A - CERTAINTY_B * d_clamped;
            if (certainty < 0.0f) certainty = 0.0f;
            density[s] += valid * certainty;

            if (valid && d < min_range[s]) min_range[s] = d;
        }
    }

    /* Goal sector: where yaw_err (already the bearing-to-target error,
     * body frame) falls in the histogram. */
    float goal_angle = ape_wrap_pi(p->yaw_err);
    int32_t goal_sector = (int32_t)((goal_angle - p->angle_min) / sector_span);
    if (goal_sector < 0) goal_sector = 0;
    if (goal_sector >= n_sectors) goal_sector = n_sectors - 1;

    float half_smax = p->vfh_smax_sectors / 2.0f;
    if (half_smax < 0.5f) half_smax = 0.5f;

    /* --- Valley search: one fixed linear pass, evaluate each run as it closes --- */
    float best_score = -1e30f;
    int32_t best_target_sector = goal_sector;
    float best_clearance = 0.0f;
    int32_t found_valley = 0;

    int32_t run_start = -1;
    for (int32_t s = 0; s <= n_sectors; s++) {
        int32_t blocked = (s < n_sectors) ? (density[s] > p->vfh_threshold) : 1; /* sentinel close at end */
        if (!blocked) {
            if (run_start < 0) run_start = s;
        } else if (run_start >= 0) {
            int32_t vs = run_start, ve = s - 1;
            float target_f = ape_clampf((float)goal_sector, (float)vs + half_smax, (float)ve - half_smax);
            int32_t target_sector = (int32_t)(target_f + 0.5f);
            if (target_sector < vs) target_sector = vs;
            if (target_sector > ve) target_sector = ve;

            float width = (float)(ve - vs + 1);
            float width_score = ape_clampf(width / (2.0f * p->vfh_smax_sectors), 0.0f, 1.0f);
            float align_err = fabsf((float)(target_sector - goal_sector)) / (float)n_sectors;
            float align_score = 1.0f - ape_clampf(align_err, 0.0f, 1.0f);
            float clr = min_range[target_sector];
            float clearance_score = ape_clampf(clr / p->range_max, 0.0f, 1.0f);

            float score = 0.4f * align_score + 0.3f * width_score + 0.3f * clearance_score;
            if (score > best_score) {
                best_score = score;
                best_target_sector = target_sector;
                best_clearance = clr;
                found_valley = 1;
            }
            run_start = -1;
        }
    }

    float target_angle = p->angle_min + ((float)best_target_sector + 0.5f) * sector_span;
    float wz = ape_clampf(p->kp_yaw * ape_wrap_pi(target_angle), -p->max_wz, p->max_wz);

    float align_conf = 1.0f - ape_clampf(fabsf((float)(best_target_sector - goal_sector)) / (float)n_sectors, 0.0f, 1.0f);
    float clear_conf = ape_clampf(best_clearance / p->range_max, 0.0f, 1.0f);
    float conf = ape_clampf(0.5f * clear_conf + 0.5f * align_conf, 0.0f, 1.0f);

    /* Speed cap uses v_cap_frac (the same event-response ceiling APE1/
     * APE2 use); real VFH is meant to afford
     * a full-quality plan, not an inherited sidestep-speed throttle. */
    float v_cap_eff = ape_clampf(p->v_cap_frac + 0.2f * conf, 0.0f, 0.95f);
    float base_v = ape_clampf(p->v_cmd, 0.0f, v_cap_eff * p->max_v);
    float curv_k_eff = (p->curvature_k > 0.05f ? p->curvature_k : 0.05f) * (1.0f - 0.4f * conf);
    if (curv_k_eff < 0.05f) curv_k_eff = 0.05f;
    float curv_cap = p->max_v / (1.0f + curv_k_eff * fabsf(wz));

    float v = base_v;
    if (curv_cap < v) v = curv_cap;

    /* Multi-layer front clearance for the stopping-distance cap — the
     * one place all layers, not just the histogram's per-sector min,
     * get consulted directly for the immediate safety check. */
    float front_half = (p->front_deg > 5.0f) ? p->front_deg : 5.0f;
    float d_front = p->range_max + 1.0f;
    for (int32_t layer = 0; layer < p->n_layers; layer++) {
        float d = ape_sector_min(p, layer, 0.0f, front_half);
        if (d < d_front) d_front = d;
    }
    v = ape_stopping_limited_speed(v, d_front, p->max_decel_mps2, p->stop_margin_m);

    float ds = p->sudden_obj_radius_m + p->vehicle_radius_m + p->sudden_obj_clearance_m;
    float score = 0.12f * ds - 0.04f * fabsf(wz) + 0.02f * v;

    r.v = v;
    r.wz = wz;
    r.vz = 0.0f;
    r.score = score;
    r.ok = found_valley ? 1 : 0;
    return r;
}
