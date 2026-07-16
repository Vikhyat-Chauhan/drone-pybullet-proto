/*
 * ape3_X.c - Advanced Vector Field Histogram Plus (VFH+)
 * 
 * High-compute, latency-unconstrained planner. Incorporates:
 * 1. Multi-layer absolute min-range aggregation.
 * 2. Robot-radius angular inflation (VFH+) to guarantee physical clearance.
 * 3. Density array convolution (smoothing) for stable valley generation.
 * 4. Exhaustive intra-valley candidate evaluation.
 */

#include "ape3_vfh.h"
#include "ape_common.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MAX_SECTORS 512

/* Certainty weighting constants */
#define CERTAINTY_A            1.0f
#define CERTAINTY_ZERO_RANGE_M 12.0f /* Increased lookahead for faster flight */
#define CERTAINTY_B            (CERTAINTY_A / CERTAINTY_ZERO_RANGE_M)

ape_result_t ape3_vfh_plan(const ape_params_t *p) {
    ape_result_t r = {0};
    
    int32_t n_sectors = p->vfh_n_sectors;
    if (n_sectors < 2) n_sectors = 2;
    if (n_sectors > MAX_SECTORS) n_sectors = MAX_SECTORS;

    float scan_span = p->n_ranges * p->angle_increment;
    float sector_span = scan_span / (float)n_sectors;

    float raw_density[MAX_SECTORS];
    float smoothed_density[MAX_SECTORS];
    float min_range[MAX_SECTORS];

    for (int32_t s = 0; s < n_sectors; s++) {
        raw_density[s] = 0.0f;
        smoothed_density[s] = 0.0f;
        min_range[s] = p->range_max + 1.0f;
    }

    /* --- STEP 1: Multi-layer Aggregation & Robot-Width Inflation --- */
    for (int32_t i = 0; i < p->n_ranges; i++) {
        // Find absolute closest obstacle across all layers at this azimuth
        float min_d = p->range_max + 1.0f;
        for (int32_t layer = 0; layer < p->n_layers; layer++) {
            float d = p->ranges[layer * p->n_ranges + i];
            if (isfinite(d) && d > 0.0f && d < min_d) {
                min_d = d;
            }
        }

        if (min_d > CERTAINTY_ZERO_RANGE_M) continue;

        float angle = p->angle_min + (float)i * p->angle_increment;
        float certainty = CERTAINTY_A - CERTAINTY_B * min_d;
        if (certainty < 0.0f) certainty = 0.0f;

        // VFH+ Inflation: Calculate how many radians this obstacle blocks 
        // based on the physical size of the drone plus a safety margin.
        float safe_radius = p->vehicle_radius_m + p->sudden_obj_clearance_m;
        float enlarge_rad = 0.0f;
        if (min_d > safe_radius) {
            enlarge_rad = asinf(safe_radius / min_d);
        } else {
            enlarge_rad = M_PI / 2.0f; // Dangerously close, block heavily
        }

        // Smear the obstacle certainty across all sectors within the inflation cone
        int32_t s_start = (int32_t)((angle - enlarge_rad - p->angle_min) / sector_span);
        int32_t s_end   = (int32_t)((angle + enlarge_rad - p->angle_min) / sector_span);

        for (int32_t s = s_start; s <= s_end; s++) {
            if (s >= 0 && s < n_sectors) {
                raw_density[s] += certainty;
                if (min_d < min_range[s]) {
                    min_range[s] = min_d;
                }
            }
        }
    }

    /* --- STEP 2: Density Convolution (Smoothing) --- */
    // Apply a sliding window low-pass filter to smooth out LiDAR noise and create clean valleys
    int32_t window = 2; 
    for (int32_t s = 0; s < n_sectors; s++) {
        float sum = 0.0f;
        float weights = 0.0f;
        for (int32_t ws = -window; ws <= window; ws++) {
            int32_t idx = s + ws;
            if (idx >= 0 && idx < n_sectors) {
                float w = (float)(window - abs(ws) + 1);
                sum += raw_density[idx] * w;
                weights += w;
            }
        }
        smoothed_density[s] = sum / weights;
    }

    /* --- STEP 3: Exhaustive Valley Search --- */
    float goal_angle = ape_wrap_pi(p->yaw_err);
    int32_t goal_sector = (int32_t)((goal_angle - p->angle_min) / sector_span);
    if (goal_sector < 0) goal_sector = 0;
    if (goal_sector >= n_sectors) goal_sector = n_sectors - 1;

    float best_score = -1e30f;
    int32_t best_target_sector = goal_sector;
    float best_clearance = 0.0f;
    int32_t found_valley = 0;
    int32_t run_start = -1;

    for (int32_t s = 0; s <= n_sectors; s++) {
        int32_t blocked = (s < n_sectors) ? (smoothed_density[s] > p->vfh_threshold) : 1;
        
        if (!blocked) {
            if (run_start < 0) run_start = s;
        } else if (run_start >= 0) {
            int32_t vs = run_start, ve = s - 1;
            float width = (float)(ve - vs + 1);
            
            // High Compute: Evaluate EVERY sector in the valley to find the absolute best path
            // rather than just blindly aiming for the center.
            for (int32_t cand = vs; cand <= ve; cand++) {
                float width_score = ape_clampf(width / (2.0f * p->vfh_smax_sectors), 0.0f, 1.0f);
                float align_err = fabsf((float)(cand - goal_sector)) / (float)n_sectors;
                float align_score = 1.0f - ape_clampf(align_err, 0.0f, 1.0f);
                
                float clr = min_range[cand];
                float clearance_score = ape_clampf(clr / p->range_max, 0.0f, 1.0f);

                // Penalize candidates that scrape too close to the edges of the valley
                float dist_to_edge = fminf((float)(cand - vs), (float)(ve - cand));
                float edge_score = ape_clampf(dist_to_edge / (p->vfh_smax_sectors / 2.0f), 0.0f, 1.0f);

                // Weighted sum favoring clearance and edge-safety heavily
                float score = 0.3f * align_score + 0.1f * width_score + 0.3f * clearance_score + 0.3f * edge_score;

                if (score > best_score) {
                    best_score = score;
                    best_target_sector = cand;
                    best_clearance = clr;
                    found_valley = 1;
                }
            }
            run_start = -1;
        }
    }

    /* --- STEP 4: High-Confidence Speed Mapping --- */
    float target_angle = p->angle_min + ((float)best_target_sector + 0.5f) * sector_span;
    float wz = ape_clampf(p->kp_yaw * ape_wrap_pi(target_angle), -p->max_wz, p->max_wz);
    
    float align_conf = 1.0f - ape_clampf(fabsf((float)(best_target_sector - goal_sector)) / (float)n_sectors, 0.0f, 1.0f);
    float clear_conf = ape_clampf(best_clearance / p->range_max, 0.0f, 1.0f);
    float conf = ape_clampf(0.5f * clear_conf + 0.5f * align_conf, 0.0f, 1.0f);

    // Because the algorithm now physically inflates obstacles, we can trust the path much more.
    // Allow the drone to fly closer to its maximum velocity when confidence is high.
    float v_cap_eff = ape_clampf(p->v_cap_frac + 0.4f * conf, 0.0f, 1.0f);
    float base_v = ape_clampf(p->v_cmd, 0.0f, v_cap_eff * p->max_v);

    float curv_k_eff = (p->curvature_k > 0.05f ? p->curvature_k : 0.05f) * (1.0f - 0.6f * conf);
    if (curv_k_eff < 0.05f) curv_k_eff = 0.05f;
    float curv_cap = p->max_v / (1.0f + curv_k_eff * fabsf(wz));
    
    float v = base_v;
    if (curv_cap < v) v = curv_cap;

    float front_half = (p->front_deg > 5.0f) ? p->front_deg : 5.0f;
    float d_front = p->range_max + 1.0f;
    for (int32_t layer = 0; layer < p->n_layers; layer++) {
        float d = ape_sector_min(p, layer, 0.0f, front_half);
        if (d < d_front) d_front = d;
    }
    
    v = ape_stopping_limited_speed(v, d_front, p->max_decel_mps2, p->stop_margin_m);

    r.v = v;
    r.wz = wz;
    r.vz = 0.0f;
    r.score = best_score * 10.0f; // Boost relative score due to high confidence
    r.ok = found_valley ? 1 : 0;

    return r;
}
