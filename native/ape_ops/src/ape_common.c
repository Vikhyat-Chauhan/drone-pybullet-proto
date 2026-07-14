#include "ape_common.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float ape_wrap_pi(float a) {
    float r = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (r < 0.0f) r += 2.0f * (float)M_PI;
    return r - (float)M_PI;
}

float ape_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float deg2rad(float d) { return d * (float)M_PI / 180.0f; }

float ape_sector_min(const ape_params_t *p, int32_t layer, float center_deg, float half_deg) {
    float clear = p->range_max + 1.0f;
    if (p->ranges == NULL || p->n_ranges <= 0 || layer < 0 || layer >= p->n_layers)
        return clear;

    const float *row = p->ranges + (size_t)layer * (size_t)p->n_ranges;
    int32_t lo, hi;

    if (!isfinite(p->angle_increment) || fabsf(p->angle_increment) < 1e-9f) {
        int32_t center_idx = p->n_ranges / 2;
        int32_t half = (int32_t)(half_deg / 90.0f * (float)p->n_ranges);
        if (half < 1) half = 1;
        lo = center_idx - half; if (lo < 0) lo = 0;
        hi = center_idx + half; if (hi > p->n_ranges - 1) hi = p->n_ranges - 1;
    } else {
        float center = deg2rad(center_deg);
        float half = deg2rad(half_deg);
        lo = (int32_t)((center - half - p->angle_min) / p->angle_increment);
        hi = (int32_t)((center + half - p->angle_min) / p->angle_increment);
        if (lo < 0) lo = 0;
        if (lo > p->n_ranges - 1) lo = p->n_ranges - 1;
        if (hi < 0) hi = 0;
        if (hi > p->n_ranges - 1) hi = p->n_ranges - 1;
        if (lo > hi) { int32_t t = lo; lo = hi; hi = t; }
    }

    float best = clear;
    int32_t found = 0;
    for (int32_t i = lo; i <= hi; i++) {
        float r = row[i];
        if (isfinite(r) && r > 0.0f) {
            if (!found || r < best) { best = r; found = 1; }
        }
    }
    return found ? best : clear;
}

float ape_stopping_limited_speed(float v_des, float dmin, float max_decel_mps2, float stop_margin_m) {
    if (!isfinite(dmin) || dmin <= stop_margin_m)
        return 0.0f;
    float inner = 2.0f * max_decel_mps2 * (dmin - stop_margin_m);
    if (inner < 0.0f) inner = 0.0f;
    float vmax = sqrtf(inner);
    return (v_des < vmax) ? v_des : vmax;
}

int32_t ape_grid_index(const ape_search_state_t *s, float x, float y) {
    if (s == NULL || !s->initialized || s->grid_w <= 0 || s->grid_h <= 0 || s->cell_size_m <= 0.0f)
        return -1;
    int32_t cx = (int32_t)floorf((x - s->origin_x) / s->cell_size_m);
    int32_t cy = (int32_t)floorf((y - s->origin_y) / s->cell_size_m);
    if (cx < 0 || cx >= s->grid_w || cy < 0 || cy >= s->grid_h)
        return -1;
    return cy * s->grid_w + cx;
}

#define APE_TTC_INF 1.0e6f

float ape_threat_time_to_collision(float range_m, float closing_speed_mps) {
    if (closing_speed_mps > 1e-3f && range_m > 0.0f) {
        return range_m / closing_speed_mps;
    }
    return APE_TTC_INF;
}

float ape_threat_angular_half_width(float range_m, float radius_m, float vehicle_radius_m) {
    float r = (range_m > 0.5f) ? range_m : 0.5f;
    return atan2f(radius_m + vehicle_radius_m, r);
}
