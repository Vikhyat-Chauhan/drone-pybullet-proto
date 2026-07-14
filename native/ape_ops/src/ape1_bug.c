/*
 * ape1_bug.c — real, cheap reactive planner: a proportional heading
 * controller toward the goal bearing (yaw_err), blended with a simple
 * potential-field repulsion term steering away from whichever side
 * (left/right wide window) has the nearer obstacle. This is deliberately
 * the fast/crude tier — no forward simulation, no candidate sampling,
 * just three sector-min lookups and closed-form arithmetic — matching
 * the paper's "fast but least accurate" role for APE1.
 *
 * Tuning constants below (K_REPULSION, side-window geometry, SLOW_FRAC)
 * have no external citation — they're modeler's choices, empirically
 * tunable against the sim, not datasheet facts.
 */
#include "ape1_bug.h"
#include "ape_common.h"
#include <math.h>

#define SLOW_FRAC       0.05f   /* APE1 always moves cautiously slow */
#define SIDE_WINDOW_CENTER_DEG 45.0f
#define SIDE_WINDOW_HALF_DEG   15.0f
#define K_REPULSION     2.0f    /* rad/s per (1/m) side-clearance imbalance */
#define EPS_M           0.1f

ape_result_t ape1_bug_plan(const ape_params_t *p) {
    ape_result_t r = {0};

    float front_half = (p->front_deg > 5.0f) ? p->front_deg : 5.0f;
    float d_front = ape_sector_min(p, 0, 0.0f, front_half);
    float d_left  = ape_sector_min(p, 0, +SIDE_WINDOW_CENTER_DEG, SIDE_WINDOW_HALF_DEG);
    float d_right = ape_sector_min(p, 0, -SIDE_WINDOW_CENTER_DEG, SIDE_WINDOW_HALF_DEG);

    float inv_left  = 1.0f / ((d_left  > EPS_M) ? d_left  : EPS_M);
    float inv_right = 1.0f / ((d_right > EPS_M) ? d_right : EPS_M);
    float repulsion = inv_left - inv_right;  /* >0: obstacle closer on left -> steer right (negative wz) */

    float wz_goal = ape_clampf(p->kp_yaw * ape_wrap_pi(p->yaw_err), -p->max_wz, p->max_wz);
    float wz = ape_clampf(wz_goal - K_REPULSION * repulsion, -p->max_wz, p->max_wz);

    float curv_k = (p->curvature_k > 0.05f) ? p->curvature_k : 0.05f;
    float curv_cap = p->max_v / (1.0f + curv_k * fabsf(wz));
    float v_cap_local = 0.3f * p->v_cap_frac * p->max_v;
    float base_v = ape_clampf(p->v_cmd, 0.0f, SLOW_FRAC * p->max_v);

    float v = base_v;
    if (v_cap_local < v) v = v_cap_local;
    if (curv_cap < v) v = curv_cap;
    v = ape_stopping_limited_speed(v, d_front, p->max_decel_mps2, p->stop_margin_m);

    r.v = v;
    r.wz = wz;
    r.vz = 0.0f;
    r.score = -1e6f;   /* always lowest priority among the three tiers */
    r.ok = 1;
    return r;
}
