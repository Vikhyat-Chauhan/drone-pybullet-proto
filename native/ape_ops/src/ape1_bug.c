/*
 * ape1_bug.c — real, cheap reactive planner: a proportional heading
 * controller toward the goal bearing (yaw_err), blended with a simple
 * potential-field repulsion term steering away from whichever side
 * (left/right wide window) has the nearer obstacle. This is deliberately
 * the fast/crude tier — no forward simulation, no candidate sampling,
 * just three sector-min lookups and closed-form arithmetic — matching
 * the paper's "fast but least accurate" role for APE1. What makes this
 * tier "cheap" is decision QUALITY (no prediction, no candidate search),
 * not an artificial speed cap -- it shares the same v_cap_frac*max_v
 * ceiling (plus the same curvature/stopping-distance safety caps) the
 * other two tiers use, so a slow reach-time or a failure to physically
 * outrun a threat reflects the crudeness of its steering decisions, not
 * a hobbled top speed.
 *
 * Tuning constants below (K_REPULSION, side-window geometry) have no
 * external citation — they're modeler's choices, empirically tunable
 * against the sim, not datasheet facts.
 *
 * NOTE: the threat-collision braking term added below (threat_brake_range)
 * adds real instructions (a sinf() call and comparisons per active
 * threat) to the measured ROI loop -- gem5_measured_latencies.py's
 * APE1 cycle count was frozen BEFORE this change and needs
 * regenerating (native/ape_ops/gem5_bench/scripts/
 * freeze_measured_latencies.py) on a machine with a working gem5 build
 * before APE1's budget_ms in the live sim can be trusted again. No
 * gem5 toolchain was available in the environment that made this
 * change, so it could not be regenerated here.
 */
#include "ape1_bug.h"
#include "ape_common.h"
#include <math.h>

#define SIDE_WINDOW_CENTER_DEG 45.0f
#define SIDE_WINDOW_HALF_DEG   15.0f
#define K_REPULSION     2.0f    /* rad/s per (1/m) side-clearance imbalance */
#define K_THREAT        6.0f    /* rad/s per (1/m) threat-range, per active threat */
#define K_ESCALATE      6.0f    /* rad/s per meter of collision-buffer intrusion */
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

    /* Search mode (target undetected): no memory, no state -- just steer
     * away from whichever side is more obstructed, i.e. wander toward
     * open space using the same repulsion term avoidance already
     * computes. This is deliberately the cheapest possible tier: it adds
     * nothing beyond the branch below, keeping APE1's compute (and its
     * deadline margin) effectively unchanged from pure avoidance. */
    float wz_goal = p->target_detected
        ? ape_clampf(p->kp_yaw * ape_wrap_pi(p->yaw_err), -p->max_wz, p->max_wz)
        : 0.0f;

    /* Cheap reflexive threat dodge: steer away from whichever side each
     * active threat's bearing is on, weighted only by inverse range --
     * closing_speed_mps is deliberately ignored for the TURN term (no
     * prediction). This tier still doesn't forward-simulate or sample
     * candidates like APE2/APE3 -- it's the tier's core weakness that
     * it reacts to ANY nearby threat regardless of whether it's
     * actually closing, which is what makes its avoidance oversized/
     * conservative (worse path efficiency) even though it's cheap
     * enough to always finish well inside its budget.
     *
     * threat_brake_range: unlike the turn term, this DOES need a
     * minimal notion of "is this threat actually on a collision line,"
     * or APE1 can win every deadline race and still fly straight
     * through a threat it turned away from too little/late (measured:
     * before this term existed, APE1 crashed almost exclusively on
     * races it had already WON, unlike APE2/APE3, which only crashed
     * from missing the deadline outright). Closed-form, not predictive:
     * the threat's perpendicular offset from the body-forward axis
     * (range_m * sin(bearing_rad)) approximates where it will pass
     * relative to the current heading if nothing changes; if that's
     * inside the combined collision radius and the threat is actually
     * closing, its range is fed into the same stopping-distance cap
     * already used for lidar obstacles (d_front) below -- no separate
     * new mechanism, no forward simulation.
     *
     * threat_escalation: braking alone doesn't help against a threat
     * closing under its own velocity (ThreatSensorProvider's
     * closing_speed_mps is the threat's own motion projected onto the
     * line of sight -- the drone's speed doesn't factor in, so slowing
     * down doesn't take the drone off a collision line the way it does
     * for a static lidar obstacle; measured, braking alone left APE1's
     * crash rate on races it WON basically unchanged). What actually
     * needs to change is lateral position, i.e. turn harder -- but only
     * for genuinely intruding threats: an earlier attempt at a blanket
     * K_THREAT increase (steer harder away from EVERY nearby threat,
     * closing or not) made things worse by over-correcting into a
     * second concurrent threat. Gating the extra turn on the same
     * collision-buffer-intrusion test as the brake keeps the escalation
     * targeted at threats actually on a collision line.
     *
     * KNOWN LIMIT: this closes SOME but not all of the "won the race,
     * crashed anyway" gap. In the tightest encounters wz was already
     * saturating at max_wz from the base repulsion terms alone before
     * threat_escalation is even added (confirmed: sweeping K_ESCALATE
     * across 4/10/20 produced byte-identical mission outcomes -- proof
     * the clamp, not the gain, was binding). APE2/APE3 achieve zero
     * resolved-then-crashed outcomes under the SAME max_wz by jointly
     * searching (v, wz) candidate pairs over a forward-simulated
     * horizon instead of computing wz from a single closed-form repulsion
     * formula independent of v; matching that would mean giving APE1
     * the same candidate-search structure APE2/APE3 already have,
     * i.e. no longer being the cheap/closed-form tier. Left as a
     * documented, real limitation rather than papered over. */
    float threat_repulsion = 0.0f;
    float threat_escalation = 0.0f;
    float threat_brake_range = INFINITY;
    for (int32_t i = 0; i < APE_MAX_THREATS; i++) {
        const ape_threat_t *th = &p->threats[i];
        if (!th->active) continue;
        float rng = (th->range_m > EPS_M) ? th->range_m : EPS_M;
        float side = (th->bearing_rad >= 0.0f) ? 1.0f : -1.0f;  /* +1 left, -1 right */
        threat_repulsion += side * (1.0f / rng);

        float lateral_offset = rng * sinf(th->bearing_rad);
        float collision_buffer = th->radius_m + p->vehicle_radius_m + p->sudden_obj_clearance_m;
        float intrusion = collision_buffer - fabsf(lateral_offset);
        if (th->closing_speed_mps > 0.1f && intrusion > 0.0f) {
            if (rng < threat_brake_range) threat_brake_range = rng;
            threat_escalation += side * intrusion;
        }
    }

    float wz = ape_clampf(wz_goal - K_REPULSION * repulsion - K_THREAT * threat_repulsion
                           - K_ESCALATE * threat_escalation,
                           -p->max_wz, p->max_wz);

    float curv_k = (p->curvature_k > 0.05f) ? p->curvature_k : 0.05f;
    float curv_cap = p->max_v / (1.0f + curv_k * fabsf(wz));
    float v_cap_local = p->v_cap_frac * p->max_v;
    float base_v = ape_clampf(p->v_cmd, 0.0f, p->max_v);

    float v = base_v;
    if (v_cap_local < v) v = v_cap_local;
    if (curv_cap < v) v = curv_cap;
    v = ape_stopping_limited_speed(v, d_front, p->max_decel_mps2, p->stop_margin_m);
    if (threat_brake_range < INFINITY) {
        v = ape_stopping_limited_speed(v, threat_brake_range, p->max_decel_mps2, p->stop_margin_m);
    }

    r.v = v;
    r.wz = wz;
    r.vz = 0.0f;
    r.score = -1e6f;   /* always lowest priority among the three tiers */
    r.ok = 1;
    return r;
}
