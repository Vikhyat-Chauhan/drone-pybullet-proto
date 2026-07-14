#include "native_api.h"
#include "ape1_bug.h"
#include "ape2_dwa.h"
#include "ape3_vfh.h"

void ape_native_plan_ape1(const ape_params_t *params, ape_result_t *out) {
    *out = ape1_bug_plan(params);
}

/* APE2 = DWA (ape2_dwa.c, medium compute), APE3 = VFH (ape3_vfh.c, most
 * compute) -- dispatch matches the source-file identity, which is also
 * how the gem5 cycle table (gem5_measured_latencies.py, keyed by
 * "ape2"/"ape3") is measured. */
void ape_native_plan_ape2(const ape_params_t *params, ape_result_t *out) {
    *out = ape2_dwa_plan(params);
}

void ape_native_plan_ape3(const ape_params_t *params, ape_result_t *out) {
    *out = ape3_vfh_plan(params);
}

int32_t ape_native_sizeof_params(void) { return (int32_t)sizeof(ape_params_t); }
int32_t ape_native_sizeof_result(void) { return (int32_t)sizeof(ape_result_t); }
