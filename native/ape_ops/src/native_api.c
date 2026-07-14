#include "native_api.h"
#include "ape1_bug.h"
#include "ape2_dwa.h"
#include "ape3_vfh.h"

void ape_native_plan_ape1(const ape_params_t *params, ape_result_t *out) {
    *out = ape1_bug_plan(params);
}

/* APE2 <-> APE3 identities are deliberately swapped from the source
 * filenames: APE2 = VFH (ape3_vfh_plan), APE3 = DWA (ape2_dwa_plan), so
 * that the heavier/slower planner (DWA) sits under the APE3 label. */
void ape_native_plan_ape2(const ape_params_t *params, ape_result_t *out) {
    *out = ape3_vfh_plan(params);
}

void ape_native_plan_ape3(const ape_params_t *params, ape_result_t *out) {
    *out = ape2_dwa_plan(params);
}

int32_t ape_native_sizeof_params(void) { return (int32_t)sizeof(ape_params_t); }
int32_t ape_native_sizeof_result(void) { return (int32_t)sizeof(ape_result_t); }
