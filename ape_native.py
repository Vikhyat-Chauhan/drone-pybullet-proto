"""
ape_native.py — ctypes bridge to the native APE planners
(native/ape_ops/): APE1 = reactive potential-field/Bug-style nudge,
APE2 = Dynamic Window Approach (medium compute), APE3 = Vector Field
Histogram (most compute, most sophisticated) -- dispatch matches the
ape2_dwa.c/ape3_vfh.c source filenames, and the gem5 cycle table
(gem5_measured_latencies.py) is keyed the same way.
These are the REAL decision-making algorithms, not a synthetic cost
proxy — nav_algorithm.py's role in the APE path is to marshal sensor
data + config into a call here and unpack the result; no algorithm logic
lives in Python (see docs/POWER_MODEL.md for why).

Build the library once (rebuild after moving to different hardware — the
.so is architecture-specific, not portable):
    cd native/ape_ops && make native

GIL note: ctypes releases the GIL for the duration of every foreign call
made through a CDLL-loaded function — this is standard ctypes behavior,
not something requiring an explicit flag. That's what makes concurrent
Python threads calling plan_apeN() genuinely execute in parallel across
cores, instead of serializing behind the GIL.

ABI note: ApeParams/ApeResult below must mirror ape_types.h's
ape_params_t/ape_result_t field-for-field. This is verified at import
time against the C side's own sizeof() via ape_native_sizeof_params()/
ape_native_sizeof_result() — a mismatch raises immediately rather than
silently misreading fields across the boundary.
"""

from __future__ import annotations

import ctypes
import pathlib

_LIB_PATH = (
    pathlib.Path(__file__).resolve().parent / "native" / "ape_ops" / "build" / "libape_ops.so"
)


APE_MAX_THREATS = 3


class ApeThreat(ctypes.Structure):
    _fields_ = [
        ("active", ctypes.c_int32),
        ("range_m", ctypes.c_float),
        ("bearing_rad", ctypes.c_float),
        ("closing_speed_mps", ctypes.c_float),
        ("radius_m", ctypes.c_float),
    ]


class ApeParams(ctypes.Structure):
    _fields_ = [
        ("ranges", ctypes.POINTER(ctypes.c_float)),
        ("n_ranges", ctypes.c_int32),
        ("n_layers", ctypes.c_int32),
        ("angle_min", ctypes.c_float),
        ("angle_increment", ctypes.c_float),
        ("vertical_angle_min", ctypes.c_float),
        ("vertical_angle_increment", ctypes.c_float),
        ("range_min", ctypes.c_float),
        ("range_max", ctypes.c_float),

        ("v_cmd", ctypes.c_float),
        ("yaw_err", ctypes.c_float),
        ("target_detected", ctypes.c_int32),

        ("drone_x", ctypes.c_float),
        ("drone_y", ctypes.c_float),
        ("drone_yaw", ctypes.c_float),

        ("max_v", ctypes.c_float),
        ("max_wz", ctypes.c_float),
        ("max_vz", ctypes.c_float),
        ("kp_yaw", ctypes.c_float),
        ("vehicle_radius_m", ctypes.c_float),
        ("max_decel_mps2", ctypes.c_float),
        ("stop_margin_m", ctypes.c_float),
        ("safe_m", ctypes.c_float),
        ("front_deg", ctypes.c_float),
        ("side_deg", ctypes.c_float),
        ("v_cap_frac", ctypes.c_float),
        ("sidestep_deg", ctypes.c_float),
        ("sidestep_speed_frac", ctypes.c_float),
        ("sudden_obj_radius_m", ctypes.c_float),
        ("sudden_obj_clearance_m", ctypes.c_float),
        ("curvature_k", ctypes.c_float),

        ("dwa_n_v", ctypes.c_int32),
        ("dwa_n_w", ctypes.c_int32),
        ("dwa_dt", ctypes.c_float),
        ("dwa_horizon_s", ctypes.c_float),
        ("dwa_w_clear", ctypes.c_float),
        ("dwa_w_heading", ctypes.c_float),
        ("dwa_w_speed", ctypes.c_float),

        ("vfh_n_sectors", ctypes.c_int32),
        ("vfh_threshold", ctypes.c_float),
        ("vfh_smax_sectors", ctypes.c_float),

        ("threats", ApeThreat * APE_MAX_THREATS),
        ("n_threats", ctypes.c_int32),
        ("dwa_w_threat", ctypes.c_float),
        ("vfh_w_threat", ctypes.c_float),
        ("vfh_threat_horizon_s", ctypes.c_float),
    ]


class ApeResult(ctypes.Structure):
    _fields_ = [
        ("v", ctypes.c_float),
        ("wz", ctypes.c_float),
        ("vz", ctypes.c_float),
        ("score", ctypes.c_float),
        ("ok", ctypes.c_int32),
    ]


APE_GRID_MAX_CELLS = 4096


class ApeSearchState(ctypes.Structure):
    """Mirrors ape_types.h's ape_search_state_t. One instance per APE2/
    APE3 per mission, owned by nav_algorithm.py (allocated/reset in
    begin_mission()), passed by pointer into every plan_ape2/plan_ape3
    call for that mission's lifetime. APE1 never gets one -- no memory,
    by design."""
    _fields_ = [
        ("initialized", ctypes.c_int32),
        ("grid_w", ctypes.c_int32),
        ("grid_h", ctypes.c_int32),
        ("cell_size_m", ctypes.c_float),
        ("origin_x", ctypes.c_float),
        ("origin_y", ctypes.c_float),
        ("cells", ctypes.c_uint8 * APE_GRID_MAX_CELLS),
    ]


_lib: ctypes.CDLL | None = None


def _load() -> ctypes.CDLL:
    global _lib
    if _lib is not None:
        return _lib
    if not _LIB_PATH.exists():
        raise RuntimeError(
            f"ape_native: {_LIB_PATH} not found.\n"
            "Build it first with:\n"
            "    cd native/ape_ops && make native\n"
        )
    lib = ctypes.CDLL(str(_LIB_PATH))

    lib.ape_native_plan_ape1.argtypes = [ctypes.POINTER(ApeParams), ctypes.POINTER(ApeResult)]
    lib.ape_native_plan_ape1.restype = None
    for name in ("ape_native_plan_ape2", "ape_native_plan_ape3"):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.POINTER(ApeParams), ctypes.POINTER(ApeSearchState), ctypes.POINTER(ApeResult)]
        fn.restype = None

    lib.ape_native_search_state_reset.argtypes = [
        ctypes.POINTER(ApeSearchState), ctypes.c_int32, ctypes.c_int32,
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ]
    lib.ape_native_search_state_reset.restype = None

    lib.ape_native_sizeof_params.argtypes = []
    lib.ape_native_sizeof_params.restype = ctypes.c_int32
    lib.ape_native_sizeof_result.argtypes = []
    lib.ape_native_sizeof_result.restype = ctypes.c_int32
    lib.ape_native_sizeof_search_state.argtypes = []
    lib.ape_native_sizeof_search_state.restype = ctypes.c_int32

    c_params_size = lib.ape_native_sizeof_params()
    c_result_size = lib.ape_native_sizeof_result()
    c_search_state_size = lib.ape_native_sizeof_search_state()
    py_params_size = ctypes.sizeof(ApeParams)
    py_result_size = ctypes.sizeof(ApeResult)
    py_search_state_size = ctypes.sizeof(ApeSearchState)
    if (c_params_size != py_params_size or c_result_size != py_result_size
            or c_search_state_size != py_search_state_size):
        raise RuntimeError(
            "ape_native: ABI mismatch between ape_types.h and ApeParams/ApeResult/ApeSearchState.\n"
            f"  ape_params_t: C sizeof={c_params_size}, Python ctypes sizeof={py_params_size}\n"
            f"  ape_result_t: C sizeof={c_result_size}, Python ctypes sizeof={py_result_size}\n"
            f"  ape_search_state_t: C sizeof={c_search_state_size}, Python ctypes sizeof={py_search_state_size}\n"
            "Field order/types have drifted out of sync — fix ApeParams/ApeResult/ApeSearchState in "
            "ape_native.py to match ape_types.h exactly before trusting this binding."
        )

    _lib = lib
    return lib


def plan_ape1(params: ApeParams) -> ApeResult:
    lib = _load()
    result = ApeResult()
    lib.ape_native_plan_ape1(ctypes.byref(params), ctypes.byref(result))
    return result


def _plan_with_state(fn_name: str, params: ApeParams, state: "ApeSearchState | None") -> ApeResult:
    lib = _load()
    result = ApeResult()
    state_ptr = ctypes.byref(state) if state is not None else None
    getattr(lib, fn_name)(ctypes.byref(params), state_ptr, ctypes.byref(result))
    return result


def plan_ape2(params: ApeParams, state: "ApeSearchState | None" = None) -> ApeResult:
    return _plan_with_state("ape_native_plan_ape2", params, state)


def plan_ape3(params: ApeParams, state: "ApeSearchState | None" = None) -> ApeResult:
    return _plan_with_state("ape_native_plan_ape3", params, state)


def reset_search_state(state: ApeSearchState, grid_w: int, grid_h: int,
                        cell_size_m: float, origin_x: float, origin_y: float) -> None:
    lib = _load()
    lib.ape_native_search_state_reset(ctypes.byref(state), int(grid_w), int(grid_h),
                                       float(cell_size_m), float(origin_x), float(origin_y))
