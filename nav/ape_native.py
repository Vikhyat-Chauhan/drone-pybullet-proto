"""
ape_native.py — ctypes bridge to the native APE planners (native/ape_ops/):
APE1 = reactive potential-field/Bug-style nudge (ape1_bug.c), APE2 =
Vector Field Histogram (vfh.c, cheap tier: single-layer scan), APE3 =
Dynamic Window Approach (dwa.c, heavy tier: multi-layer scan) -- the
APE2/APE3 <-> algorithm binding lives in native_api.c's dispatch, not
here or in gem5_measured_latencies.py (whose "ape2"/"ape3" keys track
the tier's cycle budget, independent of which algorithm currently runs
there). These are the real decision algorithms, not a cost proxy -- no
planning logic lives in Python (see docs/POWER_MODEL.md).

Build once, rebuild after switching target hardware (.so is
architecture-specific): cd native/ape_ops && make native

GIL note: ctypes releases the GIL for the duration of any CDLL call, so
concurrent plan_apeN() calls from Python threads genuinely run in
parallel across cores.

ABI note: ApeParams/ApeResult must mirror ape_types.h's
ape_params_t/ape_result_t field-for-field; checked at import time via
ape_native_sizeof_params()/_result() so a drift raises immediately
instead of silently misreading fields.
"""

from __future__ import annotations

import ctypes
import pathlib

_LIB_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / "native" / "ape_ops" / "build" / "libape_ops.so"
)


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
    ]


class ApeResult(ctypes.Structure):
    _fields_ = [
        ("v", ctypes.c_float),
        ("wz", ctypes.c_float),
        ("vz", ctypes.c_float),
        ("score", ctypes.c_float),
        ("ok", ctypes.c_int32),
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

    for name in ("ape_native_plan_ape1", "ape_native_plan_ape2", "ape_native_plan_ape3"):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.POINTER(ApeParams), ctypes.POINTER(ApeResult)]
        fn.restype = None

    lib.ape_native_sizeof_params.argtypes = []
    lib.ape_native_sizeof_params.restype = ctypes.c_int32
    lib.ape_native_sizeof_result.argtypes = []
    lib.ape_native_sizeof_result.restype = ctypes.c_int32

    c_params_size = lib.ape_native_sizeof_params()
    c_result_size = lib.ape_native_sizeof_result()
    py_params_size = ctypes.sizeof(ApeParams)
    py_result_size = ctypes.sizeof(ApeResult)
    if c_params_size != py_params_size or c_result_size != py_result_size:
        raise RuntimeError(
            "ape_native: ABI mismatch between ape_types.h and ApeParams/ApeResult.\n"
            f"  ape_params_t: C sizeof={c_params_size}, Python ctypes sizeof={py_params_size}\n"
            f"  ape_result_t: C sizeof={c_result_size}, Python ctypes sizeof={py_result_size}\n"
            "Field order/types have drifted out of sync — fix ApeParams/ApeResult in "
            "ape_native.py to match ape_types.h exactly before trusting this binding."
        )

    _lib = lib
    return lib


def _plan(fn_name: str, params: ApeParams) -> ApeResult:
    lib = _load()
    result = ApeResult()
    getattr(lib, fn_name)(ctypes.byref(params), ctypes.byref(result))
    return result


def plan_ape1(params: ApeParams) -> ApeResult:
    return _plan("ape_native_plan_ape1", params)


def plan_ape2(params: ApeParams) -> ApeResult:
    return _plan("ape_native_plan_ape2", params)


def plan_ape3(params: ApeParams) -> ApeResult:
    return _plan("ape_native_plan_ape3", params)
