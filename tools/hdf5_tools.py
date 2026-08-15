"""
hdf5_tools.py — Core module for reading trajectory HDF5 files and converting
them to the 12-servo format used by the Mini Pupper ESP32 firmware.

HDF5 format (from the trajectory optimizer in minipupper-backflip).
Two variants are auto-detected by q.shape[1]:

  14-DOF (Mini Pupper reduced — hips locked out):
    q: (n, 14)  [base_x, base_y, base_z, MRP_x, MRP_y, MRP_z,
                 FR_HFE, FR_KFE, HR_HFE, HR_KFE,
                 FL_HFE, FL_KFE, HL_HFE, HL_KFE]
                 ^ SIDE-MAJOR (right side, then left side) — NOT FR,FL,HR,HL
    v: (n, 14)  tau: (n, 8)

  18-DOF (Solo12 full model):
    q: (n, 18)  [base_xyz(3), MRP(3),
                 FR_HAA, FR_HFE, FR_KFE, FL_HAA, FL_HFE, FL_KFE,
                 HR_HAA, HR_HFE, HR_KFE, HL_HAA, HL_HFE, HL_KFE]
    v: (n, 18)  tau: (n, 12)

All joint angles are in RADIANS.

ESP32 12-servo order (matching mp2_calib.h / mp2_backflip_data.h):
    Servo  1: FR_abd  (abduction, always 0 — mechanically locked)
    Servo  2: FR_HFE  (hip flexion/extension)
    Servo  3: FR_KFE  (knee flexion/extension)
    Servo  4: FL_abd
    Servo  5: FL_HFE
    Servo  6: FL_KFE
    Servo  7: RR_abd
    Servo  8: RR_HFE
    Servo  9: RR_KFE
    Servo 10: RL_abd
    Servo 11: RL_HFE
    Servo 12: RL_KFE

Conversion to SCS units:
    SCS = round(511 + (SIGN * (URDF_DEG - STAND) + offset) / 0.263)
    where 0.263 = 270 deg / 1024 ticks
"""

import h5py
import numpy as np
import json
import os
from typing import Tuple, Optional, Dict, Any

# ---------------------------------------------------------------------------
# HDF5 q-index → 12-column mapping
#
# Two HDF5 formats are auto-detected:
#
#   14-DOF (Mini Pupper reduced, hips locked):
#     q indices 6..13 = FR_HFE, FR_KFE, FL_HFE, FL_KFE,
#                        HR_HFE, HR_KFE, HL_HFE, HL_KFE
#
#   18-DOF (Solo12 full model):
#     q indices 6..17 = FR_HAA, FR_HFE, FR_KFE, FL_HAA, FL_HFE, FL_KFE,
#                        HR_HAA, HR_HFE, HR_KFE, HL_HAA, HL_HFE, HL_KFE
#     We skip HAA and pick HFE/KFE at indices 7,8, 10,11, 13,14, 16,17.
#
#   ESP32 12-col order:
#     FR_abd, FR_HFE, FR_KFE,  FL_abd, FL_HFE, FL_KFE,
#     RR_abd, RR_HFE, RR_KFE,  RL_abd, RL_HFE, RL_KFE
# ---------------------------------------------------------------------------

# Mapping tables keyed by q.shape[1]:
#   ESP32 column → (HDF5 q index or None for abduction)
# NOTE (fixed): the 14-DOF Mini Pupper files are SIDE-MAJOR, not front-major.
# The actual q[6:14] order is:
#     FR_HFE, FR_KFE,  RR_HFE, RR_KFE,  FL_HFE, FL_KFE,  RL_HFE, RL_KFE
# i.e. (front, rear) of one side, then (front, rear) of the other side.
# Verified empirically: in a sagittally-symmetric backflip, q[6:8] == q[10:12]
# and q[8:10] == q[12:14] — which only holds if slots 0/2 are the same END of
# the robot mirrored L/R, not the same SIDE.  The old map sent the REAR-leg
# trajectory to FL and the FRONT-leg trajectory to RR.
_Q_INDEX_MAP_BY_DIM = {
    14: [None, 6, 7,   None, 10, 11,   None, 8, 9,   None, 12, 13],
    18: [None, 7, 8,   None, 10, 11,  None, 13, 14,   None, 16, 17],
}

# Velocity index maps (same shape as q for the reduced model)
_V_INDEX_MAP_BY_DIM = {
    14: [None, 6, 7,   None, 10, 11,   None, 8, 9,   None, 12, 13],
    18: [None, 7, 8,   None, 10, 11,  None, 13, 14,   None, 16, 17],
}

# Torque index maps (tau has only actuated joint dims, same leg order as q)
_TAU_INDEX_MAP_BY_DIM = {
    8:  [None, 0, 1,   None, 4, 5,   None, 2, 3,   None, 6, 7],
    12: [None, 1, 2,   None, 4, 5,   None, 7, 8,   None, 10, 11],
}

# Default calibration matching mp2_calib.h
BF_STAND = np.array([0, -90, 45, 0, -90, 45, 0, -90, 45, 0, -90, 45],
                     dtype=np.float64)
BF_SIGN  = np.array([+1, +1, +1, +1, -1, -1, +1, +1, +1, +1, -1, -1],
                     dtype=np.float64)

# ---------------------------------------------------------------------------
# KNEE CONVENTION — parallel-linkage coupling
#
# The optimizer's URDF is a plain SERIAL two-link chain, so KFE is the knee
# angle measured RELATIVE TO THE THIGH (i.e. -(pi - beta), the bend).
#
# The Mini Pupper's lower-leg servo is body-mounted and drives the calf through
# a parallel linkage, so it commands the calf's ABSOLUTE orientation, in the
# same angular sense and from the same datum as the hip servo. This is exactly
# what the firmware's IK produces:
#     stanford_kinematics.c:  knee_angle = hip_angle - (M_PI - beta)
#
# Therefore:  servo_knee_deg  =  HFE + KFE     (absolute calf angle)
#
# Verified against frame 0 of the trajectories:
#     HFE = +56.95, KFE = -101.24  ->  implied leg length 70.2 mm
#         (== SK_NEUTRAL_HEIGHT_MM / NEUTRAL_Z = 70)
#     HFE + KFE = -44.29  ->  |.| = 44.3 == BF_STAND knee datum of 45
#   The relative bend (101.2 deg) matches nothing in the calibration; the
#   absolute calf angle matches it to 0.7 deg. The firmware is absolute.
#
# Feeding the raw relative KFE straight through leaves the hip correct but the
# calf wrong on ALL FOUR legs, because it drops the +HFE coupling term.
# ---------------------------------------------------------------------------
KNEE_ABSOLUTE = True          # module default; override per call if needed
_KNEE_COLS = (2, 5, 8, 11)    # KFE columns in the 12-col ESP32 order
_HIP_COLS  = (1, 4, 7, 10)    # matching HFE columns


def couple_knee_to_hip(angles_deg: np.ndarray) -> np.ndarray:
    """Convert serial-chain URDF knee angles to ABSOLUTE calf angles.

    knee_abs = HFE + KFE, applied to all four legs. Accepts (n, 12) or (12,).
    """
    a = np.atleast_2d(np.array(angles_deg, dtype=np.float64, copy=True))
    for hip, knee in zip(_HIP_COLS, _KNEE_COLS):
        a[:, knee] = a[:, hip] + a[:, knee]
    return a.reshape(np.shape(angles_deg))

# ---------------------------------------------------------------------------
# Leg / joint name helpers
# ---------------------------------------------------------------------------
LEG_NAMES = ["FR", "FL", "RR", "RL"]
JOINT_NAMES = ["ABD", "HFE", "KFE"]

SERVO_NAMES = [
    f"{leg}_{jnt}" for leg in LEG_NAMES for jnt in JOINT_NAMES
]


def load_hdf5(filepath: str) -> Dict[str, np.ndarray]:
    """Load all datasets from a trajectory HDF5 file.

    Returns a dict with keys: q, v, a, tau, knot_durations, n_knots.
    """
    data = {}
    with h5py.File(filepath, "r") as f:
        for key in ("q", "v", "a", "τ", "knot_durations", "n_knots"):
            try:
                data[key] = np.array(f[key])
            except KeyError:
                pass  # not all files have all fields

        # Also try the unicode tau
        if "tau" in f and "τ" not in data:
            data["τ"] = np.array(f["tau"])

    # Rename τ key to 'tau' for convenience
    if "τ" in data:
        data["tau"] = data.pop("τ")

    return data


def _get_q_dim(data: Dict[str, np.ndarray]) -> int:
    """Return the q dimension (number of DOF in the state vector)."""
    return data["q"].shape[1]


def _get_tau_dim(data: Dict[str, np.ndarray]) -> int:
    """Return the tau dimension (number of actuated joints)."""
    if "tau" in data:
        return data["tau"].shape[1]
    return 8  # default for Mini Pupper


def extract_joint_angles_deg(data: Dict[str, np.ndarray],
                             couple_knee: Optional[bool] = None) -> np.ndarray:
    """Extract joint angles from HDF5 data, returning (n_knots, 12) in
    URDF degrees with abduction columns set to 0.0.

    If ``couple_knee`` (default: KNEE_ABSOLUTE) the KFE columns are converted
    from the URDF's thigh-relative angle to the ABSOLUTE calf angle the
    parallel-linkage knee servo actually commands (see couple_knee_to_hip).

    Auto-detects the HDF5 format from q.shape[1]:
      14 = Mini Pupper reduced (hips locked)
      18 = Solo12 full model

    Parameters
    ----------
    data : dict
        Output of load_hdf5(), must contain 'q'.

    Returns
    -------
    angles_deg : ndarray of shape (n_knots, 12)
        Joint angles in degrees, 12-column ESP32 order.
    """
    q = data["q"]
    q_dim = q.shape[1]
    n_knots = q.shape[0]

    idx_map = _Q_INDEX_MAP_BY_DIM.get(q_dim)
    if idx_map is None:
        raise ValueError(
            f"Unknown q dimension: {q_dim}. "
            f"Expected 14 (Mini Pupper reduced) or 18 (Solo12)."
        )

    angles_deg = np.zeros((n_knots, 12), dtype=np.float64)
    for col, q_idx in enumerate(idx_map):
        if q_idx is not None:
            angles_deg[:, col] = np.rad2deg(q[:, q_idx])
        # else: stays 0.0 (abduction)

    if KNEE_ABSOLUTE if couple_knee is None else couple_knee:
        angles_deg = couple_knee_to_hip(angles_deg)

    return angles_deg


def extract_joint_velocities_deg_s(data: Dict[str, np.ndarray]) -> np.ndarray:
    """Extract joint velocities (rad/s → deg/s), (n_knots, 12) with 0 abd."""
    v = data.get("v")
    if v is None:
        raise KeyError("HDF5 file has no 'v' dataset")

    v_dim = v.shape[1]
    n_knots = v.shape[0]

    idx_map = _V_INDEX_MAP_BY_DIM.get(v_dim)
    if idx_map is None:
        raise ValueError(
            f"Unknown v dimension: {v_dim}. "
            f"Expected 14 (Mini Pupper reduced) or 18 (Solo12)."
        )

    vel_deg_s = np.zeros((n_knots, 12), dtype=np.float64)
    for col, v_idx in enumerate(idx_map):
        if v_idx is not None:
            vel_deg_s[:, col] = np.rad2deg(v[:, v_idx])

    return vel_deg_s


def extract_torques(data: Dict[str, np.ndarray]) -> np.ndarray:
    """Extract joint torques (N*m), (n_knots, 12) with 0 for abduction."""
    tau = data.get("tau")
    if tau is None:
        raise KeyError("HDF5 file has no 'tau'/'τ' dataset")

    tau_dim = tau.shape[1]
    n_knots = tau.shape[0]

    idx_map = _TAU_INDEX_MAP_BY_DIM.get(tau_dim)
    if idx_map is None:
        raise ValueError(
            f"Unknown tau dimension: {tau_dim}. "
            f"Expected 8 (Mini Pupper) or 12 (Solo12)."
        )

    torques = np.zeros((n_knots, 12), dtype=np.float64)
    for col, t_idx in enumerate(idx_map):
        if t_idx is not None:
            torques[:, col] = tau[:, t_idx]

    return torques


# ---------------------------------------------------------------------------
# Angle convention calibration
#
# The HDF5 uses the *optimizer's* URDF joint-angle convention (based on
# whatever Pinocchio's neutral() returns for the model). The ESP32 firmware
# uses a different "BF_STAND" convention where the standing pose maps to
# convenient round numbers (HFE=-90, KFE=+45).
#
# The key insight: the HDF5's **first frame is the standing/initial pose**.
# By auto-detecting the first frame's HFE/KFE, we can compute the offset
# between the two conventions:
#
#   bf_urdf_deg = opt_deg - standing_opt + BF_STAND
#   servo_deg    = 135 + BF_SIGN * (opt_deg - standing_opt)
#
# Note that BF_STAND *cancels* in the servo path — only BF_SIGN matters
# for left/right mirroring.
# ---------------------------------------------------------------------------

def get_standing_reference(data: Dict[str, np.ndarray]) -> np.ndarray:
    """Extract the first frame's joint angles as the standing reference.

    This is the optimizer's URDF joint angle at the neutral/standing pose.
    Returns a (12,) array with abduction columns set to 0.
    """
    angles = extract_joint_angles_deg(data)
    return angles[0, :].copy()


def opt_to_bf_stand_deg(
    angles_deg: np.ndarray,
    standing_ref: Optional[np.ndarray] = None,
    bf_stand: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Convert optimizer URDF degrees to BF_STAND URDF degrees.

    Formula: bf_deg = opt_deg - standing_opt + BF_STAND

    Parameters
    ----------
    angles_deg : (n_frames, 12) joint angles in optimizer URDF degrees
    standing_ref : (12,) standing-pose angles in optimizer convention.
                   If None, uses the first row of angles_deg.
    bf_stand : (12,) BF_STAND calibration. Default: BF_STAND.

    Returns
    -------
    bf_deg : (n_frames, 12) angles in BF_STAND convention
    """
    if standing_ref is None:
        standing_ref = angles_deg[0, :]
    if bf_stand is None:
        bf_stand = BF_STAND

    return angles_deg - standing_ref[np.newaxis, :] + bf_stand[np.newaxis, :]


def opt_to_servo_deg(
    angles_deg: np.ndarray,
    standing_ref: Optional[np.ndarray] = None,
    bf_sign: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Convert optimizer URDF degrees to absolute servo degrees (0-270).

    Formula: servo_deg = 135 + BF_SIGN * (opt_deg - standing_opt)

    This is the conversion needed for the ESP32 /leg endpoint, which
    expects absolute servo angles in the 0-270 range (135 = center).

    Parameters
    ----------
    angles_deg : (n_frames, 12) joint angles in optimizer URDF degrees
    standing_ref : (12,) standing-pose angles in optimizer convention.
                   If None, uses the first row of angles_deg.
    bf_sign : (12,) BF_SIGN calibration. Default: BF_SIGN.

    Returns
    -------
    servo_deg : (n_frames, 12) absolute servo degrees, clamped to [0, 270]
    """
    if standing_ref is None:
        standing_ref = angles_deg[0, :]
    if bf_sign is None:
        bf_sign = BF_SIGN

    result = 135.0 + bf_sign[np.newaxis, :] * (angles_deg - standing_ref[np.newaxis, :])
    return np.clip(result, 0.0, 270.0)


def get_timesteps(data: Dict[str, np.ndarray]) -> Tuple[np.ndarray, float]:
    """Return cumulative time array (seconds) and the nominal dt.

    Returns
    -------
    t : ndarray of shape (n_knots,)
        Cumulative time at each knot (starts at 0).
    dt : float
        Nominal (most common) time step.
    """
    dts = data.get("knot_durations")
    if dts is None:
        raise KeyError("HDF5 file has no 'knot_durations' dataset")

    t = np.cumsum(np.concatenate([[0.0], dts[:-1].flatten()]))
    # Use the most common dt as nominal
    dt = float(np.median(dts))
    return t, dt


def hdf5_summary(filepath: str) -> str:
    """Return a human-readable summary of an HDF5 trajectory file."""
    data = load_hdf5(filepath)
    q = data.get("q")
    n_knots = q.shape[0] if q is not None else 0
    t, dt = get_timesteps(data)

    lines = [
        f"File:        {os.path.basename(filepath)}",
        f"Knots:       {n_knots}",
        f"Duration:    {t[-1]:.3f} s",
        f"Nominal dt:  {dt*1000:.1f} ms  ({1/dt:.0f} Hz)",
        f"q shape:     {q.shape if q is not None else 'N/A'}",
    ]

    if "v" in data:
        lines.append(f"v shape:     {data['v'].shape}")
    if "tau" in data:
        tau = data["tau"]
        lines.append(f"tau shape:   {tau.shape}")
        lines.append(f"max |tau|:   {np.abs(tau).max():.3f} N*m")

    ang = extract_joint_angles_deg(data)
    for i, name in enumerate(SERVO_NAMES):
        lines.append(
            f"  {name:>8s}:  {ang[:, i].min():+7.1f} .. {ang[:, i].max():+7.1f} deg"
        )

    return "\n".join(lines)


def angles_to_scs(
    angles_deg: np.ndarray,
    bf_stand: Optional[np.ndarray] = None,
    bf_sign: Optional[np.ndarray] = None,
    offset: Optional[np.ndarray] = None,
    clip: bool = True,
) -> np.ndarray:
    """Convert URDF degrees to SCS servo units.

    Formula: SCS = round(511 + (SIGN * (deg - STAND) + offset) / 0.263)

    Parameters
    ----------
    angles_deg : ndarray of shape (n_frames, 12)
    bf_stand : ndarray of shape (12,), default BF_STAND
    bf_sign  : ndarray of shape (12,), default BF_SIGN
    offset   : ndarray of shape (12,), default zeros
    clip     : if True (default) clamp to [0,1023] and return uint16.

    CLIPPING IS WRONG FOR THE DELTA PIPELINE — pass clip=False there.
    ------------------------------------------------------------------
    This function measures SCS against the NOMINAL CENTRE (511), because
    opt_to_bf_stand_deg() maps frame 0 exactly onto BF_STAND, which maps
    exactly onto 511. So the value returned here is really "511 + relative
    motion", NOT the command the servo will receive.

    The command the firmware actually issues is  REF[id] + (scs - 511),
    where REF is the robot's real hand-taught stance. Clamping to [0,1023]
    at the 511 baseline therefore clamps against the WRONG limits, and it
    does so ASYMMETRICALLY: BF_SIGN mirrors the left legs, so one identical
    physical motion runs off the TOP on the right leg and off the BOTTOM on
    the left leg. After clipping the two sides no longer match, and the
    robot splays instead of moving symmetrically.

    Callers that build deltas must pass clip=False and let the delta
    generator do a single clamp against the real REF.

    Returns
    -------
    scs : (n_frames, 12) — uint16 clipped to [0,1023] if clip, else int
          (may fall outside 0..1023, by design).
    """
    if bf_stand is None:
        bf_stand = BF_STAND
    if bf_sign is None:
        bf_sign = BF_SIGN
    if offset is None:
        offset = np.zeros(12)

    scs_float = 511.0 + (bf_sign * (angles_deg - bf_stand) + offset) / 0.263
    scs = np.round(scs_float).astype(int)
    if not clip:
        return scs
    return np.clip(scs, 0, 1023).astype(np.uint16)


def scs_to_angles(
    scs: np.ndarray,
    bf_stand: Optional[np.ndarray] = None,
    bf_sign: Optional[np.ndarray] = None,
    offset: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Convert SCS (0–1023) back to URDF degrees (inverse of angles_to_scs)."""
    if bf_stand is None:
        bf_stand = BF_STAND
    if bf_sign is None:
        bf_sign = BF_SIGN
    if offset is None:
        offset = np.zeros(12)

    return (scs.astype(np.float64) - 511.0) * 0.263 / bf_sign + bf_stand - offset / bf_sign


def generate_header(
    angles_deg: np.ndarray,
    times: np.ndarray,
    var_name: str = "TRAJ",
    description: str = "",
    bf_stand: Optional[np.ndarray] = None,
    bf_sign: Optional[np.ndarray] = None,
    standing_ref: Optional[np.ndarray] = None,
) -> str:
    """Generate a C header file string with URDF-degree keyframes.

    The output follows the same format as mp2_backflip_data.h, compatible
    with the existing ESP32 firmware (bfload handler).

    If ``standing_ref`` is provided, ``angles_deg`` is assumed to be in
    *optimizer* URDF convention and is auto-converted to BF_STAND convention
    via ``opt_to_bf_stand_deg()``.

    Parameters
    ----------
    angles_deg : (n_frames, 12) joint angles in (optimizer or BF_STAND) URDF deg
    times      : (n_frames,) cumulative time in seconds
    var_name   : C identifier prefix for the arrays
    description: comment line for the header
    bf_stand, bf_sign : calibration (for the header comment only)
    standing_ref : if set, auto-convert from optimizer convention

    Returns
    -------
    header_text : str
    """
    if bf_stand is None:
        bf_stand = BF_STAND
    if bf_sign is None:
        bf_sign = BF_SIGN

    # Auto-convert from optimizer convention if standing_ref is provided
    if standing_ref is not None:
        angles_deg = opt_to_bf_stand_deg(angles_deg, standing_ref, bf_stand)

    n_frames = angles_deg.shape[0]

    lines = [
        f"// {var_name}_data.h  -- AUTO-GENERATED by hdf5_tools.py",
    ]
    if description:
        lines.append(f"// {description}")
    lines += [
        f"// {n_frames} frames, servo id order 1..12 "
        f"(FR abd/hip/knee, FL, RR, RL), URDF degrees.",
        "// no L/R swap applied.",
        "// Firmware maps to servos via BF_SIGN[]/BF_STAND[] + your offset[] calibration.",
        "#pragma once",
        f"#define {var_name}_FRAMES {n_frames}",
        "",
    ]

    # Time array
    t_parts = []
    for val in times:
        t_parts.append(f"{val:.3f}f")
    lines.append(
        f"static const float {var_name}_T[{var_name}_FRAMES] = {{ "
        + ", ".join(t_parts) + " };"
    )
    lines.append("")

    # Angle array
    lines.append(
        f"static const float {var_name}_URDF_DEG[{var_name}_FRAMES][12] = {{"
    )
    for f in range(n_frames):
        parts = []
        for col in range(12):
            parts.append(f"{angles_deg[f, col]:8.2f}f")
        comma = "," if f < n_frames - 1 else " "
        lines.append("  { " + ", ".join(parts) + " }" + comma)
    lines.append("};")

    return "\n".join(lines) + "\n"


def generate_json(
    angles_deg: np.ndarray,
    times: np.ndarray,
    torques: Optional[np.ndarray] = None,
    velocities: Optional[np.ndarray] = None,
) -> str:
    """Export trajectory as JSON.

    Returns a JSON string with keys:
        times: [t0, t1, ...]
        dt: nominal time step
        frames: [{servo_name: angle_deg, ...}, ...]
    """
    frames = []
    for f in range(angles_deg.shape[0]):
        frame = {
            "time": round(float(times[f]), 6),
            "angles": {
                SERVO_NAMES[i]: round(float(angles_deg[f, i]), 3)
                for i in range(12)
            },
        }
        if torques is not None:
            frame["torques"] = {
                SERVO_NAMES[i]: round(float(torques[f, i]), 4)
                for i in range(12)
            }
        if velocities is not None:
            frame["velocities"] = {
                SERVO_NAMES[i]: round(float(velocities[f, i]), 3)
                for i in range(12)
            }
        frames.append(frame)

    dts = np.diff(times)
    result = {
        "n_frames": len(frames),
        "dt_nominal": round(float(np.median(dts)), 6) if len(dts) > 0 else None,
        "duration": round(float(times[-1]), 6),
        "times": [round(float(t), 6) for t in times],
        "frames": frames,
    }
    return json.dumps(result, indent=2)


# ---------------------------------------------------------------------------
# Self-test when run directly
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    # Default: look for a nearby HDF5 file
    candidates = [
        # In the backflip repo
        os.path.join(
            os.path.dirname(__file__), "..", "..",
            "Quadruped", "Backflip", "minipupper-backflip",
            "trajectories", "backflip_v4", "backflip_v4.hdf5",
        ),
        # In the workspace root
        os.path.join(os.path.dirname(__file__), "..", "backflip_v4.hdf5"),
    ]

    hdf5_path = sys.argv[1] if len(sys.argv) > 1 else None
    if hdf5_path is None:
        for c in candidates:
            if os.path.exists(c):
                hdf5_path = c
                break

    if hdf5_path is None or not os.path.exists(hdf5_path):
        print("Usage: python hdf5_tools.py <trajectory.hdf5>")
        print("\nNo HDF5 file found. Provide a path as argument.")
        sys.exit(0)

    print(hdf5_summary(hdf5_path))
