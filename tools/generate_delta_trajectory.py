"""
generate_delta_trajectory.py — Convert HDF5 trajectory to a delta-format
C header compatible with the ESP32 firmware's Backflip 3 playback pattern.

The delta format is CALIBRATION-SAFE: only the REF frame (first pose) stores
absolute SCS values. All subsequent frames are stored as SIGNED INT16 DELTAS
relative to REF. After recalibration you only update REF — the deltas stay
the same because they encode RELATIVE joint motion.

Output matches the pattern in hardcode_backflip_angle.h:
    static const uint16_t REF[13];           // frame 0, absolute SCS
    static const int16_t DELTA[N-1][13];     // frames 1..N-1, offsets from REF
    static const int MOVE_MS[N];             // move time per frame
    static const int DELAY_MS[N];            // dwell time per frame

Usage:
    python generate_delta_trajectory.py <trajectory.hdf5> [options]

Options:
    --name NAME       C identifier prefix (default: derived from filename)
    -o DIR            Output directory (default: ../main/)
    --frames N        Only process first N frames
    --move-ms MS      Move time per frame in ms (default: from HDF5 timing)
    --delay-ms MS     Dwell time per frame in ms (default: 0)
    --speed FACTOR    Time multiplier for move/delay (default: 1.0)

Examples:
    # Generate trajectory header for bfv1:
    python generate_delta_trajectory.py bfv1.hdf5 --name HDF5_TRAJ1 -o ../main/

    # Limit to 28 frames with fixed 100ms move time:
    python generate_delta_trajectory.py bfv1.hdf5 --frames 28 --move-ms 100
"""

import argparse
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hdf5_tools import (
    load_hdf5,
    extract_joint_angles_deg,
    get_timesteps,
    get_standing_reference,
    opt_to_bf_stand_deg,
    angles_to_scs,
    BF_STAND,
    BF_SIGN,
    SERVO_NAMES,
)


# Robot stand pose in raw SCS, servo ids 1..12 (index 0 unused).
# Taken from BF3_REF in main/hardcode_backflip_angle.h — the hand-taught
# reference stance the working backflip plays from.
#
# This MUST be a pose the robot can actually hold. The firmware plays frame 0
# LITERALLY (main.c: rec_frames[f][id] = REF[id] + DELTA[f-1][id], written
# straight into goal[] — no offset[] applied, no clamping), so emitting the
# nominal centre 511 makes every servo slam to centre on the first move:
# the abduction servos alone travel 1023 -> 511, about 135 degrees.
#
# NOTE: this list previously read
#   [0, 1023, 511, 684, 1023, 513, 671, 1023, 504, 731, 471, 512, 585]
# which did NOT match BF3_REF despite the comment above claiming it did. The
# abduction servos (ids 1/4/7) were 1023 where BF3_REF has 54/113/26 — about
# 135 degrees of splay per leg on the very first move. Now actually copied
# from BF3_REF.
DEFAULT_REF_SCS = [0, 54, 473, 596, 113, 536, 420, 26, 478, 515, 531, 544, 439]


def parse_ref(text: str) -> np.ndarray:
    """Parse a REF pose from '1023,511,684,...' (12 or 13 values)."""
    vals = [int(float(v)) for v in text.replace("{", " ").replace("}", " ")
            .replace(";", " ").replace(",", " ").split()]
    if len(vals) == 12:
        vals = [0] + vals
    if len(vals) != 13:
        raise ValueError(f"--ref needs 12 or 13 SCS values, got {len(vals)}")
    return np.array(vals, dtype=int)


def generate_delta_header(
    scs_frames: np.ndarray,
    move_ms: list,
    delay_ms: list,
    var_name: str = "HDF5_TRAJ",
    description: str = "",
    ref_override: np.ndarray = None,
    rel_to_ini: bool = False,
    source_name: str = "",
) -> str:
    """Generate a delta-format C header string.

    Parameters
    ----------
    scs_frames : (n_frames, 13) — SCS values, column 0 unused (reserved)
    move_ms    : list of n_frames ints — move time per frame
    delay_ms   : list of n_frames ints — dwell time per frame
    var_name   : C identifier prefix
    description: header comment

    Returns
    -------
    header_text : str
    """
    n_frames = scs_frames.shape[0]

    # Deltas are always measured against the TRAJECTORY's own frame 0, since
    # that is the pose the optimizer treats as the standing stance.
    traj_ref = scs_frames[0, :].astype(int)

    # REF emitted into the header is the ROBOT's real stand pose. The whole
    # point of the delta format: deltas encode relative joint motion, REF is
    # whatever absolute pose the robot currently calibrates to.
    ref = (traj_ref if ref_override is None
           else np.asarray(ref_override, dtype=int)).copy()

    # DELTA = frames 1..N-1 as signed offsets from REF
    n_delta = n_frames - 1
    delta = np.zeros((n_delta, 13), dtype=np.int16)
    n_clamped = 0
    for f in range(1, n_frames):
        for col in range(13):
            d = int(scs_frames[f, col]) - traj_ref[col]
            # The firmware does NOT clamp REF+DELTA before writing goal[],
            # so keep the sum inside the servo's 0..1023 range here.
            lo, hi = -ref[col], 1023 - ref[col]
            if d < lo or d > hi:
                n_clamped += 1
                d = max(lo, min(hi, d))
            delta[f - 1, col] = d
    if n_clamped:
        print(f"  WARNING: {n_clamped} delta values clamped to keep "
              f"REF+DELTA within 0..1023 (trajectory exceeds servo range)")

    # ---- Per-joint fit report -------------------------------------------
    # A clamped joint stops moving and parks against its limit for those
    # frames, so the leg freezes mid-flip. Because BF_SIGN mirrors the left
    # legs, a right joint clamps HIGH while its left twin clamps LOW — the
    # two sides stop matching and the robot splays. Show exactly which joints
    # do not fit so this is never silent again.
    names = ["-", "FR_abd", "FR_HFE", "FR_KFE", "FL_abd", "FL_HFE", "FL_KFE",
             "RR_abd", "RR_HFE", "RR_KFE", "RL_abd", "RL_HFE", "RL_KFE"]
    print("\n  fit report (command = REF + relative motion, must stay 0..1023):")
    print(f"    {'joint':>7s} {'REF':>5s} {'needs':>14s} {'travel':>9s}  status")
    max_travel = 0.0
    fit_scale = 1.0   # largest --scale for which EVERY joint stays in range
    for col in range(1, 13):
        rel = scs_frames[:, col].astype(int) - int(traj_ref[col])
        want = ref[col] + rel
        lo_w, hi_w = int(want.min()), int(want.max())
        travel = (hi_w - lo_w) * 0.263
        n_bad = int(((want < 0) | (want > 1023)).sum())
        flag = "ok" if n_bad == 0 else f"CLAMPED {n_bad}/{n_frames} frames"
        print(f"    {names[col]:>7s} {ref[col]:5d} {lo_w:6d}..{hi_w:6d} "
              f"{travel:7.1f}deg  {flag}")
        max_travel = max(max_travel, travel)
        # Headroom is measured from REF, not from the middle of the range, so a
        # joint whose REF sits near a limit runs out far sooner than its travel
        # alone suggests. Both directions must fit.
        rel_lo, rel_hi = int(rel.min()), int(rel.max())
        if rel_lo < 0:
            fit_scale = min(fit_scale, ref[col] / float(-rel_lo))
        if rel_hi > 0:
            fit_scale = min(fit_scale, (1023 - ref[col]) / float(rel_hi))
    if n_clamped:
        print(f"\n  Widest joint needs {max_travel:.0f} deg of travel; the servo "
              f"has 269 deg total, and REF eats into that on both sides.")
        print(f"  No choice of --ref fixes this. Either re-run the optimizer with "
              f"joint limits that match the hardware,")
        print(f"  or shrink the motion:  --scale {fit_scale:.2f}  "
              f"(largest value where every joint fits).")

    lines = [
        f"// {var_name}.h  -- AUTO-GENERATED by generate_delta_trajectory.py",
    ]
    if description:
        lines.append(f"// {description}")
    lines += [
        f"//",
        f"// DELTA format (calibration-safe):",
        f"//   REF[]      = frame 0, absolute SCS values",
        f"//   DELTA[][]  = frames 1..{n_delta}, signed offsets from REF",
        f"//   MOVE_MS[]  = move time per frame (ms)",
        f"//   DELAY_MS[] = dwell time per frame (ms)",
        f"//",
        f"// After recalibration, only update REF[]. The deltas stay the same.",
        f"//",
        f"// {n_frames} frames, servo id order 1..12 (FR abd/hip/knee, FL, RR, RL).",
        f"// Index [0] of each row is unused (reserved for 1-based servo indexing).",
        f"#pragma once",
        f"#include <stdint.h>",
        f"",
        f"#define {var_name}_FRAMES {n_frames}",
        f"#define {var_name}_SOURCE \"{source_name}\"",
        # 1 = the firmware must add DELTA onto its LIVE Ini stance
        #     (ini[id] = 511 + offset[id]/0.263) and ignore REF[] below.
        #     The motion then always starts from wherever the robot stands.
        # 0 = use the baked-in REF[] as the starting pose (old behaviour).
        f"#define {var_name}_REL_TO_INI {1 if rel_to_ini else 0}",
        f"",
    ]

    # ---- REF ----
    lines.append(f"// ---- REFERENCE POSE (frame 0) ----")
    lines.append(f"// Absolute SCS values for the starting stance.")
    lines.append(f"static const uint16_t {var_name}_REF[13] = {{")
    ref_parts = []
    for col in range(13):
        ref_parts.append(f"{ref[col]:5d}")
    lines.append("    /* idx  1    2    3    4    5    6    7    8    9   10   11   12 */")
    lines.append("           " + ", ".join(ref_parts))
    lines.append("};")
    lines.append("")

    # ---- DELTA ----
    lines.append(f"// ---- DELTA FRAMES (frames 1..{n_delta}, relative to REF) ----")
    lines.append(f"// Each row is a signed offset from REF. int16_t for negative values.")
    lines.append(f"// After recalibration these do NOT change.")
    lines.append(f"static const int16_t {var_name}_DELTA[{var_name}_FRAMES - 1][13] = {{")
    for f in range(n_delta):
        parts = []
        for col in range(13):
            parts.append(f"{delta[f, col]:6d}")
        comma = "," if f < n_delta - 1 else " "
        lines.append("    {" + ", ".join(parts) + " }" + comma)
    lines.append("};")
    lines.append("")

    # ---- MOVE_MS ----
    lines.append(f"// ---- PER-FRAME TIMING ----")
    lines.append(f"// {var_name}_MOVE_MS[f]  = time to MOVE into frame f from previous pose.")
    mv_parts = [f"{ms:5d}" for ms in move_ms]
    lines.append(f"static const int {var_name}_MOVE_MS[{var_name}_FRAMES] = {{"
                 + ", ".join(mv_parts) + " };")

    # ---- DELAY_MS ----
    lines.append(f"// {var_name}_DELAY_MS[f] = time to DWELL on frame f after arriving.")
    dl_parts = [f"{ms:5d}" for ms in delay_ms]
    lines.append(f"static const int {var_name}_DELAY_MS[{var_name}_FRAMES] = {{"
                 + ", ".join(dl_parts) + " };")

    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(
        description="Generate delta-format C header from HDF5 trajectory",
    )
    parser.add_argument("hdf5", help="Path to trajectory .hdf5 file")
    parser.add_argument("--name", default=None, metavar="NAME",
                        help="C identifier prefix (default: derived from filename)")
    parser.add_argument("-o", "--output", default=None,
                        help="Output directory (default: ../main/)")
    parser.add_argument("--frames", type=int, default=0,
                        help="Only process first N frames")
    parser.add_argument("--move-ms", type=int, default=None,
                        help="Fixed move time per frame in ms (overrides HDF5 timing)")
    parser.add_argument("--delay-ms", type=int, default=0,
                        help="Fixed dwell time per frame in ms (default: 0)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Time multiplier (default: 1.0, 2.0 = 2x faster)")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Amplitude scale about frame 0 (default: 1.0). "
                             "The optimizer ignores the servo's mechanical "
                             "range; use <1.0 to shrink the motion until the "
                             "fit report shows 0 clamped values.")
    parser.add_argument("--min-move-ms", type=int, default=10,
                        help="Minimum move time in ms (default: 10)")
    parser.add_argument("--ref", default=None, metavar="SCS",
                        help="Robot stand pose as 12 or 13 comma-separated SCS "
                             "values (e.g. from `recdump` row 0). Default: the "
                             "BF3_REF stance. Use 'traj' to emit the raw "
                             "trajectory frame 0 (nominal centre — will slam).")
    parser.add_argument("--first-move-ms", type=int, default=800,
                        help="Time to travel from the Ini pose into REF at the "
                             "start of playback (default: 800). NOT scaled by "
                             "--speed. The old behaviour was ~10ms = a slam.")
    parser.add_argument("--first-delay-ms", type=int, default=300,
                        help="Dwell on REF before the motion starts (default: 300)")
    parser.add_argument("--no-knee-coupling", action="store_true", default=False,
                        help="Feed the URDF's thigh-RELATIVE knee angle straight "
                             "to the servo. Only correct if the knee servo is "
                             "mounted at the knee. Mini Pupper uses a parallel "
                             "linkage, so the default (knee = HFE + KFE) is right.")
    parser.add_argument("--flip-left", action="store_true", default=False,
                        help="Negate left-side DELTA values (FL/RL HFE+KFE) "
                             "— use if left legs move opposite to expected")
    parser.add_argument("--mirror-left", action="store_true", default=False,
                        help="Mirror right-side SCS to left side (FL=mirror(FR), RL=mirror(RR)). "
                             "Use when trajectory is asymmetric and --flip-left is insufficient. "
                             "This replaces left-side SCS with true mirror of right-side SCS.")

    args = parser.parse_args()

    if not os.path.exists(args.hdf5):
        print(f"ERROR: File not found: {args.hdf5}")
        sys.exit(1)

    # ---- Load & convert ----
    print(f"Loading {args.hdf5} ...")
    data = load_hdf5(args.hdf5)
    import hdf5_tools as _ht
    _ht.KNEE_ABSOLUTE = not args.no_knee_coupling
    print(f"  knee convention: "
          f"{'ABSOLUTE calf (HFE+KFE, parallel linkage)' if _ht.KNEE_ABSOLUTE else 'thigh-RELATIVE (raw KFE)'}")
    raw_angles = extract_joint_angles_deg(data)
    t, dt = get_timesteps(data)
    standing_ref = get_standing_reference(data)

    n_total = raw_angles.shape[0]
    n = min(args.frames, n_total) if args.frames > 0 else n_total
    raw_angles = raw_angles[:n]
    t = t[:n]

    print(f"  {n} frames, {t[-1]:.3f}s total, dt={dt*1000:.1f}ms ({1/dt:.0f}Hz)")

    # ---- Convert to BF_STAND then to SCS ----
    bf_angles = opt_to_bf_stand_deg(raw_angles, standing_ref)
    # clip=False is REQUIRED here. angles_to_scs() measures against the nominal
    # centre 511, but the firmware commands REF + (scs - 511) against the real
    # stance. Clipping at the 511 baseline clamps to the wrong limits and, worse,
    # does it asymmetrically (BF_SIGN mirrors the left legs, so the same physical
    # motion runs off the top on the right and off the bottom on the left).
    # Keep the motion unclipped and clamp ONCE, later, against the real REF.
    scs_all = angles_to_scs(bf_angles, clip=False)  # (n, 12), may exceed 0..1023

    # Optional amplitude scaling about frame 0. The optimizer does not know the
    # servo's mechanical range, so its trajectory routinely demands more travel
    # than the hardware has. Shrinking about frame 0 keeps the shape of the
    # motion and the stance pose, and just reduces how far the joints throw.
    if args.scale != 1.0:
        scs_all = np.round(scs_all[0:1, :] + (scs_all - scs_all[0:1, :]) * args.scale).astype(int)
        print(f"Amplitude scaled by {args.scale:g} about frame 0")

    # Expand to 13 columns (index 0 unused, 1..12 for servos)
    scs_13 = np.zeros((n, 13), dtype=int)
    scs_13[:, 1:13] = scs_all  # scs_all is (n, 12), map to columns 1..12

    # Print standing reference info
    print(f"\nStanding reference (frame 0):")
    for col in range(12):
        if "ABD" not in SERVO_NAMES[col]:
            print(f"  {SERVO_NAMES[col]:>8s}: URDF={raw_angles[0,col]:+.1f}  "
                  f"BF_STAND={bf_angles[0,col]:+.1f}  SCS={scs_13[0,col+1]:d}")
    print()

    # ---- Mirror/flip left-side SCS values if requested ----
    # In 13-col format: FR_HFE=2, FR_KFE=3, FL_HFE=5, FL_KFE=6, RR_HFE=8, RR_KFE=9, RL_HFE=11, RL_KFE=12
    LEFT_COLS_13 = [5, 6, 11, 12]
    RIGHT_COLS_13 = [2, 3, 8, 9]   # FR_HFE, FR_KFE, RR_HFE, RR_KFE

    if args.mirror_left:
        print("Mirroring left-side SCS from right side (FL=mirror(FR), RL=mirror(RR)) ...")
        ref_row = scs_13[0, :].copy()
        for lc, rc in zip(LEFT_COLS_13, RIGHT_COLS_13):
            # Mirror: left_SCS = REF - (right_SCS - REF) = 2*REF - right_SCS
            right_delta = scs_13[:, rc].astype(int) - int(ref_row[rc])
            scs_13[:, lc] = np.clip(ref_row[lc].astype(int) - right_delta, 0, 1023).astype(np.uint16)
        print(f"  Left columns {LEFT_COLS_13} now mirror right columns {RIGHT_COLS_13}")
        print()

    elif args.flip_left:
        print("Flipping left-side SCS values (FL/RL HFE+KFE) ...")
        ref_row = scs_13[0, :].copy()
        for c in LEFT_COLS_13:
            # Negate the delta: new_scs = REF - (old_scs - REF) = 2*REF - old_scs
            delta = scs_13[:, c].astype(int) - int(ref_row[c])
            scs_13[:, c] = np.clip(ref_row[c].astype(int) - delta, 0, 1023).astype(np.uint16)
        print(f"  Left-side columns flipped: {LEFT_COLS_13}")
        print()

    # ---- Compute timing ----
    if args.move_ms is not None:
        move_ms = [max(args.min_move_ms, int(args.move_ms / args.speed))] * n
    else:
        # Derive move times from HDF5 knot durations
        dts = np.diff(t, prepend=0.0)
        move_ms = []
        for i in range(n):
            ms = int(dts[i] * 1000 / args.speed)
            ms = max(args.min_move_ms, ms)
            move_ms.append(ms)

    delay_ms = [max(0, int(args.delay_ms / args.speed))] * n

    # ---- Frame 0 is the APPROACH to the reference stance, not part of the
    # motion. main.c interp_to()s from the Ini pose into rec_frames[0] using
    # MOVE_MS[0]; the HDF5 knot duration for frame 0 is 0, which clamped to
    # --min-move-ms (10ms) and slammed the robot into the stance.
    move_ms[0] = max(1, args.first_move_ms)
    delay_ms[0] = max(0, args.first_delay_ms)

    # ---- Reference stance ----
    rel_to_ini = False
    if args.ref is not None and args.ref.strip().lower() == "ini":
        # Start the motion from the robot's OWN standing pose, whatever that
        # currently is. The firmware builds its Ini frame as
        #     ini[id] = 511 + offset[id]/0.263
        # from the live `setcal` calibration, so if the firmware adds our
        # deltas onto ini[] instead of onto a baked-in REF, the trajectory
        # always starts from the stance the robot is already holding. No
        # teaching, no re-taught REF after recalibration.
        #
        # We emit 511 here (the nominal centre, i.e. offset == 0) purely so the
        # fit report below has something to measure against. The real clamping
        # happens on the robot at load time, and `framecheck` reports it.
        rel_to_ini = True
        ref_override = np.full(13, 511, dtype=int)
        ref_override[0] = 0
        print("REF: robot's live Ini stance (relative mode)")
        print("     fit report assumes zero calibration offset; run `framecheck`")
        print("     on the robot for the exact numbers.")
    elif args.ref is None:
        ref_override = np.array(DEFAULT_REF_SCS, dtype=int)
        print(f"REF: default BF3_REF stance {[int(v) for v in ref_override[1:]]}")
        print("     (override with --ref '<recdump row 0>' if recalibrated)")
    elif args.ref.strip().lower() == "traj":
        ref_override = None
        print("REF: raw trajectory frame 0 (nominal centre) — "
              "expect a large jump on the first move")
    else:
        ref_override = parse_ref(args.ref)
        print(f"REF: user-supplied {[int(v) for v in ref_override[1:]]}")

    print(f"Timing: approach={move_ms[0]}ms +{delay_ms[0]}ms dwell, "
          f"then move={move_ms[1] if n > 1 else 0}-{move_ms[-1]}ms, "
          f"delay={delay_ms[-1]}ms per frame")
    print(f"First 5 moves: {move_ms[:5]}")
    # Summary only. Printing the full (n_frames-1) x 12 delta matrix here
    # buried the fit report, which is the part that actually matters.
    _d = scs_13[1:, 1:].astype(int) - scs_13[0, 1:].astype(int)
    print(f"Delta range: {_d.min()} .. {_d.max()} ticks "
          f"({_d.min()*0.263:.0f} .. {_d.max()*0.263:.0f} deg)")

    # ---- Derive name ----
    if args.name:
        var_name = args.name.upper().replace(" ", "_").replace("-", "_")
    else:
        var_name = os.path.splitext(os.path.basename(args.hdf5))[0].upper()
        var_name = var_name.replace(" ", "_").replace("-", "_")

    # ---- Output dir ----
    output_dir = args.output
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "main")
    os.makedirs(output_dir, exist_ok=True)

    # ---- Generate header ----
    header_text = generate_delta_header(
        scs_13, move_ms, delay_ms, var_name=var_name,
        description=f"source: {os.path.basename(args.hdf5)}",
        ref_override=ref_override,
        rel_to_ini=rel_to_ini,
        source_name=os.path.basename(args.hdf5),
    )

    header_path = os.path.join(output_dir, f"{var_name.lower()}.h")
    with open(header_path, "w") as f:
        f.write(header_text)

    print(f"\nGenerated: {header_path}")
    print(f"  {n} frames, REF + {n-1} deltas")
    print()
    print("To use in the firmware:")
    print(f"  1. #include \"{var_name.lower()}.h\" in main.c")
    print(f"  2. Add load_{var_name.lower()}() function (copy load_bf3 pattern)")
    print(f"  3. Add /{var_name.lower()}load and /{var_name.lower()}play HTTP handlers")
    print(f"  4. Add web UI buttons")
    print(f"  5. Rebuild and flash")


if __name__ == "__main__":
    main()
