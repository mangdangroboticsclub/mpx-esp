"""
extract_hdf5.py — Extract joint angles & timesteps from a trajectory HDF5 file
and export to C header, JSON, CSV, or human-readable summary.

Usage:
    python extract_hdf5.py <trajectory.hdf5> [options]

Output options:
    --summary         Print a human-readable summary (default)
    --header NAME     Generate C header file (NAME_data.h)
    --json FILE       Export as JSON
    --csv FILE        Export as CSV
    --scs             Output SCS (0-1023) values instead of URDF degrees
    --include-torques Include torque data in JSON/CSV output
    -o, --output DIR  Output directory (default: current directory)

Examples:
    # See what's inside:
    python extract_hdf5.py backflip_v4.hdf5 --summary

    # Generate C header:
    python extract_hdf5.py backflip_v4.hdf5 --header BF_V4 -o ../main/

    # Export as JSON:
    python extract_hdf5.py backflip_v4.hdf5 --json trajectory.json

    # Export SCS values as CSV:
    python extract_hdf5.py backflip_v4.hdf5 --csv trajectory.csv --scs
"""

import argparse
import os
import sys

# Add parent dir to path so we can import hdf5_tools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hdf5_tools import (
    load_hdf5,
    extract_joint_angles_deg,
    extract_joint_velocities_deg_s,
    extract_torques,
    get_timesteps,
    get_standing_reference,
    opt_to_bf_stand_deg,
    opt_to_servo_deg,
    hdf5_summary,
    angles_to_scs,
    generate_header,
    generate_json,
    SERVO_NAMES,
    BF_STAND,
    BF_SIGN,
)


def main():
    parser = argparse.ArgumentParser(
        description="Extract Mini Pupper trajectory from HDF5",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("hdf5", help="Path to trajectory .hdf5 file")
    parser.add_argument(
        "--summary", action="store_true", default=False,
        help="Print human-readable summary (default if no output option given)",
    )
    parser.add_argument(
        "--header", metavar="NAME",
        help="Generate C header file NAME_data.h",
    )
    parser.add_argument(
        "--json", metavar="FILE",
        help="Export trajectory as JSON file",
    )
    parser.add_argument(
        "--csv", metavar="FILE",
        help="Export trajectory as CSV file",
    )
    parser.add_argument(
        "--scs", action="store_true", default=False,
        help="Output SCS (0-1023) values instead of URDF degrees",
    )
    parser.add_argument(
        "--include-torques", action="store_true", default=False,
        help="Include torque data in output",
    )
    parser.add_argument(
        "-o", "--output", metavar="DIR", default=".",
        help="Output directory (default: current dir)",
    )
    parser.add_argument(
        "--no-timestamps", action="store_true", default=False,
        help="Omit timestamps from CSV/JSON output",
    )
    parser.add_argument(
        "--frames", type=int, default=0, metavar="N",
        help="Limit output to first N frames",
    )
    parser.add_argument(
        "--raw", action="store_true", default=False,
        help="Keep raw optimizer URDF angles (don't auto-convert to BF_STAND convention)",
    )
    parser.add_argument(
        "--standing-hfe", type=float, default=None, metavar="DEG",
        help="Manually specify standing HFE angle in optimizer convention (overrides auto-detect)",
    )
    parser.add_argument(
        "--standing-kfe", type=float, default=None, metavar="DEG",
        help="Manually specify standing KFE angle in optimizer convention (overrides auto-detect)",
    )

    args = parser.parse_args()

    if not os.path.exists(args.hdf5):
        print(f"ERROR: File not found: {args.hdf5}")
        sys.exit(1)

    # Load data
    print(f"Loading {args.hdf5} ...")
    data = load_hdf5(args.hdf5)
    raw_angles = extract_joint_angles_deg(data)
    t, dt = get_timesteps(data)

    n = raw_angles.shape[0]
    if args.frames > 0 and args.frames < n:
        raw_angles = raw_angles[: args.frames]
        t = t[: args.frames]
        n = args.frames
        print(f"Truncated to {n} frames")

    # ---- Auto-detect or override standing reference ----
    standing_ref = get_standing_reference(data)

    if args.standing_hfe is not None or args.standing_kfe is not None:
        # Manual override
        for col in range(12):
            name = SERVO_NAMES[col]
            if "HFE" in name and args.standing_hfe is not None:
                standing_ref[col] = args.standing_hfe
            elif "KFE" in name and args.standing_kfe is not None:
                standing_ref[col] = args.standing_kfe

    print(f"Standing reference (auto-detected from first frame):")
    for col in range(12):
        if "ABD" not in SERVO_NAMES[col]:
            print(f"  {SERVO_NAMES[col]:>8s}: {standing_ref[col]:+.2f} deg  "
                  f"->  BF_STAND={BF_STAND[col]:+.0f} deg  "
                  f"(offset={BF_STAND[col]-standing_ref[col]:+.2f})")
    print()

    # ---- Convert to BF_STAND convention (unless --raw) ----
    if not args.raw:
        angles = opt_to_bf_stand_deg(raw_angles, standing_ref)
        print("Angles converted to BF_STAND convention (use --raw to disable).")
    else:
        angles = raw_angles
        print("WARNING: --raw: output uses optimizer URDF convention, NOT BF_STAND!")
        print("  These angles will NOT work directly with the ESP32 firmware.")
    print()

    # Default to summary if no output format specified
    if not any([args.header, args.json, args.csv, args.summary]):
        args.summary = True

    # ---- Summary ----
    if args.summary:
        print()
        print(hdf5_summary(args.hdf5))

    # ---- Convert to SCS if requested ----
    output_values = angles
    value_label = "URDF degrees (BF_STAND)"

    if args.scs:
        output_values = angles_to_scs(angles)
        value_label = "SCS (0-1023)"

    # ---- JSON ----
    if args.json:
        json_path = os.path.join(args.output, args.json)
        torques = None
        velocities = None
        if args.include_torques and "tau" in data:
            torques = extract_torques(data)
            if args.frames > 0:
                torques = torques[: args.frames]
        if args.include_torques and "v" in data:
            velocities = extract_joint_velocities_deg_s(data)
            if args.frames > 0:
                velocities = velocities[: args.frames]

        # Export angles in BF_STAND convention (or raw if --raw)
        json_t_data = t if not args.no_timestamps else None
        json_str = generate_json(angles, json_t_data, torques, velocities)
        with open(json_path, "w") as f:
            f.write(json_str)
        print(f"Wrote {json_path}  ({n} frames, {value_label})")

    # ---- CSV ----
    if args.csv:
        csv_path = os.path.join(args.output, args.csv)
        import csv

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)

            # Header row
            header = []
            if not args.no_timestamps:
                header.append("time_s")
            header += SERVO_NAMES
            writer.writerow(header)

            # Data rows
            for row_idx in range(n):
                row = []
                if not args.no_timestamps:
                    row.append(f"{t[row_idx]:.6f}")
                row += [f"{output_values[row_idx, col]:.3f}" for col in range(12)]
                writer.writerow(row)

        print(f"Wrote {csv_path}  ({n} frames, {value_label})")

    # ---- C Header ----
    if args.header:
        var_name = args.header.upper().replace(" ", "_").replace("-", "_")
        header_path = os.path.join(
            args.output, f"{var_name.lower()}_data.h"
        )
        # If not --raw, angles is already in BF_STAND; pass raw_angles + standing_ref
        if args.raw:
            header_text = generate_header(
                raw_angles, t, var_name=var_name,
                description=f"source: {os.path.basename(args.hdf5)} (RAW — not calibrated!)",
            )
        else:
            header_text = generate_header(
                raw_angles, t, var_name=var_name,
                description=f"source: {os.path.basename(args.hdf5)}",
                standing_ref=standing_ref,
            )
        with open(header_path, "w") as f:
            f.write(header_text)
        print(f"Wrote {header_path}  ({n} frames, BF_STAND URDF degrees)")

        # Also generate SCS version if requested
        if args.scs:
            scs_path = os.path.join(
                args.output, f"{var_name.lower()}_scs_data.h"
            )
            scs_angles = angles_to_scs(angles)
            scs_text = generate_header(
                scs_angles, t, var_name=f"{var_name}_SCS",
                description=f"source: {os.path.basename(args.hdf5)} (SCS units)",
            )
            with open(scs_path, "w") as f:
                f.write(scs_text)
            print(f"Wrote {scs_path}  ({n} frames, SCS units)")

    if not args.summary:
        print(f"\nDone. {n} frames extracted."
              f"  Duration: {t[-1]:.3f}s  dt: {dt*1000:.1f}ms ({1/dt:.0f}Hz)")


if __name__ == "__main__":
    main()
