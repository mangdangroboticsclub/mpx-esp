"""
newflip.py — ONE COMMAND to put a new HDF5 trajectory on the robot.

    python newflip.py <trajectory.hdf5>              convert + build + flash
    python newflip.py <trajectory.hdf5> --no-flash   convert only
    python newflip.py <trajectory.hdf5> --scale 0.8  force an amplitude

What it does, in order:

  1. Converts the HDF5 into main/current_flip.h.
     ALWAYS the same filename and the same C prefix (CURRENT_FLIP), so main.c
     never has to change. Dropping in a new trajectory is a re-run of this
     script, not a code edit.

  2. Picks the amplitude automatically. The optimizer does not know how far
     your servos can actually travel, so its trajectory usually demands more
     range than the hardware has. This script finds the largest --scale at
     which every joint stays inside 0..1023, and uses it. Nothing gets
     silently clamped and parked against a stop.

  3. Anchors the motion to the robot's OWN standing pose (--ref ini). The
     firmware adds the deltas onto its live Ini frame, which comes from your
     `setcal` calibration. So the flip starts from wherever the robot is
     already standing when you power it on — no teaching, no re-taught
     reference pose, nothing to redo after a recalibration.

  4. Runs `idf.py build flash` unless you pass --no-flash.

After flashing, on the web page:  Load current flip  ->  step Verify < >  ->
Play (slow motion is ON by default).
"""

import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
MAIN_DIR = os.path.join(PROJECT, "main")
CONVERTER = os.path.join(HERE, "generate_delta_trajectory.py")

# Fixed identity of the "currently loaded" trajectory slot in the firmware.
# main.c includes current_flip.h and refers to CURRENT_FLIP_* — never anything
# trajectory-specific — so a new HDF5 needs no C changes at all.
VAR_NAME = "CURRENT_FLIP"
HEADER = "current_flip.h"


def run_converter(hdf5, scale, extra=()):
    """Run generate_delta_trajectory.py, return (ok, combined_output)."""
    cmd = [sys.executable, CONVERTER, hdf5,
           "--name", VAR_NAME,
           "-o", MAIN_DIR,
           "--ref", "ini",
           "--scale", f"{scale:.4f}", *extra]
    p = subprocess.run(cmd, capture_output=True, text=True)
    return p.returncode == 0, (p.stdout or "") + (p.stderr or "")


def find_best_scale(hdf5):
    """Ask the converter what scale fits, then confirm it actually does.

    The converter's fit report already prints the largest scale at which every
    joint stays in range. We read it back, apply it, and verify no clamping
    remains — rather than trusting the suggestion blindly.
    """
    ok, out = run_converter(hdf5, 1.0)
    if not ok:
        return None, out
    if "CLAMPED" not in out:
        return 1.0, out

    suggested = None
    for line in out.splitlines():
        if "--scale" in line and "largest value" in line:
            for tok in line.replace(",", " ").split():
                try:
                    suggested = float(tok)
                    break
                except ValueError:
                    continue
    if suggested is None:
        return None, out

    # Walk down in small steps until nothing clamps. Normally the first try
    # succeeds; the loop is here so a rounding edge case cannot ship a header
    # that still clamps.
    s = min(suggested, 1.0)
    for _ in range(20):
        ok, out2 = run_converter(hdf5, s)
        if ok and "CLAMPED" not in out2:
            return s, out2
        s -= 0.02
        if s <= 0.05:
            break
    return None, out


def main():
    ap = argparse.ArgumentParser(
        description="Convert an HDF5 trajectory and flash it to the robot.")
    ap.add_argument("hdf5", help="Path to the new trajectory .hdf5")
    ap.add_argument("--scale", type=float, default=None,
                    help="Force an amplitude scale (default: auto-fit)")
    ap.add_argument("--no-flash", action="store_true",
                    help="Generate the header but do not build or flash")
    ap.add_argument("--port", default=None,
                    help="Serial port for idf.py (e.g. COM5)")
    args = ap.parse_args()

    if not os.path.exists(args.hdf5):
        print(f"ERROR: no such file: {args.hdf5}")
        sys.exit(1)

    print(f"[1/3] Converting {os.path.basename(args.hdf5)} ...")
    if args.scale is not None:
        scale = args.scale
        ok, out = run_converter(args.hdf5, scale)
        if not ok:
            print(out)
            sys.exit(1)
    else:
        scale, out = find_best_scale(args.hdf5)
        if scale is None:
            print(out)
            print("\nERROR: could not find an amplitude that fits the servos.")
            print("This trajectory is far outside the robot's joint range.")
            sys.exit(1)

    # Echo the fit report — it is the thing worth reading. Stop before the
    # converter's generic "now wire it into main.c" steps, which are already
    # done: main.c includes current_flip.h permanently.
    printing = False
    for line in out.splitlines():
        # Match the TABLE header specifically. Plain "fit report" also appears
        # in the --ref ini notice further up, which would start the echo early
        # and drag in the whole 72-row delta dump.
        if "fit report (command" in line:
            printing = True
        if line.startswith("To use in the firmware") or "Generated:" in line:
            break
        if printing:
            print(("   " + line.strip()) if line.strip() else "")

    header_path = os.path.join(MAIN_DIR, HEADER)
    generated = os.path.join(MAIN_DIR, f"{VAR_NAME.lower()}.h")
    if generated != header_path and os.path.exists(generated):
        shutil.move(generated, header_path)

    if not os.path.exists(header_path):
        print(f"ERROR: converter did not produce {header_path}")
        sys.exit(1)

    print(f"\n   amplitude scale : {scale:.2f}"
          f"{'  (auto-fit)' if args.scale is None else '  (forced)'}")
    print(f"   wrote           : main/{HEADER}")
    print(f"   starts from     : the robot's own stance (no teaching needed)")

    if args.no_flash:
        print("\n[2/3] --no-flash given, stopping here.")
        print(f"      Build it yourself with:  cd {PROJECT} && idf.py build flash")
        return

    print("\n[2/3] Building ...")
    idf = shutil.which("idf.py")
    if idf is None:
        print("   idf.py not on PATH. Open the ESP-IDF terminal and run:")
        print(f"     cd {PROJECT} && idf.py build flash monitor")
        sys.exit(1)

    cmd = [idf]
    if args.port:
        cmd += ["-p", args.port]
    cmd += ["build", "flash"]
    rc = subprocess.run(cmd, cwd=PROJECT).returncode
    if rc != 0:
        print("\nERROR: build/flash failed. Fix the error above and re-run.")
        sys.exit(rc)

    print("\n[3/3] Done.")
    print("   On the web page:")
    print("     1. check 'Slow motion' is ON (green)")
    print("     2. Load current flip")
    print("     3. step Verify < >  -- no red rows means every joint is in range")
    print("     4. Play")


if __name__ == "__main__":
    main()
