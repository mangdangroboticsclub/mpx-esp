"""
flash_trajectory.py — Upload trajectory frames to a running Mini Pupper ESP32
via HTTP, and/or generate the C header file for firmware compilation.

This tool supports three workflows:

  1. GENERATE  — Create a C header file to compile into the firmware
                 (same as extract_hdf5.py --header)

  2. UPLOAD    — Stream trajectory frames to the ESP32 over WiFi/HTTP in
                 real time, one frame at a time. This works with any
                 trajectory length (no MAX_FRAMES limit).

  3. LOAD      — Load frames into the ESP32's teach/record buffer via HTTP
                 (limited to MAX_FRAMES=32). Use Verify/Play from the web UI.

Usage:
    # Generate C header for compilation:
    python flash_trajectory.py backflip_v4.hdf5 --generate

    # Stream trajectory in real-time (robot moves as data is sent):
    python flash_trajectory.py backflip_v4.hdf5 --upload --host 192.168.1.100

    # Load first 28 frames into the teach buffer for web-UI playback:
    python flash_trajectory.py backflip_v4.hdf5 --load --host 192.168.1.100

    # Dry-run: print what would be sent without actually sending:
    python flash_trajectory.py backflip_v4.hdf5 --upload --dry-run

Options:
    --host HOST        ESP32 IP address (default: 192.168.4.1 for AP mode)
    --port PORT        HTTP port (default: 80)
    --generate         Generate C header file for firmware compilation
    --upload           Stream frames in real time via HTTP
    --load             Load frames into teach/record buffer
    --scs              Use SCS (0-1023) values instead of degrees
    --speed FACTOR     Playback speed multiplier (default: 1.0)
                       >1 = faster playback, <1 = slower (time-stretched)
    --frames N         Only process first N frames
    --dry-run          Print what would be done without sending
    -o, --output DIR   Output directory for generated files (default: ../main/)
"""

import argparse
import os
import sys
import time
import urllib.request
import urllib.error
import numpy as np

# Add parent dir to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hdf5_tools import (
    load_hdf5,
    extract_joint_angles_deg,
    get_timesteps,
    get_standing_reference,
    opt_to_bf_stand_deg,
    opt_to_servo_deg,
    angles_to_scs,
    generate_header,
    SERVO_NAMES,
    BF_STAND,
    BF_SIGN,
)


class ESP32Client:
    """Lightweight HTTP client for the Mini Pupper ESP32 web interface."""

    def __init__(self, host: str = "192.168.4.1", port: int = 80,
                 dry_run: bool = False, timeout: float = 3.0):
        self.base = f"http://{host}:{port}"
        self.dry_run = dry_run
        self.timeout = timeout

    def _get(self, path: str) -> str:
        url = f"{self.base}{path}"
        if self.dry_run:
            print(f"  [DRY-RUN] GET {url}")
            return ""
        try:
            with urllib.request.urlopen(url, timeout=self.timeout) as resp:
                return resp.read().decode("utf-8", errors="replace")
        except urllib.error.URLError as e:
            print(f"  ERROR: Cannot reach ESP32 at {self.base} — {e}")
            raise
        except Exception as e:
            print(f"  ERROR: {e}")
            raise

    def send_leg_angle(self, servo_id: int, deg: float):
        """Move a single servo to an absolute angle (0–270 deg)."""
        path = f"/leg?id={servo_id}&deg={deg:.1f}"
        self._get(path)

    def send_all_legs(self, angles_deg, cur_ma: int = 0):
        """Send all 12 servos to their target angles (URDF degrees).

        Parameters
        ----------
        angles_deg : array-like of 12 floats
        cur_ma : current limit in mA (0 = use default)
        """
        for sid in range(1, 13):
            deg = float(angles_deg[sid - 1])
            # Clamp to valid servo range
            deg = max(0.0, min(270.0, deg))
            self.send_leg_angle(sid, deg)

    def send_cli(self, command: str):
        """Send a CLI command to the ESP32."""
        import urllib.parse
        encoded = urllib.parse.quote(command, safe="")
        path = f"/clicmd?cmd={encoded}"
        self._get(path)

    def enter_cli_mode(self):
        """Put the ESP32 into CLI mode (stops gait loop)."""
        self._get("/climode")

    def exit_cli_mode(self):
        """Exit CLI mode (resume gait loop)."""
        self._get("/clix")

    def set_relax(self, enable: bool = True):
        """Toggle teach/relax mode."""
        self._get("/relax")

    def send_rec(self):
        """Request a snapshot record."""
        self._get("/rec")

    def send_play(self):
        """Trigger playback of recorded frames."""
        self._get("/play")

    def reset_modes(self):
        """Reset all motion modes to neutral stand."""
        self._get("/ini")


def cmd_generate(args, data, angles_deg, t, standing_ref):
    """Generate C header file."""
    output_dir = args.output or os.path.join(os.path.dirname(__file__), "..", "main")
    os.makedirs(output_dir, exist_ok=True)

    var_name = os.path.splitext(os.path.basename(args.hdf5))[0].upper()
    var_name = var_name.replace(" ", "_").replace("-", "_")

    header_path = os.path.join(output_dir, f"{var_name.lower()}_data.h")
    header_text = generate_header(
        angles_deg, t, var_name=var_name,
        description=f"source: {os.path.basename(args.hdf5)}",
        standing_ref=standing_ref,
    )
    with open(header_path, "w") as f:
        f.write(header_text)

    print(f"Generated: {header_path}")
    print(f"  {len(angles_deg)} frames, {t[-1]:.3f}s total")
    print()
    print("To use this trajectory in the firmware:")
    print(f"  1. Make sure {os.path.basename(header_path)} is in main/")
    print(f"  2. Add #include \"{os.path.basename(header_path)}\" to main.c")
    print(f"  3. Add a /{var_name.lower()}load HTTP handler (copy the bfload pattern)")
    print(f"  4. Rebuild and flash the firmware")


def cmd_upload(args, data, angles_deg, t, standing_ref):
    """Stream trajectory frames to the ESP32 in real time."""

    client = ESP32Client(host=args.host, port=args.port, dry_run=args.dry_run)

    # Convert to absolute servo degrees using the standing reference
    servo_deg_all = opt_to_servo_deg(angles_deg, standing_ref)

    n_frames = servo_deg_all.shape[0]
    dts = np.diff(t)
    nominal_dt = float(np.median(dts)) if len(dts) > 1 else 0.05
    speed = args.speed if args.speed > 0 else 1.0

    print(f"Uploading {n_frames} frames to {client.base}")
    print(f"Duration: {t[-1]:.3f}s  Nominal dt: {nominal_dt*1000:.1f}ms"
          f"  Speed: {speed:.1f}x")
    print()

    if not args.dry_run:
        print("Connecting to ESP32...")
        try:
            # Quick connectivity check
            client._get("/")
            print("Connected!")
        except Exception:
            print("Cannot reach ESP32. Check:")
            print("  - Is the robot powered on?")
            print("  - Is WiFi connected to the robot's network?")
            print(f"  - Is {client.base} the correct address?")
            print("\nUse --dry-run to preview without connecting.")
            sys.exit(1)

        # Stop any active motion, go to neutral stand
        print("Stopping motion, going to neutral stand...")
        client.reset_modes()
        time.sleep(0.3)

    print(f"\nStreaming {n_frames} frames...")

    frame_start = time.time()
    for f_idx in range(n_frames):
        # Calculate when this frame should be sent
        if f_idx == 0:
            target_time = frame_start
        else:
            target_time = frame_start + t[f_idx] / speed

        wait = target_time - time.time()
        if wait > 0:
            time.sleep(wait)

        degs = servo_deg_all[f_idx]
        if args.dry_run:
            if f_idx % 10 == 0 or f_idx < 5:
                short = ", ".join(
                    f"{SERVO_NAMES[i]}={degs[i]:.1f}" for i in range(12)
                )
                print(f"  Frame {f_idx:4d}  t={t[f_idx]:.3f}s  [{short}]")
        else:
            # Send all 12 servo angles
            for sid in range(1, 13):
                deg = float(degs[sid - 1])
                deg = max(0.0, min(270.0, deg))
                client.send_leg_angle(sid, deg)

        if f_idx % 20 == 0 and f_idx > 0:
            elapsed = time.time() - frame_start
            progress = 100.0 * f_idx / n_frames
            print(f"  Progress: {progress:.0f}%  ({f_idx}/{n_frames})"
                  f"  elapsed: {elapsed:.1f}s")

    elapsed = time.time() - frame_start
    print(f"\nDone! {n_frames} frames streamed in {elapsed:.1f}s"
          f"  (replay speed: {t[-1]/elapsed:.1f}x)")

    # Return to neutral stand
    if not args.dry_run:
        print("Returning to neutral stand...")
        time.sleep(0.2)
        client.reset_modes()


def cmd_load(args, data, angles_deg, t, standing_ref):
    """Load frames into the ESP32 teach/record buffer via CLI commands.

    This uses the 'recsave' CLI command pattern to load SCS frames
    into the record buffer so they can be played via the web UI.
    Limited to MAX_FRAMES=32.
    """

    # Convert to BF_STAND convention, then to SCS
    bf_angles = opt_to_bf_stand_deg(angles_deg, standing_ref)
    scs = angles_to_scs(bf_angles)
    n_frames = scs.shape[0]
    max_frames = 32  # ESP32 MAX_FRAMES

    if n_frames > max_frames:
        print(f"WARNING: Trajectory has {n_frames} frames, but ESP32"
              f" MAX_FRAMES={max_frames}.")
        print(f"Only the first {max_frames} frames will be loaded.")
        print("Use --generate or --upload for full trajectories.")
        n_frames = max_frames
        scs = scs[:max_frames]

    client = ESP32Client(host=args.host, port=args.port, dry_run=args.dry_run)

    print(f"Loading {n_frames} frames into ESP32 teach buffer at {client.base}")

    if args.dry_run:
        for f_idx in range(min(n_frames, 5)):
            scs_str = ", ".join(
                f"{SERVO_NAMES[i]}={scs[f_idx, i]}" for i in range(12)
            )
            print(f"  Frame {f_idx}: [{scs_str}]")
        if n_frames > 5:
            print(f"  ... ({n_frames - 5} more frames)")
        return

    # Approach: Use the CLI to enter teach mode, then send SCS poses
    # via the /leg endpoint (converting SCS to degrees), then record.
    print("Connecting to ESP32...")
    try:
        client._get("/")
    except Exception:
        print(f"Cannot reach ESP32 at {client.base}")
        sys.exit(1)

    print("Entering teach mode, loading frames...")
    client.reset_modes()
    client.set_relax(True)
    time.sleep(0.3)

    # Clear existing recordings
    client.send_cli("recclear")
    time.sleep(0.1)

    for f_idx in range(n_frames):
        # SCS → degrees for /leg endpoint: deg = SCS * 0.263
        for sid in range(1, 13):
            scs_val = int(scs[f_idx, sid - 1])
            deg = scs_val * 0.263
            deg = max(0.0, min(270.0, deg))
            client.send_leg_angle(sid, deg)

        time.sleep(0.05)

        # Record this frame
        client.send_rec()
        time.sleep(0.2)

        if f_idx % 10 == 0:
            print(f"  Recorded frame {f_idx + 1}/{n_frames}")

    # Exit teach mode
    client.set_relax(False)
    time.sleep(0.2)
    client.reset_modes()

    print(f"\nDone! {n_frames} frames loaded into the teach buffer.")
    print("Use the web UI Verify/Play buttons, or send:")
    print(f"  GET http://{args.host}:{args.port}/play")


def main():
    parser = argparse.ArgumentParser(
        description="Flash Mini Pupper trajectory to ESP32",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n"
               "  python flash_trajectory.py flip.hdf5 --generate\n"
               "  python flash_trajectory.py flip.hdf5 --upload --host 192.168.1.100\n"
               "  python flash_trajectory.py flip.hdf5 --upload --dry-run\n"
               "  python flash_trajectory.py flip.hdf5 --load --host 192.168.1.100",
    )
    parser.add_argument("hdf5", help="Path to trajectory .hdf5 file")

    # Action flags (at least one required)
    parser.add_argument(
        "--generate", action="store_true",
        help="Generate C header file for firmware compilation",
    )
    parser.add_argument(
        "--upload", action="store_true",
        help="Stream trajectory frames to ESP32 in real time via HTTP",
    )
    parser.add_argument(
        "--load", action="store_true",
        help="Load frames into ESP32 teach/record buffer (max 32 frames)",
    )

    # Connection options
    parser.add_argument(
        "--host", default="192.168.4.1",
        help="ESP32 IP address (default: 192.168.4.1 for AP mode)",
    )
    parser.add_argument(
        "--port", type=int, default=80,
        help="HTTP port (default: 80)",
    )

    # Playback options
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Playback speed multiplier (default: 1.0)",
    )
    parser.add_argument(
        "--frames", type=int, default=0,
        help="Only process first N frames",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print what would be done without sending commands",
    )

    # Output
    parser.add_argument(
        "-o", "--output", default=None,
        help="Output directory for generated files (default: ../main/)",
    )

    args = parser.parse_args()

    if not any([args.generate, args.upload, args.load]):
        parser.print_help()
        print("\nERROR: Specify at least one action: --generate, --upload, or --load")
        sys.exit(1)

    if not os.path.exists(args.hdf5):
        print(f"ERROR: File not found: {args.hdf5}")
        sys.exit(1)

    # Load and prepare data
    print(f"Loading {args.hdf5} ...")
    data = load_hdf5(args.hdf5)
    angles = extract_joint_angles_deg(data)
    t, dt = get_timesteps(data)

    # Auto-detect standing reference from first frame
    standing_ref = get_standing_reference(data)
    print(f"Standing reference (auto-detected from first frame):")
    for col in range(12):
        if "ABD" not in SERVO_NAMES[col]:
            print(f"  {SERVO_NAMES[col]:>8s}: {standing_ref[col]:+.2f} deg"
                  f"  -> BF_STAND={BF_STAND[col]:+.0f} deg"
                  f"  (offset={BF_STAND[col]-standing_ref[col]:+.2f})")
    print()

    if args.frames > 0:
        n = min(args.frames, len(angles))
        angles = angles[:n]
        t = t[:n]

    print(f"  {len(angles)} frames, {t[-1]:.3f}s, dt={dt*1000:.1f}ms ({1/dt:.0f}Hz)")
    print()

    if args.generate:
        cmd_generate(args, data, angles, t, standing_ref)
        print()

    if args.upload:
        cmd_upload(args, data, angles, t, standing_ref)
        print()

    if args.load:
        cmd_load(args, data, angles, t, standing_ref)
        print()


if __name__ == "__main__":
    main()
