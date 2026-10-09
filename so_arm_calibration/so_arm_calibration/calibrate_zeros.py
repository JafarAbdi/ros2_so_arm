"""Find an SO-ARM101's joint zero offsets against its description, by touching dots on the table.

LeRobot's calibration sets each servo's zero where a person judges the middle of its travel to be,
so a joint reads a degree or a few off the angle the description means, and the arm's tip lands
millimetres from where its description puts it. This finds those offsets (`fit`) and writes them
as a YAML file and as a corrected description.

With nothing else on the servo bus (stop any driver first) and the arm clamped to a flat table:

1. Mark three dots on the table with a pen, within the arm's reach: one near the base, one far
   out in front, one to the side.
2. Close the jaws, and with the torque off hold the arm so the closed tips rest in a dot. Do each
   dot in five shapes: the tool straight down, then leaning about 40 degrees towards the base, away
   from it, and to either side. Hold still and press Enter for each.
3. The fit prints each zero with its uncertainty and how far the tips were from their dots, and
   writes `joint_zeros.yaml` and the corrected URDF.

    ros2 run so_arm_calibration calibrate_zeros --port /dev/ttyACM0 --out ~/so101_zeros
    ros2 run so_arm_calibration calibrate_zeros --refit ~/so101_zeros/touches.json   # no arm needed

The offsets hold for the homing offsets they were measured with, which the YAML records: after a
new LeRobot calibration, run this again.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np
import yaml

from so_arm_calibration import fit as zero_fit
from so_arm_calibration.urdf_chain import Chain, apply_zeros

JOINTS = {
    "shoulder_pan_joint": 1,
    "shoulder_lift_joint": 2,
    "elbow_flex_joint": 3,
    "wrist_flex_joint": 4,
    "wrist_roll_joint": 5,
}
SHAPES = (
    ("down", "the tool straight down"),
    ("towards", "the tool leaning about 40 degrees towards the arm's base"),
    ("away", "the tool leaning about 40 degrees away from the base"),
    ("left", "the tool leaning about 40 degrees to the left"),
    ("right", "the tool leaning about 40 degrees to the right"),
)
# A touch further than this from the fit's dot, metres, is named for redoing.
SUSPECT_M = 0.002


def default_urdf() -> str:
    import xacro
    from ament_index_python.packages import get_package_share_directory

    share = pathlib.Path(get_package_share_directory("so_arm101_description"))
    return xacro.process_file(str(share / "urdf" / "so_arm101.urdf.xacro")).toxml()


def lean(chain: Chain, positions: dict[str, float]) -> str:
    """How the tool leans, by the description: degrees from straight down, and which way."""
    pose = chain.forward(positions)
    pointing, tip = pose[:3, 2], pose[:3, 3]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -pointing[2]))))
    if tilt < 10:
        return f"{tilt:.0f} deg from straight down"
    radial = np.array([tip[0], tip[1]]) / max(math.hypot(tip[0], tip[1]), 1e-9)
    horizontal = pointing[:2] / max(np.linalg.norm(pointing[:2]), 1e-9)
    along, across = (
        float(horizontal @ radial),
        float(horizontal[1] * radial[0] - horizontal[0] * radial[1]),
    )
    way = (
        "away from the base"
        if along > 0.7
        else "towards the base"
        if along < -0.7
        else "to the left"
        if across > 0
        else "to the right"
    )
    return f"{tilt:.0f} deg {way}"


def record(
    args, chain: Chain, touches_path: pathlib.Path
) -> tuple[list[zero_fit.Touch], dict]:
    from so_arm_calibration.feetech import Bus

    bus = Bus(args.port)
    try:
        homing = {name: bus.homing_offset(servo) for name, servo in JOINTS.items()}
        held = [name for name, servo in JOINTS.items() if bus.torque_on(servo)]
        if held:
            if not args.release:
                sys.exit(
                    f"torque is on ({', '.join(held)}): the arm cannot be moved by hand. Rerun with "
                    "--release to turn it off."
                )
            input(
                "Hold the arm: its torque goes off and it will fall. Press Enter when holding it. "
            )
            for name in held:
                bus.release(JOINTS[name])
        touches, saved = [], []
        print("\n\n".join(__doc__.split("\n\n")[2:4]))
        for dot in range(args.dots):
            shape = 0
            while shape < len(SHAPES):
                key, words = SHAPES[shape]
                answer = (
                    input(
                        f"\nDot {dot + 1} of {args.dots}, {words}. Enter to record, s to skip, "
                        "q to finish: "
                    )
                    .strip()
                    .lower()
                )
                if answer == "q":
                    return touches, homing
                if answer == "s":
                    shape += 1
                    continue
                positions = bus.still_positions(JOINTS)
                if positions is None:
                    print(
                        "  the arm moved while recording: hold it still and press Enter again"
                    )
                    continue
                touches.append(zero_fit.Touch(dot, positions))
                saved.append({"dot": dot, "shape": key, "positions": positions})
                touches_path.write_text(
                    json.dumps({"homing_offsets": homing, "touches": saved}, indent=1)
                )
                print(f"  recorded: {lean(chain, positions)}")
                shape += 1
        return touches, homing
    finally:
        bus.close()


def report(result: zero_fit.Result, touches: list[zero_fit.Touch], shapes: list[str]):
    print(
        f"\n{len(touches)} touches on {len(result.dots)} dots; the tips are {result.rms_m * 1000:.2f} mm rms "
        "from their dots after the fit."
    )
    for joint, value in result.zeros.items():
        print(
            f"  {joint:22s} {math.degrees(value):+6.2f} deg  (+- {math.degrees(result.sigma[joint]):.2f})"
        )
    print(
        f"  closed tips off the tip frame: {np.round(result.tip_offset * 1000, 1)} mm"
    )
    for touch, shape, distance in zip(touches, shapes, result.residuals_m, strict=True):
        if distance > SUSPECT_M:
            print(
                f"  dot {touch.dot + 1}, {shape}: {distance * 1000:.1f} mm off its dot; redo it if the "
                "tips were not in the dot"
            )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", help="the arm's servo bus, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--out", type=pathlib.Path, default=pathlib.Path.home() / "so101_zeros"
    )
    parser.add_argument(
        "--urdf",
        type=pathlib.Path,
        help="the description to correct (default: so_arm101_description)",
    )
    parser.add_argument("--base", default="base_link")
    parser.add_argument("--tip", default="gripper_frame_link")
    parser.add_argument("--dots", type=int, default=3)
    parser.add_argument(
        "--release",
        action="store_true",
        help="turn the servos' torque off to move the arm by hand",
    )
    parser.add_argument(
        "--refit",
        type=pathlib.Path,
        help="fit touches recorded before (touches.json), no arm needed",
    )
    args = parser.parse_args(argv)

    urdf = args.urdf.read_text() if args.urdf else default_urdf()
    chain = Chain(urdf, args.base, args.tip)
    args.out.mkdir(parents=True, exist_ok=True)
    if args.refit:
        data = json.loads(args.refit.read_text())
        homing = data["homing_offsets"]
        touches = [zero_fit.Touch(t["dot"], t["positions"]) for t in data["touches"]]
        shapes = [t["shape"] for t in data["touches"]]
    else:
        if not args.port:
            parser.error(
                "--port is needed to record touches (or --refit a touches.json)"
            )
        touches, homing = record(args, chain, args.out / "touches.json")
        shapes = (
            [
                t["shape"]
                for t in json.loads((args.out / "touches.json").read_text())["touches"]
            ]
            if touches
            else []
        )
    result = zero_fit.fit(chain, touches)
    report(result, touches, shapes)

    (args.out / "joint_zeros.yaml").write_text(
        yaml.safe_dump(
            {
                "joint_zeros_rad": {j: round(v, 6) for j, v in result.zeros.items()},
                "joint_zeros_sigma_rad": {
                    j: round(v, 6) for j, v in result.sigma.items()
                },
                "touch_rms_m": round(result.rms_m, 6),
                "touches": len(touches),
                "tip_link": args.tip,
                "tip_offset_m": [round(float(v), 6) for v in result.tip_offset],
                "homing_offsets_ticks": homing,
                "measured": time.strftime("%Y-%m-%d %H:%M:%S"),
            },
            sort_keys=False,
        )
    )
    (args.out / "so_arm101_calibrated.urdf").write_text(apply_zeros(urdf, result.zeros))
    print(
        f"\nwrote {args.out / 'joint_zeros.yaml'} and {args.out / 'so_arm101_calibrated.urdf'}"
    )


if __name__ == "__main__":
    main()
