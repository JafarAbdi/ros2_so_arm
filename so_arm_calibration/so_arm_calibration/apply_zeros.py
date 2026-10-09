"""Fold joint zero offsets (`calibrate_zeros`'s joint_zeros.yaml) into an SO-ARM description.

ros2 run so_arm_calibration apply_zeros --urdf so_arm101.urdf --zeros joint_zeros.yaml --out calibrated.urdf
"""

from __future__ import annotations

import argparse
import pathlib

import yaml

from so_arm_calibration.urdf_chain import apply_zeros


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--urdf", type=pathlib.Path, required=True)
    parser.add_argument("--zeros", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args(argv)
    zeros = yaml.safe_load(args.zeros.read_text())["joint_zeros_rad"]
    args.out.write_text(
        apply_zeros(args.urdf.read_text(), {j: float(v) for j, v in zeros.items()})
    )
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
