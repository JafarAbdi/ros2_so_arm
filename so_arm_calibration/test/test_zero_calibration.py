"""Joint zeros from touches on table dots, on the SO-ARM101's own description, with a simulated hand."""

import math
import pathlib

import numpy as np
import pytest
from scipy.optimize import least_squares

from so_arm_calibration import fit as zero_fit
from so_arm_calibration.urdf_chain import (
    Chain,
    apply_zeros,
    matrix_to_rpy,
    rpy_to_matrix,
)

ARM = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)
BASE, TIP = "base_link", "gripper_frame_link"
DOTS = np.array([(0.18, -0.10, 0.0), (0.28, 0.05, 0.0), (0.20, 0.15, 0.0)])
SHAPES = ("down", "towards", "away", "left", "right")
TILT = math.radians(40)


@pytest.fixture(scope="module")
def urdf() -> str:
    xacro = pytest.importorskip("xacro")
    try:
        from ament_index_python.packages import get_package_share_directory

        share = pathlib.Path(get_package_share_directory("so_arm101_description"))
    except Exception:  # noqa: BLE001 - run from a source checkout instead
        share = pathlib.Path(__file__).resolve().parents[2] / "so_arm101_description"
        pytest.importorskip("ament_index_python")
    document = xacro.process_file(str(share / "urdf" / "so_arm101.urdf.xacro"))
    return document.toxml()


def approach(point: np.ndarray, shape: str, tilt: float) -> np.ndarray:
    radial = np.array([point[0], point[1], 0.0]) / math.hypot(point[0], point[1])
    across = np.array([-radial[1], radial[0], 0.0])
    lean = {
        "down": np.zeros(3),
        "towards": -radial,
        "away": radial,
        "left": across,
        "right": -across,
    }[shape]
    direction = np.array([0.0, 0.0, -math.cos(tilt)]) + math.sin(tilt) * lean
    return direction / np.linalg.norm(direction)


def hand(
    chain: Chain,
    true_zeros: dict,
    tip: np.ndarray,
    rng,
    tilt=TILT,
    noise=0.0005,
):
    """Touches a person makes: the true arm put with its tips (`tip` in the tip frame) on each dot,
    missed by `noise`, the tool along each shape; the joints read the true angles less the zeros."""
    touches = []
    for d, dot in enumerate(DOTS):
        for shape in SHAPES:
            target = dot + np.array([*rng.normal(0.0, noise, 2), 0.0])
            pointing = approach(dot, shape, tilt)

            def residual(theta, target=target, pointing=pointing):
                pose = chain.forward(dict(zip(ARM, theta, strict=True)))
                return np.concatenate(
                    [
                        (pose[:3, :3] @ tip + pose[:3, 3] - target) / 0.001,
                        pose[:3, 2] - pointing,
                    ]
                )

            pan = math.atan2(dot[1], dot[0])
            solution = least_squares(residual, [pan, 0.3, 0.3, 1.0, 0.0])
            if np.linalg.norm(solution.fun[:3]) > 0.1:  # out of reach in this shape
                continue
            touches.append(
                zero_fit.Touch(
                    d,
                    {
                        j: float(t - true_zeros.get(j, 0.0))
                        for j, t in zip(ARM, solution.x, strict=True)
                    },
                )
            )
    return touches


def test_rpy_round_trip():
    for rpy in [(0.1, -0.4, 2.0), (-1.5708, -1.5708, 0.0), (3.0, 0.2, -1.0)]:
        np.testing.assert_allclose(
            rpy_to_matrix(*matrix_to_rpy(rpy_to_matrix(*rpy))),
            rpy_to_matrix(*rpy),
            atol=1e-9,
        )


def test_zeros_folded_into_the_description_move_the_arm_as_the_offsets_do(urdf):
    zeros = {
        "shoulder_lift_joint": 0.03,
        "elbow_flex_joint": -0.05,
        "wrist_flex_joint": 0.02,
    }
    plain, corrected = (
        Chain(urdf, BASE, TIP),
        Chain(apply_zeros(urdf, zeros), BASE, TIP),
    )
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = dict(zip(ARM, rng.uniform(-1.2, 1.2, 5), strict=True))
        np.testing.assert_allclose(
            corrected.forward(q), plain.forward(q, zeros), atol=1e-9
        )


def test_limits_keep_the_same_physical_travel(urdf):
    import xml.etree.ElementTree as ET

    def limits(text):
        joint = next(
            j
            for j in ET.fromstring(text).findall("joint")
            if j.get("name") == "elbow_flex_joint"
        )
        return float(joint.find("limit").get("lower")), float(
            joint.find("limit").get("upper")
        )

    lower, upper = limits(urdf)
    assert limits(apply_zeros(urdf, {"elbow_flex_joint": 0.05})) == pytest.approx(
        (lower - 0.05, upper - 0.05)
    )


def test_touches_on_table_dots_recover_the_zeros(urdf):
    chain = Chain(urdf, BASE, TIP)
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(4):
        true_zeros = dict(
            zip(
                zero_fit.FREE_JOINTS,
                rng.uniform(-math.radians(3), math.radians(3), 3),
                strict=True,
            )
        )
        tip = rng.uniform(-0.002, 0.002, 3)
        touches = hand(chain, true_zeros, tip, rng)
        assert len(touches) >= 12
        result = zero_fit.fit(chain, touches)
        for joint, value in true_zeros.items():
            worst = max(worst, abs(math.degrees(result.zeros[joint] - value)))
        assert result.rms_m < 0.001
        np.testing.assert_allclose(result.dots[:, 2], result.dots[0, 2])
    assert worst < 0.3


def test_a_dot_touched_once_is_refused(urdf):
    chain = Chain(urdf, BASE, TIP)
    touches = [
        zero_fit.Touch(0, dict.fromkeys(ARM, 0.0)),
        zero_fit.Touch(1, dict.fromkeys(ARM, 0.1)),
        zero_fit.Touch(1, dict.fromkeys(ARM, 0.2)),
    ]
    with pytest.raises(ValueError, match="dot 0 has one touch"):
        zero_fit.fit(chain, touches)


def test_refit_writes_the_offsets_and_a_corrected_description(urdf, tmp_path):
    import json

    import yaml

    from so_arm_calibration import calibrate_zeros

    chain = Chain(urdf, BASE, TIP)
    true_zeros = {
        "shoulder_lift_joint": 0.03,
        "elbow_flex_joint": -0.04,
        "wrist_flex_joint": 0.05,
    }
    touches = hand(chain, true_zeros, np.zeros(3), np.random.default_rng(2))
    (tmp_path / "touches.json").write_text(
        json.dumps(
            {
                "homing_offsets": dict.fromkeys(ARM, 0),
                "touches": [
                    {"dot": t.dot, "shape": "down", "positions": t.positions}
                    for t in touches
                ],
            }
        )
    )
    (tmp_path / "arm.urdf").write_text(urdf)

    calibrate_zeros.main(
        [
            "--refit",
            str(tmp_path / "touches.json"),
            "--urdf",
            str(tmp_path / "arm.urdf"),
            "--out",
            str(tmp_path),
        ]
    )

    written = yaml.safe_load((tmp_path / "joint_zeros.yaml").read_text())[
        "joint_zeros_rad"
    ]
    for joint, value in true_zeros.items():
        assert written[joint] == pytest.approx(value, abs=math.radians(0.3))
    corrected = Chain((tmp_path / "so_arm101_calibrated.urdf").read_text(), BASE, TIP)
    q = touches[0].positions
    np.testing.assert_allclose(
        corrected.forward(q), chain.forward(q, written), atol=1e-5
    )


class _Port:
    """A servo bus that answers every read with `value`, and records what was written."""

    def __init__(self, value: int, size: int):
        self.value, self.size, self.sent = value, size, []

    def reset_input_buffer(self):
        pass

    def write(self, packet):
        self.sent.append(packet)

    def read(self, n):
        servo = self.sent[-1][2]
        params = self.value.to_bytes(self.size, "little") if n > 6 else b""
        body = bytes([servo, len(params) + 2, 0]) + params
        return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def test_bus_reads_sign_magnitude_positions_and_offsets():
    pytest.importorskip("serial")
    from so_arm_calibration.feetech import Bus, ticks_to_rad

    bus = Bus.__new__(Bus)
    bus.serial = _Port(2048 + 512, 2)
    assert ticks_to_rad(bus.position_ticks(3)) == pytest.approx(math.pi / 4)
    assert bus.serial.sent[-1] == bytes(
        [0xFF, 0xFF, 3, 4, 2, 56, 2, (~(3 + 4 + 2 + 56 + 2)) & 0xFF]
    )
    bus.serial = _Port((1 << 11) | 100, 2)
    assert bus.homing_offset(1) == -100
