"""Joint zero offsets from touches on dots on a table.

The arm's torque is off and its closed jaw tips are rested, by hand, on a few dots marked anywhere
on the table the arm stands on, each dot in several arm shapes (the tool straight down, and leaning
towards the base, away from it and to either side). Every touch of one dot puts the tips on the same
point, whatever the joints read, so the joints' zero offsets are the ones that make the description
agree with itself about where each dot is.

The dots need not be measured: their positions are fitted too. They share one height, the table's,
which is level with the arm's base. That is what fixes the shoulder lift: with each dot free in
height, lifting the whole arm about the shoulder would move every dot along with it and fit as
well. Some zeros cannot be found from touches at all and are held: turning the base pan turns every
dot about the base and fits as well, and the closed tips lie too close to the wrist roll's axis
to show its turn.

The closed tips are near the description's tip frame but not exactly on it (`tip_offset`, fitted,
held lightly to none).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from so_arm_calibration.urdf_chain import Chain

# Zeros a table's touches can find, by default: see the module docstring for those they cannot.
FREE_JOINTS = ("shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint")
# How far a hand rests the tips from the dot's centre, metres: one standard deviation.
TOUCH_SIGMA_M = 0.0005
# How far the closed tips are expected from the description's tip frame, metres.
TIP_SIGMA_M = 0.003
# How far a zero is expected to be off, radians: a weak prior, only to keep a fit bounded.
ZERO_SIGMA_RAD = math.radians(10.0)


@dataclass
class Touch:
    """The closed tips resting on dot `dot`, with the joints reading `positions` (radians)."""

    dot: int
    positions: dict[str, float]


@dataclass
class Result:
    zeros: dict[str, float]  # radians, to add to each joint's reading
    sigma: dict[str, float]  # one standard deviation of each zero, radians
    tip_offset: (
        np.ndarray
    )  # the closed tips in the tip frame, less the frame's origin, metres
    dots: np.ndarray  # (N, 3), base frame, metres
    rms_m: float  # touched tips from their dots after the fit
    residuals_m: list[float] = field(
        default_factory=list
    )  # each touch's distance from its dot


def fit(chain: Chain, touches: list[Touch], free_joints=FREE_JOINTS) -> Result:
    """The zero offsets of `free_joints` (and the dots, the table's height and the tips' offset)
    that best explain `touches`."""
    free = [j for j in free_joints if j in chain.movable]
    dots = sorted({t.dot for t in touches})
    index = {d: i for i, d in enumerate(dots)}
    for d in dots:
        if sum(t.dot == d for t in touches) < 2:
            raise ValueError(
                f"dot {d} has one touch: each dot needs several, in different arm shapes"
            )
    nz, nd = len(free), len(dots)

    def unpack(x):
        zeros = dict(zip(free, x[:nz], strict=True))
        tip = x[nz : nz + 3]
        xy = x[nz + 3 : nz + 3 + 2 * nd].reshape(nd, 2)
        points = np.column_stack([xy, np.full(nd, x[-1])])
        return zeros, tip, points

    def tip_of(touch, zeros, tip):
        pose = chain.forward(touch.positions, zeros)
        return pose[:3, :3] @ tip + pose[:3, 3]

    def residual(x):
        zeros, tip, points = unpack(x)
        out = [
            (tip_of(t, zeros, tip) - points[index[t.dot]]) / TOUCH_SIGMA_M
            for t in touches
        ]
        out.append(x[:nz] / ZERO_SIGMA_RAD)
        out.append(tip / TIP_SIGMA_M)
        return np.concatenate(out)

    first = {d: next(t for t in touches if t.dot == d) for d in dots}
    starts = np.array([tip_of(first[d], {}, np.zeros(3)) for d in dots])
    x0 = np.concatenate(
        [np.zeros(nz), np.zeros(3), starts[:, :2].ravel(), [starts[:, 2].mean()]]
    )
    solution = least_squares(residual, x0, loss="soft_l1", f_scale=3.0, x_scale="jac")
    zeros, tip, points = unpack(solution.x)
    distances = [
        float(np.linalg.norm(tip_of(t, zeros, tip) - points[index[t.dot]]))
        for t in touches
    ]
    rms = float(np.sqrt(np.mean(np.square(distances))))
    # Uncertainty from the normal matrix, the touches' noise taken as what the fit leaves.
    scale = max(rms / TOUCH_SIGMA_M, 1.0)
    try:
        cov = np.linalg.inv(solution.jac.T @ solution.jac) * scale**2
        sigma = {j: float(math.sqrt(max(cov[i, i], 0.0))) for i, j in enumerate(free)}
    except np.linalg.LinAlgError:
        sigma = {j: float("inf") for j in free}
    return Result(
        {j: float(v) for j, v in zeros.items()}, sigma, tip, points, rms, distances
    )
