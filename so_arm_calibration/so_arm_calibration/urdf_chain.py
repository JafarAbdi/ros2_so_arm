"""A serial chain read from a URDF: forward kinematics, and joint zero offsets folded into it.

A joint zero offset ``delta`` says the joint's true angle is its reading plus ``delta``. Folding it
into the description turns the joint's origin by ``delta`` about the joint's axis, so the joint's
transform for a reading ``q`` is the true one, ``origin * rot(axis, q + delta)``, and shifts its
limits by ``-delta``, so they keep bounding the same physical travel.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF's fixed-axis roll, pitch, yaw: ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def matrix_to_rpy(r: np.ndarray) -> tuple[float, float, float]:
    """The inverse of `rpy_to_matrix`."""
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) < 1e-9:
        # Gimbal lock: only roll - yaw (or roll + yaw) is defined; put it all in roll.
        return math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0
    return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation by `angle` about the unit `axis` (Rodrigues)."""
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


@dataclass
class Joint:
    name: str
    kind: str  # "revolute", "continuous", "prismatic" or "fixed"
    origin_rotation: np.ndarray
    origin_xyz: np.ndarray
    axis: np.ndarray

    def transform(self, q: float) -> np.ndarray:
        t = np.eye(4)
        t[:3, :3] = self.origin_rotation
        t[:3, 3] = self.origin_xyz
        if self.kind in ("revolute", "continuous"):
            m = np.eye(4)
            m[:3, :3] = axis_rotation(self.axis, q)
            t = t @ m
        elif self.kind == "prismatic":
            m = np.eye(4)
            m[:3, 3] = self.axis * q
            t = t @ m
        return t


def _origin(joint: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = joint.find("origin")
    xyz = (
        [0.0, 0.0, 0.0]
        if origin is None
        else [float(v) for v in origin.get("xyz", "0 0 0").split()]
    )
    rpy = (
        [0.0, 0.0, 0.0]
        if origin is None
        else [float(v) for v in origin.get("rpy", "0 0 0").split()]
    )
    return rpy_to_matrix(*rpy), np.array(xyz)


class Chain:
    """The joints from `base_link` to `tip_link` in the URDF `urdf` (its XML text)."""

    def __init__(self, urdf: str, base_link: str, tip_link: str):
        root = ET.fromstring(urdf)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        path, link = [], tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f"no chain of joints from {base_link} to {tip_link}")
            joint = by_child[link]
            path.append(joint)
            link = joint.find("parent").get("link")
        self.joints: list[Joint] = []
        for element in reversed(path):
            rotation, xyz = _origin(element)
            axis = element.find("axis")
            vector = np.array(
                [1.0, 0.0, 0.0]
                if axis is None
                else [float(v) for v in axis.get("xyz").split()]
            )
            norm = np.linalg.norm(vector)
            self.joints.append(
                Joint(
                    element.get("name"),
                    element.get("type"),
                    rotation,
                    xyz,
                    vector / norm if norm > 0 else vector,
                )
            )
        self.movable = [j.name for j in self.joints if j.kind != "fixed"]

    def forward(
        self, positions: dict[str, float], zeros: dict[str, float] | None = None
    ) -> np.ndarray:
        """The tip link's pose in the base link's frame (4x4) for joint `positions` as read, each
        plus its zero offset in `zeros`."""
        zeros = zeros or {}
        t = np.eye(4)
        for joint in self.joints:
            q = positions.get(joint.name, 0.0) + zeros.get(joint.name, 0.0)
            t = t @ joint.transform(q)
        return t


def apply_zeros(urdf: str, zeros: dict[str, float]) -> str:
    """The URDF text with each joint in `zeros` turned by its offset about its axis, and its limits
    shifted by the offset's negative (module docstring)."""
    root = ET.fromstring(urdf)
    found = set()
    for element in root.findall("joint"):
        name = element.get("name")
        if name not in zeros or zeros[name] == 0.0:
            continue
        if element.get("type") not in ("revolute", "continuous"):
            raise ValueError(f"{name} is not a revolute joint: a zero offset needs one")
        found.add(name)
        rotation, _xyz = _origin(element)
        axis_element = element.find("axis")
        axis = (
            np.array([float(v) for v in axis_element.get("xyz").split()])
            if axis_element is not None
            else np.array([1.0, 0.0, 0.0])
        )
        rpy = matrix_to_rpy(
            rotation @ axis_rotation(axis / np.linalg.norm(axis), zeros[name])
        )
        origin = element.find("origin")
        if origin is None:
            origin = ET.SubElement(element, "origin", xyz="0 0 0")
        origin.set("rpy", " ".join(f"{v:.12g}" for v in rpy))
        limit = element.find("limit")
        if limit is not None and element.get("type") == "revolute":
            for key in ("lower", "upper"):
                if limit.get(key) is not None:
                    limit.set(key, f"{float(limit.get(key)) - zeros[name]:.12g}")
    missing = {n for n, v in zeros.items() if v != 0.0} - found
    if missing:
        raise ValueError(
            f"no such joints in the description: {', '.join(sorted(missing))}"
        )
    return ET.tostring(root, encoding="unicode")
