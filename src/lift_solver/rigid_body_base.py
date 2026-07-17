"""Top-level module for bodies."""
from typing import Self

import numpy as np

from . import ureg
from .visual_geometry import Visual


class RigidBodyBase:
    """Base class for bodies."""

    def __init__(self: Self, id: str = None) -> None:
        """Initialize class."""
        self.id = id

        # Local transform relative to parent
        self.position = np.zeros(3) * ureg.meters
        self.rotation = np.eye(3) #* ureg.radians

        # Hierarchy
        self.parent = None
        self.children = []
        self.attachment_points = {}

    # -------------------------------
    # Hierarchy management
    # -------------------------------
    def add_child(self: Self, child: "RigidBodyBase") -> None:
        """Add children to body."""
        if child.parent is not None:
            raise RuntimeError("Child already has a parent")

        self.children.append(child)
        child.parent = self

    # -------------------------------
    # Transform getters
    # -------------------------------
    def global_rotation(self: Self) -> list:
        """Return global orientation of body."""
        if self.parent is None:
            return self.rotation
        return self.parent.global_rotation() @ self.rotation

    def global_position(self: Self) -> list:
        """Return global position of body."""
        if self.parent is None:
            return self.position
        return self.parent.global_position() + self.parent.global_rotation() @ self.position


    # -------------------------------
    # Pose
    # -------------------------------
    def set_pose(self: Self, position: np.array(3), orientation: np.array(3)) -> None:
        """Set body pose (position and orientation)."""
        self.position = position
        self.rotation = self._euler_to_matrix(orientation)


    def translate(self: Self, vec: np.array(3)) -> None:
        """Set body position."""
        self.position += vec


    def rotate(self: Self, R_new: np.array(3)) -> None:
        """Set body orientation."""
        self.rotation = R_new @ self.rotation


    def set_global_pose(self, position, rotation):
        """Set pose in global coordinates."""
        if self.parent is None:
            self.position = position
            self.rotation = rotation
        else:
            R_parent = self.parent.global_rotation()
            p_parent = self.parent.global_position()

            self.position = R_parent.T @ (position - p_parent)
            self.rotation = R_parent.T @ rotation


    def set_local_pose(self, position, rotation):
        """
        Set pose relative to parent.
        """
        self.position = position
        self.rotation = rotation


    def relative_to(self, parent):
        """Return transform of self relative to given parent."""

        R_self = self.global_rotation()
        p_self = self.global_position()

        R_parent = parent.global_rotation()
        p_parent = parent.global_position()

        R_rel = R_parent.T @ R_self
        p_rel = R_parent.T @ (p_self - p_parent)

        return p_rel, R_rel


    # -------------------------------
    # Utilities
    # -------------------------------
    def _euler_to_matrix(self: Self, euler: list) -> list:
        # Euler convention: ZYX (Rz @ Ry @ Rx)
        rx, ry, rz = euler.to("radians")

        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)

        Rx = np.array([
            [1, 0, 0],
            [0, cx, -sx],
            [0, sx, cx]
        ])

        Ry = np.array([
            [cy, 0, sy],
            [0, 1, 0],
            [-sy, 0, cy]
       ])

        Rz = np.array([
            [cz, -sz, 0],
            [sz, cz, 0],
            [0, 0, 1]
        ])

        return Rz @ Ry @ Rx


    def _matrix_to_euler(self, R):
        """
        Inverse of _euler_to_matrix().

        Returns:
            [rx, ry, rz] as pint angles.
        """

        ry = np.arcsin(-R[2, 0])

        cy = np.cos(ry)

        if abs(cy) > 1e-8:
            rx = np.arctan2(
                R[2, 1],
                R[2, 2]
            )

            rz = np.arctan2(
                R[1, 0],
                R[0, 0]
            )

        else:
            # Gimbal lock

            rx = 0.0

            rz = np.arctan2(
                -R[0, 1],
                 R[1, 1]
            )

        return (np.array([
            rx,
            ry,
            rz
        ]) * ureg.radian).to("deg")


    # -------------------------------
    # Export
    # -------------------------------
    def to_dict(self):
        """Create a dict representation of the class."""
        return {
            "id": self.id,
            "type": self.__class__.__name__,
            "position": self.global_position(),
            "rotation_matrix": self.global_rotation(),
            "rotation_euler": self._matrix_to_euler(self.global_rotation()),
            "cog": self.cog,
            "mass": self.mass.to("t"),
            "children": [c.id for c in self.children],
            "attachment_points": [a.to_dict() for a in self.attachment_points.values()],
            "visual": self.visual.to_dict() if isinstance(self.visual, Visual) else self.visual,
        }
