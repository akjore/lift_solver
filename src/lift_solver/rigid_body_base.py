"""Top-level module for bodies."""
from typing import Self

import numpy as np

from . import ureg


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
    # Set pose
    # -------------------------------
    def set_pose(self: Self, position: np.array(3), orientation: np.array(3)) -> None:
        """Set body pose (position and orientation)."""
        self.position = position
        self.rotation = self._euler_to_matrix(orientation.to("radians"))

    def translate(self: Self, vec: np.array(3)) -> None:
        """Set body position."""
        self.position += vec

    def rotate(self: Self, R_new: np.array(3)) -> None:
        """Set body orientation."""
        self.rotation = R_new @ self.rotation


    def _euler_to_matrix(self: Self, euler: list) -> list:
        # Euler convention: ZYX (Rz @ Ry @ Rx)
        rx, ry, rz = euler

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx),  np.cos(rx)],
        ])

        Ry = np.array([
            [ np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)],
        ])

        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz),  np.cos(rz), 0],
            [0, 0, 1],
        ])

        return Rz @ Ry @ Rx
