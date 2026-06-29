"""AttachmentPoint."""
from typing import Self

import numpy as np

from pint import Quantity


class AttachmentPoint:
    """Represents a single attachment point on a rigid body."""

    def __init__(
            self: Self,
            id: str,
            parent: str,
            position_local: list,
            axis_local: list | None = None,
            radius: Quantity | None = None
        ) -> None:
        """Create a new Attachment Point. Minimum input is an id, a parent, and a position."""
        self.id = id
        self.parent = parent
        self.position_local = position_local
        self.axis_local = None if axis_local is None else np.asarray(axis_local)
        self.radius = radius

    def global_position(self: Self) -> np.array(3):
        """Return global position of AttachmentPoint."""
        if self.parent is None:
            return self.position_local

        R = self.parent.global_rotation()
        t = self.parent.global_position()

        return t + R @ self.position_local

    def global_axis(self: Self) -> np.array(3):
        """Return global axis of AttachmentPoint."""
        R = self.parent.global_rotation()
        axis = R @ self.axis_local
        return axis / np.linalg.norm(axis)
