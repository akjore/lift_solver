import math
from typing import Self

import numpy as np


class AttachmentPoint:
    def __init__(self: Self, id, parent, position_local, axis_local=None, radius=None):
        self.id = id
        self.parent = parent
        self.position_local = position_local
        self.axis_local = None if axis_local is None else np.asarray(axis_local)
        self.radius = radius

    def global_position(self: Self):
        """Return global position of AttachmentPoint."""
        if self.parent is None:
            return self.position_local

        R = self.parent.global_rotation()
        t = self.parent.global_position()

        return t + R @ self.position_local

    def global_axis(self: Self):
        """Return global axis of AttachmentPoint."""
        R = self.parent.global_rotation()
        axis = R @ self.axis_local
        return axis / np.linalg.norm(axis)
