import math
from typing import Self

import numpy as np


class AttachmentPoint:
    _body_number = None

#    def __init__(self: Self, id, parent, position_local, axis_local=None, radius=None):
    def __init__(self: Self, id, parent, position_local, axis_local=None, radius=None):
        self.id = id
        self.parent = parent
#        self.position_local = np.asarray(position_local)
#        self.position_local = position_local
        self.position_local = position_local
        self.axis_local = None if axis_local is None else np.asarray(axis_local)
        self.radius = radius

    def global_position(self: Self):
#        T = self.parent.global_transform()
#        p = np.append(self.position_local, 1)
#        return (T @ p)[:3]

        if self.parent is None:
            return self.position_local

        R = self.parent.global_rotation()
        t = self.parent.global_position()

        return t + R @ self.position_local

    def global_axis(self: Self):
#        R = self.parent.global_transform()[:3, :3]
#        return R @ self.axis_local

        R = self.parent.global_rotation()
        axis = R @ self.axis_local
        return axis / np.linalg.norm(axis)
