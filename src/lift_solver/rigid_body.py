"""Module for handling rigid bodies."""
import numpy as np
from typing import Self

from . import ureg
from .attachment_point import AttachmentPoint
from .rigid_body_base import RigidBodyBase


class RigidBody(RigidBodyBase):
    """Class for rigid bodies."""

    def __init__(self: Self, id: str) -> None:
        """Initialize a rigid body."""
        super().__init__(id=id)
        self.offset = np.array([0, 0, 0]) * ureg.meter
        self.size = np.array([0, 0, 0]) * ureg.meter


    def from_dict(self: Self, **kwargs) -> None:
        """Set body values based on values proviced in dict."""
        self.mass = kwargs.get("mass")
        self.cog = kwargs.get("cog")

        if "pose" in kwargs:
            position = kwargs["pose"].get("position")
            orientation = kwargs["pose"].get("orientation")
            self.set_pose(position, orientation)

        self.visual = kwargs.get("visual")

        # Attachment points
        for point in kwargs["points"]:

            other_fields = {
                k: v
                for k, v in point.items()
                if k not in {"id", "position_local", "axis_local", "type"}
            }

            attachment_point = AttachmentPoint(
                id = point["id"],
                parent = self,
                position_local = point["position"],
                axis_local = point.get("axis"),
                type = point.get("type"),
                **other_fields,
            )

            self.attachment_points[attachment_point.id] = attachment_point
