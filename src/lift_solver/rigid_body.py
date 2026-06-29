import numpy as np
from .rigid_body_base import RigidBodyBase
from .attachment_point import AttachmentPoint
from . import ureg

class RigidBody(RigidBodyBase):
    def __init__(self, id: str):
        super().__init__(id=id)
        self.offset = np.array([0, 0, 0]) * ureg.meter
        self.size = np.array([0, 0, 0]) * ureg.meter

    def from_dict(self, kwargs):
        self.mass = kwargs.get("mass")
        self.cog = kwargs.get("cog")

        if "pose" in kwargs:
            position = kwargs["pose"].get("position")
            orientation = kwargs["pose"].get("orientation")
            self.set_pose(position, orientation)

        self.visual = kwargs.get("visual")

        # Attachment points
        for point in kwargs["points"]:
            attachment_point = AttachmentPoint(
                id = point["id"],
                parent = self,
                position_local = point["position"],
                axis_local = point.get("axis"),
#               radius = radius,
            )

            self.attachment_points[attachment_point.id] = attachment_point

        # Create graphics


