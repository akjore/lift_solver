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

#        if "visual" in kwargs:
#            self.offset = kwargs["visual"].get("offset")
#            self.size = kwargs["visual"].get("size")
        self.visual = kwargs.get("visual")

        # Attachment points
        for id, props in kwargs["points"].items():
            coords = props
            axis_local = [1, 0, 0]
#            radius = 50 * ureg.millimeter

            attachment_point = AttachmentPoint(
                id = id,
                parent = self,
                position_local = coords,
                axis_local = axis_local,
#                radius = radius,
            )

#            self.add_child(attachment_point)
            self.attachment_points[id] = attachment_point

        # Create graphics


