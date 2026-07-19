"""AttachmentPoint."""
import logging
from typing import Self

import numpy as np

logger = logging.getLogger(__name__)


REQUIRES_AXIS = {
    "pin",
    "padeye",
}

class AttachmentPoint:
    """Represents a single attachment point on a rigid body."""

    def __init__(
            self: Self,
            id: str,
            parent: str,
            position_local: list,
            type: str | None = None,
            axis_local: list | None = None,
            **kwargs: dict,
        ) -> None:
        """Create a new Attachment Point. Minimum input is an id, a parent, and a position."""
        self.id = id
        self.parent = parent
        self.position_local = position_local

        self.axis_local = axis_local
        self.type = type

        # padeye properties
        self.hole_diameter = None
        self.outer_diameter = None
        self.thickness = None

        # pin properties
        self.diameter = None
        self.length = None

        for key, value in kwargs.items():
            setattr(self, key, value)

        if self.axis_local is not None:
            self.axis_local = np.asarray(self.axis_local)

        # Input validation
        if self.type in REQUIRES_AXIS and self.axis_local is None:
            raise ValueError(
                f"Attachment point '{self.id}' of type '{self.type}' requires an axis."
            )


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

        if self.axis_local is None:
            return None

        axis = R @ self.axis_local
        return axis / np.linalg.norm(axis)


    def to_dict(self: Self) -> dict:
        """Return a dict representation of the class."""
        data = {
            "id": self.id,
            "parent": self.parent.id,
            "position_local": self.position_local,
            "axis_local": self.axis_local,
            "axis_global": self.global_axis(),
            "position_global": self.global_position(),
        }

        for field in (
            "type",
            "hole_diameter",
            "outer_diameter",
            "thickness",
            "diameter",
            "length",
        ):
            value = getattr(self, field, None)

            if value is not None:
                data[field] = value

        return data
