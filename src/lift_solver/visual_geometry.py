"""Module for handling graphics or body representations."""
from typing import Self

import numpy as np


class Visual:
    """Class for visual representation of rigid bodies."""

    def to_dict(self) -> dict:
        """Create a dict representation of self. Overridden by children."""
        raise NotImplementedError


class BoxVisual(Visual):
    """Class for box primitives."""

    pass


class CylinderVisual(Visual):
    """Class for cylinder primitives."""

    pass


class MeshVisual(Visual):
    """Class for stl files."""

    def __init__(
            self: Self,
            folder: str,
            file: str,
            scale: float = 1.0,
            rotation: np.array(3) = None,
            translation: np.array(3) = None
        ) -> None:
        """Initialize Mesh object."""
        self.type = "stl"
        self.folder = folder
        self.file = file
        self.scale = scale
        self.rotation = rotation if rotation is not None else np.eye(3)
        self.translation = translation if translation is not None else np.zeros(3)

    def to_dict(self: Self) -> dict:
        """Create a dict representation of self for export."""
        return {
            "type": self.type,
            "file": self.file,
            "scale": self.scale,
            "rotation": self.rotation,
            "translation": self.translation,
        }
