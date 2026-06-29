"""Module for handling graphics or body representations."""
from typing import Self

import numpy as np


class Mesh:
    """Class for stl files."""

    def __init__(
            self: Self,
            file: str,
            scale: float = 1.0,
            rotation: np.array(3) = None,
            translation: np.array(3) = None
        ) -> None:
        """Initialize Mesh object."""
        self.file = file
        self.scale = scale
        self.rotation = rotation if rotation is not None else np.eye(3)
        self.translation = translation if translation is not None else np.zeros(3)
