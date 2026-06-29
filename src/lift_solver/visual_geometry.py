import numpy as np

class Mesh:
    def __init__(self, file, scale=1.0, rotation=None, translation=None):
        self.file = file
        self.scale = scale
        self.rotation = rotation if rotation is not None else np.eye(3)
        self.translation = translation if translation is not None else np.zeros(3)
