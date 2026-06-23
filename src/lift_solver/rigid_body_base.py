import numpy as np


class RigidBodyBase:
    def __init__(self, id: str = None):
        self.id = id

        # Local transform relative to parent
        self.position = np.zeros(3)
        self.rotation = np.eye(3)

        # Hierarchy
        self.parent = None
        self.children = []

    # -------------------------------
    # Hierarchy management
    # -------------------------------
    def add_child(self, child: "RigidBodyBase"):
        self.children.append(child)
        child.parent = self

    # -------------------------------
    # Transform getters
    # -------------------------------
    def global_rotation(self):
        if self.parent is None:
            return self.rotation
        return self.parent.global_rotation() @ self.rotation

    def global_position(self):
        if self.parent is None:
            return self.position
        return self.parent.global_position() + self.parent.global_rotation() @ self.position

    # -------------------------------
    # Set pose
    # -------------------------------
    def set_pose(self, position, orientation):
        self.position = position
        self.rotation = self._euler_to_matrix(orientation)

    def translate(self, vec):
        self.position += np.array(vec)

    def rotate(self, R_new):
        self.rotation = R_new @ self.rotation


    def _euler_to_matrix(self, euler):
        rx, ry, rz = euler

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx),  np.cos(rx)],
        ])

        Ry = np.array([
            [ np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)],
        ])

        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz),  np.cos(rz), 0],
            [0, 0, 1],
        ])

        return Rz @ Ry @ Rx
