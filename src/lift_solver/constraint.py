import numpy as np

from .attachment_point import AttachmentPoint


class World:
    id = "world"


class Constraint:

    def __init__(self, id, ap1, ap2, constraints):
        self.id = id
        self.ap1 = ap1
        self.ap2 = ap2  # may be "world"

        if len(constraints) != 6:
            raise ValueError("constraints must have 6 values")

        self.constraints = [int(c) for c in constraints]

    def validate(self):
        self._validate_attachment_points()


class GenericJoint(Constraint):
    """Generic joint, where user can fix (1) or free (0) any DoF."""

    def __init__(self, ap1: str | AttachmentPoint, ap2: str | AttachmentPoint, constraints: list, id: str | None = None) -> None:
        if not id:
            id = "test"
        super().__init__(id, ap1, ap2, constraints)


class PinConstraint(Constraint):
    """
    Represents a pin joint between two attachment points.

    Conventions:
        constrained_axes = [Tx, Ty, Tz, Rx, Ry, Rz]
        1 = constrained
        0 = free
    """

    AXIS_TO_CONSTRAINT = {
        "x": [1, 1, 1, 0, 1, 1],
        "y": [1, 1, 1, 1, 0, 1],
        "z": [1, 1, 1, 1, 1, 0],
    }

    def __init__(self, id, ap1, ap2):
        # derive hinge axis from geometry
        self.free_axis_local = ap1.axis_local

        super().__init__(id, ap1, ap2, self._compute_constrained_axes())


    # -------------------------------
    # Validation
    # -------------------------------
    def validate(self, tol=1e-6):
        import numpy as np

        # Position consistency
        p1 = self.ap1.global_position()
        p2 = self.ap2.global_position()

        if np.linalg.norm(p1 - p2).magnitude > tol:
            raise ValueError(f"{self.id}: pin positions do not coincide")

        # Axis alignment
        a1 = self.ap1.global_axis()
        a2 = self.ap2.global_axis()

        a1 = a1 / np.linalg.norm(a1)
        a2 = a2 / np.linalg.norm(a2)

        dot = float(np.dot(a1, a2))

        if abs(abs(dot) - 1.0) > tol:
            raise ValueError(
                f"{self.id}: pin axes not aligned (dot={dot:.6f})"
            )

    # -------------------------------
    # Debug-friendly repr
    # -------------------------------
    def __repr__(self):
        return (
            f"PinConstraint(id={self.id}, "
            f"free_rotation='{self.free_axis_local}', "
            f"constrained_axes={self.constraints})"
        )

    def _compute_constrained_axes(self):
        axis = self.free_axis_local

        if np.allclose(axis, [1,0,0]):
            return [1,1,1,0,1,1]
        elif np.allclose(axis, [0,1,0]):
            return [1,1,1,1,0,1]
        elif np.allclose(axis, [0,0,1]):
            return [1,1,1,1,1,0]
        else:
            raise ValueError(f"{self.id}: unsupported axis {axis}")
