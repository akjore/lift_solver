"""Creates a shackle."""

import csv
import logging
from pathlib import Path
from typing import Self

import numpy as np

from . import Q_, ureg
from .attachment_point import AttachmentPoint
from .constraint import PinConstraint
from .rigid_body_base import RigidBodyBase
from .visual_geometry import Mesh

logger = logging.getLogger(__name__)


class Transform:
    """Class to hold transformation details."""

    def __init__(self, position: np.array(3) = None, rotation: np.array(3) = None) -> None:
        """Initialize."""
        self.position = np.array(position if position is not None else [0, 0, 0])
        self.rotation = rotation if rotation is not None else np.eye(3)

    def matrix(self) -> np.array(3):
        """Return transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.position
        return T


class Shackle(RigidBodyBase):
    """Create a shackle.

    Shackle properties are loaded from external resources
    Local coordinate system:
        Right handed system, origin at centre of pin
        Shackle is modelled in the xz plane
        x-axis along pin
        positive z-axis towards the shackle bow

    Shackle is initially created at the global origin.
    """

    resource_file = "shackles.csv"


    def __init__(
        self,
        id: str = "",
        model: str = None,
        *,
        manufacturer: str = None,
        wll: float = None,
        pin_diameter: float = 0,
        bow_diameter: float = 0,
        inside_length: float = 0,
        mass: float = 0,
        sub_type: str = ""
    ) -> None:
        """Create a shackle object.

        id:             id to be given to shackle
        model:          type and size of shackle to be created
        manufacturer:   manufacturer of shackle
        wll:            working load limit
        pin_diameter:   pin diameter
        bow_diameter:   bow diameter
        """
        super().__init__(id=id)


        self.id = id
        self.model = model
        self.manufacturer = manufacturer
        self.wll = wll
        self.pin_diameter = pin_diameter
        self.bow_diameter = bow_diameter
        self.inside_length = inside_length
        self.mass = mass
        self.sub_type = sub_type

        self.visual = None

        self.pose = None
        self.pin_connection = None
        self.rotation_about_pin = Q_("0 deg")


        self.pin = AttachmentPoint(
            id = self.id + ".pin",
            parent = self,
            position_local = self._pin_ap_position,
            axis_local = self._pin_axis,
            radius = self.pin_diameter / 2,
        )

        self.bow = AttachmentPoint(
            id = self.id + ".bow",
            parent = self,
            position_local = self._bow_ap_position,
            axis_local = self._pin_axis,
            radius = self.bow_diameter / 2,
        )

        self.attachment_points[self.pin.id] = self.pin
        self.attachment_points[self.bow.id] = self.bow

        self.parent = None
        self.transform = Transform()

        if self.id:
            self.mesh = Mesh(
                file = "lift_solver/data/shackles/shackle_gp800.stl",
                scale = self._visual_scale / 1000,
                rotation = self._stl_to_shackle_rotation(),
                translation = self._stl_to_shackle_offset()
            )


    def global_rotation(self: Self) -> np.array(3):
        """Return global rotation of shackle."""
        R = super().global_rotation()

        if self.rotation_about_pin:
            angle = self.rotation_about_pin.to("rad").magnitude

            axis = self.pin.axis_local

            R_flip = self.rot_axis_angle(axis, angle)

            R = R_flip @ R

        return R


    @property
    def mode(self: Self) -> str:
        """Return if shackle is connected to a body."""
        if self.pin_connection:
            return "connected"
        elif self.pose:
            return "free"
        return "free"


    def rot_axis_angle(self: Self, axis: np.array(3), angle: float) -> np.array(3):
        """Return rotation matrix based on rotation and axis."""
        axis = np.asarray(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)

        x, y, z = axis

        c = np.cos(angle)
        s = np.sin(angle)
        C = 1 - c

        R = np.array([
            [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
            [z*x*C - y*s,   z*y*C + x*s, c + z*z*C  ]
        ])

        return R


    def global_transform(self: Self) -> np.array(3):
        """Return transformation matrix."""
        if self.parent:
            return self.parent.global_transform() @ self.transform.matrix()
        return self.transform.matrix()


    def connect_pin_to(self: Self, target: AttachmentPoint) -> PinConstraint:
        """Move shackle pin such that it coincides with target, and pin aligns with hole axis."""
        constraint = None
        if target:
            if self.parent is not None:
                self.parent.remove_child(self)

            # --- 1. Extract target info ---
            parent_body = target.parent

            p_target = target.global_position()     # global coords (with units)
            axis_target = target.global_axis()      # unit vector

            # --- 2. Local pin definition ---
            p_local = self.pin.position_local
            axis_local = self.pin.axis_local

            # --- 3. Compute rotation ---
            R = self.rotation_matrix_from_vectors(axis_local, axis_target)

            # --- 4. Compute position ---
            t = p_target - R @ p_local

            # --- 5. Convert into parent-local coordinates ---
            R_parent = parent_body.global_rotation()
            t_parent = parent_body.global_position()

            self.rotation = R_parent.T @ R
            self.position = R_parent.T @ (t - t_parent)

            # --- 6. Attach to parent ---
            parent_body.add_child(self)

            # --- 7. Return pin constraint ---
            constraint = PinConstraint(
                id = f"{self.pin.id}.constraint",
                ap1 = self.pin,
                ap2 = target,
            )
            constraint.validate()
        else:
            logger.debug("No target specified - skipping.")
        return constraint


    @classmethod
    def from_model(cls, id: str, model: str) -> "Shackle":
        """Return a shackle object based on provided model."""
        data = SHACKLE_LIBRARY.get(model)

        return cls(
            id = id,
            model = data.model,
            manufacturer = data.manufacturer,
            wll = data.wll,
            pin_diameter = data.pin_diameter,
            bow_diameter = data.bow_diameter,
            inside_length = data.inside_length,
            mass = data.mass,
        )

    @property
    def id(self: Self) -> str:
        """User-defined id of shackle."""
        return self._id

    @id.setter
    def id(self: Self, value: str) -> None:
        self._id = value


    @property
    def model(self: Self) -> str:
        """Make and size of shackle."""
        return self._model

    @model.setter
    def model(self: Self, value: str) -> None:
        self._model = value


    @property
    def wll(self: Self) -> float:
        """Shackle WLL."""
        return self._wll

    @wll.setter
    def wll(self: Self, value: float) -> None:
        self._wll = value


    @property
    def _pin_ap_position(self: Self) -> float:
        """Position of the centre of the pin."""
        # Definition choice. Centre of pin at origin. Shackle modelled in the xz-plane
        return np.array([0, 0, 0]) * ureg.meters


    @property
    def _pin_axis(self: Self) -> float:
        """Longitudinal axis of pin."""
        # Definition choice.
        return [1, 0, 0]


    @property
    def _bow_ap_position(self: Self) -> float:
        """Return position of the centre of the pin."""
        # Ensure bearing-bearing length is right
        ap_distance = 0.5 * self.pin_diameter + self.inside_length + 0.5 * self.bow_diameter
        return self._pin_ap_position + np.array([0, 0, 1]) * ap_distance


    @property
    def pin_diameter(self: Self) -> float:
        """Shackle pin diameter."""
        return self._pin_diameter

    @pin_diameter.setter
    def pin_diameter(self: Self, value: float) -> None:
        self._pin_diameter = value


    @property
    def bow_diameter(self: Self) -> float:
        """Shackle bow diameter."""
        return self._bow_diameter

    @bow_diameter.setter
    def bow_diameter(self: Self, value: float) -> None:
        self._bow_diameter = value


    @property
    def inside_length(self: Self) -> float:
        """Shackle inside length - bearing to bearing."""
        return self._inside_length

    @inside_length.setter
    def inside_length(self: Self, value: float) -> None:
        self._inside_length = value


    @property
    def mass(self: Self) -> float:
        """Shackle mass."""
        return self._mass

    @mass.setter
    def mass(self: Self, value: float) -> None:
        self._mass = value


    @property
    def cog(self: Self) -> float:
        """Shackle CoG. Not provided in catalogues - set to reasonable value."""
        return self._pin_ap_position + np.array([0, 0, 1]) * 0.5 * self.inside_length


    @property
    def sub_type(self: Self) -> float:
        """Sub type."""
        return self._sub_type

    @sub_type.setter
    def sub_type(self: Self, value: str) -> None:
        self._sub_type = value


    def _stl_to_shackle_rotation(self: Self) -> np.array(3):
        # e.g. STL has pin along Z → rotate to X
        return np.eye(3)


    def _stl_to_shackle_offset(self: Self) -> np.array(3):
        # shift STL origin to pin center
        return np.array([0, 0, 0]) * ureg.meters


    @property
    def _visual_scale(self: Self) -> str:
        """Shackle scale factor for graphics."""
        # Reference shackle is a GP 800t shackle. Scale so bearing-to-bearing length matches
        ref_length = 1
        if self.sub_type == "WideBody":
            ref_shackle = Shackle().from_model("", "GPWB800")
            ref_length = ref_shackle.inside_length
        else:
            ref_shackle = Shackle().from_model("", "GP800")
            ref_length = ref_shackle.inside_length

        return self.inside_length / ref_length


    def rotation_matrix_from_vectors(self: Self, vec1: np.array, vec2: np.array) -> np.array:
        """Find a rotation matrix that aligns vec1 to vec2.

        vec1: A 3d "source" vector
        vec2: A 3d "destination" vector

        return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        """
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        if any(v): #if not all zeros then
            c = np.dot(a, b)
            s = np.linalg.norm(v)
            kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))

        return np.eye(3)


class BaseAdapter:
    """Base adapter, handling I/O and csv loading tasks."""

    registry = []

    def __init_subclass__(cls) -> None:
        """Add adapter's contribution to library."""
        BaseAdapter.registry.append(cls)

    def __init__(self: Self, resource_path: str, filename: str) -> None:
        """Initialize."""
        self.resource_path = resource_path
        self.filename = filename

    def load(self: Self) -> list:
        """Parse csv and return list of loaded items."""
        rows = self._read_csv()
        return [self.map_row(row) for row in rows]

    def _read_csv(self: Self) -> list:
        """Load CSV."""
        path = Path(self.resource_path).joinpath(self.filename)

        with path.open("r", newline="") as f:
            reader = csv.DictReader(f, delimiter=";")
            return list(reader)

    def map_row(self: Self, row: dict) -> None:
        """To be implemented by subclasses."""
        raise NotImplementedError


class CrosbyAdapter(BaseAdapter):
    """Parse Crosby shackles - map catalogue values to class properties."""

    def __init__(self: Self) -> None:
        """Initialize."""
        super().__init__(
            resource_path = "lift_solver/data/shackles",
            filename = "crosby_shackles.csv",
        )

    def map_row(self, row: dict) -> Shackle:
        """Instantiate a shackle based on an entry from Crosby."""
        # Wide body shackles
        bow_diameter = row["J [mm]"]
        bow_diameter = float(bow_diameter) if bow_diameter else float(float(row["D [mm]"]))

        return Shackle(
            model = row["Model"],
            manufacturer = row["Manufacturer"],
            wll = float(row["WLL [tonne]"]) * ureg.metric_ton,
            pin_diameter = float(row["B [mm]"]) * ureg.millimeter,
            bow_diameter = bow_diameter * ureg.millimeter,
            inside_length = float(row["C [mm]"]) * ureg.millimeter,
            mass = float(row["Weight Each [kg]"]) * ureg.kg,
            sub_type = row["Subtype"],
        )


class GnAdapter(BaseAdapter):
    """Parse GN shackles - map catalogue values to class properties."""

    def __init__(self: Self) -> None:
        """Initialize."""
        super().__init__(
            resource_path = "lift_solver/data/shackles",
            filename = "gn_shackles.csv",
        )

    def map_row(self: Self, row: dict) -> Shackle:
        """Instantiate a shackle based on an entry from GN."""
        # Wide body shackles
        bow_diameter = row["G [mm]"]
        bow_diameter = float(bow_diameter) if bow_diameter else float(float(row["A [mm]"]))

        return Shackle(
            model = row["Model"],
            manufacturer = row["Manufacturer"],
            wll = float(row["WLL [tonne]"]) * ureg.metric_ton,
            pin_diameter = float(row["B [mm]"]) * ureg.millimeter,
            bow_diameter = bow_diameter * ureg.millimeter,
            inside_length = float(row["D [mm]"]) * ureg.millimeter,
            mass = float(row["Weight [kg]"]) * ureg.kg,
            sub_type = row["Subtype"],
        )


class GreenPinAdapter(BaseAdapter):
    """Parse GreenPin shackles - map catalogue values to class properties."""

    def __init__(self: Self) -> None:
        """Initialize."""
        super().__init__(
            resource_path = "lift_solver/data/shackles",
            filename = "gp_shackles.csv",
        )

    def map_row(self: Self, row: dict) -> Shackle:
        """Instantiate a shackle based on an entry from GreenPin."""
        # Wide body shackles
        bow_diameter = row["bearing surface L [mm]"]
        bow_diameter = float(bow_diameter) if bow_diameter else float(float(row["diameter body A [mm]"]))

        return Shackle(
            model = row["Model"],
            manufacturer = row["Manufacturer"],
            wll = float(row["working load limit [ton]"]) * ureg.metric_ton,
            pin_diameter = float(row["diameter pin B [mm]"]) * ureg.millimeter,
            bow_diameter = bow_diameter * ureg.millimeter,
            inside_length = float(row["length inside F [mm]"]) * ureg.millimeter,
            mass = float(row["Net weight [kg]"]) * ureg.kg,
            sub_type = row["Subtype"],
        )


class ShackleLibrary:
    """Class to hold shackle library."""

    def __init__(self: Self) -> None:
        """Initialize library."""
        self._data = {}
        self._is_loaded = False

    def load(self: Self) -> None:
        """Load library."""
        if self._is_loaded:
            return

        for AdapterClass in BaseAdapter.registry:
            adapter = AdapterClass()
            for shackle in adapter.load():
                self.add(shackle)

        self._is_loaded = True

    def add(self: Self, shackle: Shackle) -> None:
        """Add a shackle to the library."""
        self._data[shackle.model] = shackle

    def get(self: Self, model: str) -> Shackle:
        """Get a shackle from the library based on model."""
        if not self._is_loaded:
            self.load()

        if model not in self._data:
            raise KeyError(f"Shackle not found: {model}")

        return self._data[model]


SHACKLE_LIBRARY = ShackleLibrary()
