"""Creates a shackle."""

import csv
import logging
from math import pi
from pathlib import Path
from typing import Self

import numpy as np

from .attachment_point import AttachmentPoint
from .rigid_body_base import RigidBodyBase
from . import ureg, Q_

logger = logging.getLogger(__name__)


class Transform:
    def __init__(self, position=None, rotation=None):
        self.position = np.array(position if position is not None else [0, 0, 0])
        self.rotation = rotation if rotation is not None else np.eye(3)

    def matrix(self):
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

    _DEFAULT_VISUAL_REF_LENGTH = 0.22/2 + 0.718 + 0.21/2    #800t SWL shackle

    resource_file = "shackles.csv"


    def __init__(
        self,
        id: str = None,
        model: str = None,
        *,
        manufacturer: str = None,
        wll: float = None,
#        pin_diameter: float = None,
        pin_diameter: float = 0,
        bow_diameter: float = None,
        inside_length: float = None,
    ) -> None:
        """Create a shackle object.

        id:         id to be given to shackle
        kind:       type and size of shackle to be created
        gravity:    3d numpy vector for gravity component
        color:      color to be given to the shackle
        """
        super().__init__(id=id)

        self.id = id
        self.model = model
        self.manufacturer = manufacturer
        self.wll = wll
        self.pin_diameter = pin_diameter
        self.bow_diameter = bow_diameter
        self.inside_length = inside_length

        self.pin = AttachmentPoint(
            id = "pin",
            parent = self,
            position_local = np.array([0, 0, 0]) * ureg.meters,
            axis_local = [1, 0, 0],
            radius = pin_diameter / 2,
        )
        self.parent = None
        self.transform = Transform()


    def global_transform(self):
        if self.parent:
            return self.parent.global_transform() @ self.transform.matrix()
        return self.transform.matrix()


    def connect_pin_to(self, target: AttachmentPoint):

        print("=== CONNECT DEBUG START ===")

        print("Target global:", target.global_position())
        print("Target axis  :", target.global_axis())

        print("Pin local pos:", self.pin.position_local)
        print("Pin local ax :", self.pin.axis_local)


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
#        if parent_body.parent is None:
#            # parent is root → easy case
#            self.position = t
#            self.rotation = R
#        else:
            # general case: convert global → parent-local
        R_parent = parent_body.global_rotation()
        t_parent = parent_body.global_position()

        self.rotation = R_parent.T @ R
        self.position = R_parent.T @ (t - t_parent)


        # --- 6. Attach to parent ---
        parent_body.add_child(self)


    @classmethod
    def from_model(cls, id: str, model: str) -> "Shackle":
        data = SHACKLE_LIBRARY.get(model)

        return cls(
            id = id,
            model = data.model,
            manufacturer = data.manufacturer,
            wll = data.wll,
            pin_diameter = data.pin_diameter,
            bow_diameter = data.bow_diameter,
            inside_length = data.inside_length,
        )

    @property
    def id(self: Self) -> str:
        """User-defined id of shackle."""
        return self._id

    @id.setter
    def id(self: Self, value: str):
        self._id = value


    @property
    def model(self: Self) -> str:
        """Make and size of shackle."""
        return self._model

    @model.setter
    def model(self: Self, value: str):
        self._model = value

    @property
    def wll(self: Self) -> float:
        """Shackle WLL."""
        return self._wll

    @wll.setter
    def wll(self: Self, value: float):
        self._wll = value


#    @property
#    def mbl(self: Self) -> float:
#        """Shackle MBL."""
#        return self.shackle_properties["MBL [t]"]

#    @property
#    def mass(self: Self) -> float:
#        """Shackle weight."""
#        return self.shackle_properties["weight [kg]"] / 1000

#    @property
#    def width(self: Self) -> float:
#        """Shackle width."""
#        return self.shackle_properties["width [mm]"] / 1000

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

#    @property
#    def bow_inside_diameter(self: Self) -> float:
#        """Shackle inside diameter."""
#        return self.shackle_properties["bow inside diameter [mm]"] / 1000

#    @property
#    def inside_width(self: Self) -> float:
#        """Shackle inside width."""
#        return self.shackle_properties["inside width [mm]"] / 1000

#    @property
#    def visual(self: Self) -> str:
#        """Shackle graphics."""
#        return self.shackle_properties["visual"]

#    @property
#    def description(self: Self) -> str:
#        """Shackle description."""
#        return self.shackle_properties["Description"]

#    @property
#    def visual_scale(self: Self) -> str:
#        """Shackle scale factor for graphics."""
#        if self.shackle_properties["visual scale"]:
#            return self.shackle_properties["visual scale"]

#        return (self.pin_diameter/2 + self.inside_length + self.bow_diameter/2) / self._DEFAULT_VISUAL_REF_LENGTH / 1000


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







#    @property
#    def shackle_rotation_matrix(self: Self) -> np.array:
#        """Return shackle's current rotation matrix."""
#        return self._mbs.GetObjectOutputBody(
#            self.body_number,
#            exu.OutputVariableType.RotationMatrix,
#            localPosition=[0, 0, 0],
#            configuration=exu.ConfigurationType.Reference,
#        ).reshape(3, 3)

#    def rotate(self, angle: float) -> None:
#        """Rotate shackle by 'angle' degrees about the pin."""
#        mbs = self._mbs

#        # get shackle node number and 1x7 reference array
#        shackle_node_no = mbs.GetObject(self.body_number)["nodeNumber"]
#        reference_coordinates = mbs.GetNode(shackle_node_no)["referenceCoordinates"]

#        # apply the desired rotation - convert degrees to radians
#        shackle_rotation_matrix = self.shackle_rotation_matrix @ exu.utilities.RotationMatrixX(np.radians(angle))

#        # update the reference coordinates
#        reference_coordinates[3:] = exu.utilities.RotationMatrix2EulerParameters(shackle_rotation_matrix)

#        # update mbs
#        mbs.SetNodeParameter(shackle_node_no, "referenceCoordinates", reference_coordinates)



class BaseAdapter:
    """Base adapter, handling I/O and csv loading tasks."""

    registry = []

    def __init_subclass__(cls):
        BaseAdapter.registry.append(cls)

    def __init__(self, resource_path: str, filename: str):
        self.resource_path = resource_path
        self.filename = filename

    def load(self):
        """Main entry point: returns list of canonical objects."""
        rows = self._read_csv()
        return [self.map_row(row) for row in rows]

    def _read_csv(self):
        """Generic CSV loader."""
        path = Path(self.resource_path).joinpath(self.filename)

        with path.open("r", newline="") as f:
            reader = csv.DictReader(f, delimiter=";")
            return list(reader)

    def map_row(self, row: dict):
        """To be implemented by subclasses."""
        raise NotImplementedError


class CrosbyAdapter(BaseAdapter):
    """Parse Crosby shackles - map catalogue values to class properties."""

    def __init__(self):
        super().__init__(
            resource_path="lift_solver/data/shackles",
            filename="crosby_shackles.csv",
        )

    def map_row(self, row: dict):
        return Shackle(
            model = row["Model"],
            manufacturer = row["Manufacturer"],
            wll = float(row["WLL [tonne]"]) * ureg.metric_ton,
            pin_diameter = float(row["B [mm]"]) * ureg.millimeter,
            bow_diameter = float(row["D [mm]"]) * ureg.millimeter,
            inside_length = float(row["C [mm]"]) * ureg.millimeter,
        )


class GnAdapter(BaseAdapter):
    """Parse GN shackles - map catalogue values to class properties."""
    def __init__(self):
        super().__init__(
            resource_path="lift_solver/data/shackles",
            filename="gn_shackles.csv",
        )

    def map_row(self, row: dict):
        return Shackle(
            model = row["Model"],
            manufacturer = row["Manufacturer"],
            wll = float(row["WLL [tonne]"]) * ureg.metric_ton,
            pin_diameter = float(row["B [mm]"]) * ureg.millimeter,
            bow_diameter = float(row["A [mm]"]) * ureg.millimeter,
            inside_length = float(row["D [mm]"]) * ureg.millimeter,
        )


class GreenPinAdapter(BaseAdapter):
    """Parse GreenPin shackles - map catalogue values to class properties."""
    def __init__(self):
        super().__init__(
            resource_path="lift_solver/data/shackles",
            filename="gp_shackles.csv",
        )

    def map_row(self, row: dict) -> Shackle:
        return Shackle(
            model = row["Model"],
            manufacturer = row["Manufacturer"],
            wll=float(row["working load limit [ton]"]) * ureg.metric_ton,
            pin_diameter=float(row["diameter pin B [mm]"]) * ureg.millimeter,
            bow_diameter=float(row["diameter body A [mm]"]) * ureg.millimeter,
            inside_length=float(row["length inside F [mm]"]) * ureg.millimeter,
        )


class ShackleLibrary:

    def __init__(self):
        self._data = {}
        self._is_loaded = False


    def load(self):
        if self._is_loaded:
            return

        for AdapterClass in BaseAdapter.registry:
            adapter = AdapterClass()
            for shackle in adapter.load():
                self.add(shackle)

        self._is_loaded = True


    def add(self, shackle: Shackle):
        self._data[shackle.model] = shackle


    def get(self, model: str) -> Shackle:
        if not self._is_loaded:
            self.load()

        if model not in self._data:
            raise KeyError(f"Shackle not found: {model}")

        return self._data[model]


SHACKLE_LIBRARY = ShackleLibrary()
