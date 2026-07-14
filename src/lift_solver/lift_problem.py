"""Module for handling a single lift problem."""
import logging
from collections.abc import Mapping, Sequence
from typing import Self

import numpy as np
import pint
import yaml

from . import Q_
from .constraint import GenericJoint, World
from .rigid_body import RigidBody
from .shackle import Shackle
from .sling import Sling

logger = logging.getLogger(__name__)


#QUANTITY_KEYS = {
#    "position",
#    "cog",
#    "mass",
#    "diameter",
#    "length",
#    "hole_diameter",
#    "outer_diameter",
#    "thickness",
#    "size",
#}

NON_QUANTITY_KEYS = {
    "id",
    "type",
    "stl",
    "catalogue",
    "material",
    "color",
    "parent",
}


class LiftProblem:
    """Data related to a single lift problem."""

    def __init__(self: Self) -> None:
        """Initialize an empty lift problem."""
        self.bodies = {}                # Rigid bodies
        self.shackles = {}              # Shackles
        self.attachment_points = {}     # All attachment points
        self.connections = {}           # Pin joints, etc.
#        self.rigging = {}               # All slings and grommets
        self.slings = {}                # Slings

        self.world = World()


    def from_yaml(self: Self, text: str) -> "LiftProblem":
        """Parse the provided text as yaml-input, and build the lift problem to be solved."""
        # Keep a copy of the provided data
        self._raw = yaml.load(text, Loader=yaml.SafeLoader)
        self.registry = {}

        try:
            data = self.normalize_units(self._raw)
            self.registry = {}
            self.from_dict(data)
        except ValueError as exc:
            msg = "No input or malformed input provided."
            raise ValueError(msg) from exc
        else:
            return self


    def from_dict(self: Self, problem: dict) -> None:
        """Parse a dict into a lift problem."""
        # Set up environment
        self.g = problem["environment"]["gravity"]

        # Add the bodies
        bodies = problem.get("bodies")
        if bodies:
            for body in bodies:
                self.add_body(body)

        # Add the shackles
        shackles = problem.get("shackles")
        if shackles:
            for shackle in shackles:
                self.add_shackle(shackle)

        # Add the slings
        slings = problem.get("elements")
        if slings:
            for sling in slings:
                self.add_sling(sling)

        # Add the constraints
        constraints = problem.get("constraints")
        if constraints:
            for constraint in constraints:
                self.add_constraint(constraint)

        # Optionally update starting positions
        initial_state = problem.get("initial_state")
        if initial_state:
            self.apply_initial_state(initial_state)

        # Set code check parameters (optional)
        self.code_check_settings = problem.get("code_check")


    def add_body(self: Self, body: dict) -> None:
        """Add a body to the lift problem with the properties given by 'body'."""
        bdy = RigidBody(body["id"])
        bdy.from_dict(**body)

        self.bodies[bdy.id] = bdy

        for val in bdy.attachment_points.values():
            key = bdy.id + "." + val.id
            self.attachment_points[key] = val

        return bdy


    def add_shackle(self: Self, shackle: dict) -> None:
        """Add a shackle to the lift problem with the properties given by 'shackle'."""
        sh = Shackle().from_model(shackle["id"], shackle["model"])

        # Add shackle's attachment points to registry
        self.attachment_points[sh.pin.id] = sh.pin
        self.attachment_points[sh.bow.id] = sh.bow

        # Add shackle to registry
        self.shackles[sh.id] = sh

        # If pin_connection specified, move shackle and align pin with attachment point
        pin_connection = shackle.get("pin_connection")
        if pin_connection:
            pin_constraint = sh.connect_pin_to(self.attachment_points[pin_connection])
            self.connections[pin_constraint.id] = pin_constraint

        rotation_about_pin = shackle.get("rotation_about_pin")
        if rotation_about_pin:
            sh.set_rotation_about_pin(rotation_about_pin)

        # If shackle pose was provided, use that: Note either position is derived from pin_connnection
        #  and optional pin_rotation, or pose.
        pose = shackle.get("pose")
        if pose:
            sh.set_pose(pose.get("position"), pose.get("orientation"))

        return sh


    def add_sling(self: Self, sling: dict) -> None:
        """Add a sling to the lift problem with the properties given by 'sling'."""
        sl = Sling(**sling)

        # Resolve attachment points and sheaves to AttachmentPoint objects
        sl.end_a = self.attachment_points[sl.end_a]
        sl.end_b = self.attachment_points[sl.end_b]
        sl.sheaves = [self.attachment_points[sheave] for sheave in sl.sheaves]

        self.slings[sl.id] = sl


    def add_constraint(self: Self, constraint: dict) -> None:
        """Add a constraint to the lift problem with the properties give by 'constraint'."""
        cn = GenericJoint(**constraint)

        cn.ap1 = self.attachment_points[cn.ap1]
        if not cn.ap2 or cn.ap2 == "world":
            cn.ap2 = self.world
        else:
            cn.ap2 = self.attachment_points[cn.ap2]

        self.connections[cn.id] = cn


    def normalize_units(self: Self, obj: object, key=None) -> object:
        """Recursively convert YAML-loaded structure into Pint quantities."""
        # --- dict ---
        if isinstance(obj, Mapping):
            return {k: self.normalize_units(v, k) for k, v in obj.items()}

        # --- list / tuple ---
        elif isinstance(obj, Sequence) and not isinstance(obj, str | bytes):
            # detect if this is a vector (flat numeric list)
            if all(self.is_scalar(v) for v in obj):
                return self.parse_vector(obj)
            else:
                return [self.normalize_units(v) for v in obj]

        # --- scalar ---
        elif isinstance(obj, str) and key not in NON_QUANTITY_KEYS:
            try:
                return self.parse_quantity(obj)
            except pint.UndefinedUnitError:
                return obj

        else:
            return obj


    def parse_quantity(self: Self, value: str) -> pint.Quantity:
        """Convert a str into a Pint Quantity."""
        if isinstance(value, str):
            return Q_(value)
        return value


    def parse_vector(self: Self, vec: list) -> pint.Quantity:
        """Convert a list of quantities into a quantity of a numpy array."""
        lst = [self.parse_quantity(v) for v in vec]
        try:
            return Q_.from_list(lst)
        except pint.DimensionalityError:
            # list contains mix of e.g. meters and deg
            return lst
        except AttributeError:
            return lst


    def is_scalar(self: Self, value: object) -> bool:
        """Check of 'value' is an int | float | str."""
        return isinstance(value, int | float | str)


    def apply_initial_state(self: Self, initial_state: dict):
        """
        Apply initial_state using RigidBodyBase methods.

        Rules:
        - parent=None  → absolute (global) pose
        - parent!=None → local (relative) pose
        """

        resolved = set()

        def parse_pose(values):
            # [x, y, z, rx, ry, rz] with units
            x, y, z, rx, ry, rz = values

            position = Q_.from_list([x, y, z])
            orientation = Q_.from_list([rx, ry, rz])

            return position, orientation

        # Resolve in dependency order
        while len(resolved) < len(initial_state):

            progress = False

            for obj_id, values in initial_state.items():

                if obj_id in resolved:
                    continue

                obj = self.bodies.get(obj_id)
                if obj is None:
                    obj = self.shackles.get(obj_id)

                # --- ROOT: absolute pose ---
                if obj.parent is None:
                    pos, ori = parse_pose(values)

                    obj.set_global_pose(
                        pos,
                        obj._euler_to_matrix(ori)
                    )

                    resolved.add(obj_id)
                    progress = True

                # --- CHILD: relative pose ---
                else:
                    parent = obj.parent

                    if parent.id not in resolved:
                        continue  # wait for parent

                    pos_rel, ori_rel = parse_pose(values)

                    obj.set_local_pose(
                        pos_rel,
                        obj._euler_to_matrix(ori_rel)
                    )

                    resolved.add(obj_id)
                    progress = True

            if not progress:
                unresolved = set(initial_state.keys()) - resolved
                raise RuntimeError(
                    f"Could not resolve initial_state; unresolved: {unresolved}"
                )

    def to_render_model(self):
        return {
            "bodies": [self._render_dict(obj) for obj in self.bodies.values()],
            "shackles": [self._render_dict(obj) for obj in self.shackles.values()],
            "rigging": [self._render_dict(obj) for obj in self.rigging.values()]
        }

    def _render_dict(self, obj):
        d = obj.to_dict()

        return cnv_quantity(d)


def cnv_quantity(value) -> dict:
    if isinstance(value, dict):
        return {
            k: cnv_quantity(v)
            for k, v in value.items()
        }

    if isinstance(value, (list, tuple)):
        return [
            cnv_quantity(v)
            for v in value
        ]

    if isinstance(value, np.ndarray):
        return value.tolist()

    if isinstance(value, pint.Quantity):
        val = value
        if val.check("[length]"):
            val.ito("m")
        elif value.check("[mass]"):
            val.ito("kg")
        elif value.check("[]"):
            val.ito("rad")

        mag = val.magnitude
        unit = val.units
        if isinstance(mag, np.ndarray):
            mag = mag.tolist()

        return {
            "magnitude": mag,
            "units": f"{unit:~P}",
        }

    return value
