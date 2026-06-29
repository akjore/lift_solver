import logging
from collections.abc import Mapping, Sequence

import numpy as np
import pint
# import simplejson
import yaml

from . import Q_, ureg
from .rigid_body import RigidBody
from .shackle import Shackle
from .sling import Sling
from .constraint import GenericJoint, World

logger = logging.getLogger(__name__)


class LiftProblem():
    """Data related to a single lift problem."""

    def __init__(self):
        self.objects = {}               # Rigid bodies (including shackles)
        self.attachment_points = {}     # All attachment points
        self.connections = {}           # Pin joints, etc.
        self.rigging = {}               # All slings and grommets

        self.world = World()


    def from_yaml(self, text: str):
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


    def from_dict(self, problem: dict):
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


    def add_body(self, body: dict):
        """Add a body to the lift problem with the properties given by 'body'."""
        bdy = RigidBody(body["id"])
        bdy.from_dict(body)

        self.objects[bdy.id] = bdy

        for val in bdy.attachment_points.values():
            key = bdy.id + "." + val.id
            self.attachment_points[key] = val

        return bdy


    def add_shackle(self, shackle: dict):
        """Add a shackle to the lift problem with the properties given by 'shackle'."""
        sh = Shackle().from_model(shackle["id"], shackle["model"])

        # Add shackle's attachment points to registry
        self.attachment_points[sh.pin.id] = sh.pin
        self.attachment_points[sh.bow.id] = sh.bow

        # Add shackle to registry
        self.objects[sh.id] = sh

        # If pin_connection specified, move shackle and align pin with attachment point
        pin_connection = shackle.get("pin_connection")
        if pin_connection:
            pin_constraint = sh.connect_pin_to(self.attachment_points[pin_connection])
            self.connections[pin_constraint.id] = pin_constraint

        rotation_about_pin = shackle.get("rotation_about_pin")
        if rotation_about_pin:
            sh.rotation_about_pin = rotation_about_pin

        # If shackle pose was provided, use that - note either position is derived from pin_connnection and optional pin_rotation, or pose
        pose = shackle.get("pose")
        if pose:
            sh.set_pose(pose.get("position"), pose.get("orientation"))

        return sh


    def add_sling(self, sling: dict):
        """Add a sling to the lift problem with the properties given by 'sling'."""
        sl = Sling(**sling)

        # Resolve attachment points and sheaves to AttachmentPoint objects
        sl.end_a = self.attachment_points[sl.end_a]
        sl.end_b = self.attachment_points[sl.end_b]
        sl.sheaves = [self.attachment_points[sheave] for sheave in sl.sheaves]

        self.rigging[sl.id] = sl


    def add_constraint(self, constraint: dict):
        """Add a constraint to the lift problem with the properties give by 'constraint'."""
        cn = GenericJoint(**constraint)

        cn.ap1 = self.attachment_points[cn.ap1]
        if not cn.ap2 or cn.ap2 == "world":
            cn.ap2 = self.world
        else:
            cn.ap2 = self.attachment_points[cn.ap2]

        self.connections[cn.id] = cn


    def normalize_units(self, obj):
        """Recursively convert YAML-loaded structure into Pint quantities."""

        # --- dict ---
        if isinstance(obj, Mapping):
            return {k: self.normalize_units(v) for k, v in obj.items()}

        # --- list / tuple ---
        elif isinstance(obj, Sequence) and not isinstance(obj, (str, bytes)):
            # detect if this is a vector (flat numeric list)
            if all(self.is_scalar(v) for v in obj):
                return self.parse_vector(obj)
            else:
                return [self.normalize_units(v) for v in obj]

        # --- scalar ---
        elif isinstance(obj, str):
            try:
                return self.parse_quantity(obj)
            except pint.UndefinedUnitError:
                return obj

        else:
            return obj


    def parse_quantity(self, value):
        """Convert YAML value into a Pint Quantity."""
        if isinstance(value, str):
            return Q_(value)
        return value


    def parse_vector(self, vec):
        lst = [self.parse_quantity(v) for v in vec]
        try:
            return Q_.from_list(lst)
        except AttributeError:
            return lst


    def is_scalar(self, value):
        return isinstance(value, (int, float, str))


#    def positions_equal(self, a, b, tol=1e-9):
#        return np.allclose(
#            a.to("meter").magnitude,
#            b.to("meter").magnitude,
#            atol = tol,
#        )