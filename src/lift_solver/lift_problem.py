import logging
from collections.abc import Mapping, Sequence

import numpy as np
import pint
# import simplejson
import yaml

from . import Q_, ureg
from .rigid_body import RigidBody
from .shackle import Shackle

logger = logging.getLogger(__name__)


class LiftProblem():
    """Data related to a single lift problem."""

    def __init__(self):
        pass


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

    def add_to_registry(self, prefix:str, lst: list) -> None:
        for itm in lst:
            key = prefix + "." + itm.id
            self.registry[key] = itm

    def resolve_attachment(self, attachment_ref: str):
        return self.registry[attachment_ref]

    def add_body(self, body: dict):
        """Add a body to the lift problem with the properties given by 'body'."""
        bdy = RigidBody(body["id"])
        bdy.from_dict(body)
        self.add_to_registry(bdy.id, bdy.children)
        return bdy


    def add_shackle(self, shackle: dict):
        """Add a shackle to the lift problem with the properties given by 'shackle'."""
        sh = Shackle().from_model(shackle["id"], shackle["model"])

        target = self.resolve_attachment(shackle.get("pin_connection"))
        sh.connect_pin_to(target)

        return sh


    def positions_equal(self, a, b, tol=1e-9):
        print(f"a: {a}")
        print(f"b: {b}")
        return np.allclose(
            a.to("meter").magnitude,
            b.to("meter").magnitude,
            atol=tol
        )



    def from_dict(self, problem: dict):
        # Set up environment
        self.g = problem["environment"]["gravity"]

        # Create the bodies
        for body in problem["bodies"]:
            bdy = self.add_body(body)
            self.registry[bdy.id] = bdy

        # Create the shackles
        for shackle in problem["shackles"]:
            sh = self.add_shackle(shackle)
            self.registry[sh.id] = sh

        # Create the slings
#        for sling in problem["elements"]:
#            self.components.append(Sling(sling))


        # Create constraints
#        for constraint in problem["constraints"]:
#            self.components.append(Constraint(constraint))

#        print("The following components exist in the registry:")
#        for component in self.registry.values():
#            print()
#            print(f"id: {component.id}")
#            print(f"    mass: {component.mass}")
#            print(f"    position: {component.position}")
#            print(f"    global_position: {component.global_position()}")
#            print(f"    parent: {component.parent}")
#            print(f"    children: {component.children}")
#            for child in component.children:
#                print(f"        Child id: {child.id}")
#                print(f"        Child position: {child.position_local}")
#                print(f"        Child global position: {child.global_position()}")

#        assert self.registry["sh1"].pin.global_position() == self.registry["spreader.left_lower"].global_position()

        print(self.positions_equal(
            self.registry["sh1"].pin.global_position(),
            self.registry["spreader.left_lower"].global_position()
        ))
