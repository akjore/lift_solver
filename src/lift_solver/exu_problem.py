"""Module for exudy problem definition."""
import logging
from typing import Self

# Temporary workaround while making exudyn available in pyodide
try:
    import exudyn as exu
    from exudyn import graphics
except ImportError:
    exu = None
    graphics = None

# import exudyn as exu
import numpy as np
#from exudyn import graphics

from .attachment_point import AttachmentPoint
from .constraint import Constraint, PinConstraint, World
from .lift_problem import LiftProblem
from .rigid_body import RigidBody
from .sling import Sling
from .visual_geometry import BoxVisual, CylinderVisual, MeshVisual

logger = logging.getLogger(__name__)


class ExuProblem:
    """Class to set up an exudyn version of a single LiftProblem."""

    # Exudyn units: SI
    # Forces in N
    # Mass in kg
    # Lengths in m

    def __init__(self: Self, problem: LiftProblem) -> None:
        """Initialize and set up an exudyn representation of the lift problem."""
        self.problem = problem

       # Set up mbs
        self.SC = exu.SystemContainer()
        self.mbs = self.SC.AddSystem()
        self.g = None
        self.ground = None
        self.problem = problem

        self.bodies = {}
        self.shackles = {}
        self.slings = {}
        self.constraints = {}
        self.attachment_points = {}

        # Create world reference system
        self.ground = self.create_ground()

        # Create model
        self.setup_from_problem(problem)

        # Add damping
        self.create_damping(problem)


    def create_ground(self: Self) -> exu.ObjectIndex:
        """Create exudyn ground, world fixed reference system."""
        g_ground = graphics.CheckerBoard(point=[0,0,0], normal = [0,0,1], size=60, nTiles=12)
        return self.mbs.AddObject(
            exu.utilities.ObjectGround(
                visualization=exu.utilities.VObjectGround(
                    graphicsData=[g_ground]
                )
            )
        )


    def setup_from_problem(self, problem: LiftProblem) -> None:
        """Set up problem - convert quantities to SI units."""
        # Set up environment
        self.g = problem.g.to("m/s/s").magnitude

        # Need a representative mass to tune damping coefficients
        representative_mass = (max([b.mass for b in problem.bodies.values() if isinstance(b, RigidBody)]))

        # Create and place the bodies
        for body in problem.bodies.values():
            body_number, attachment_points = self.create_body(body)
            self.bodies[body.id] = body_number
            self.attachment_points = self.attachment_points | attachment_points

        # Create and place the shackles
        for shackle in problem.shackles.values():
            shackle_number, attachment_points = self.create_body(shackle)
            self.shackles[shackle.id] = shackle_number
            self.attachment_points = self.attachment_points | attachment_points

        # Create slings between attachment points
        for sl in problem.slings.values():
            if isinstance(sl, Sling):
                self.slings[sl.id] = self.create_sling(sl, representative_mass)

        # Apply constraints
        for constraint in problem.connections.values():
            self.constraints[constraint.id] = self.create_constraint(constraint)


    def create_body(self: Self, body: RigidBody) -> exu.BodyIndex:
        """Create an exudyn body from body."""
        mass = body.mass.to("kg").magnitude
        cog = body.cog.to("m").magnitude

        position = body.global_position().to("m").magnitude
        orientation = body.global_rotation()

        # TODO: Inertia hard-coded - if realistic dynamic simulations are required this should be improved by
        # e.g. letting user specify in yaml.
        length = 1
        width = 1
        height = 1
        density = mass / (length * width * height)

        inertia = exu.utilities.InertiaCuboid(
            density = density,
            sideLengths = [length, width, height],
        )

        # Shift the cuboid cog from [0, 0, 0] to the CoG
        inertia = inertia.Translated(cog)

        # Create graphics
        graphics_data_list = []
        graphics_data_list.append(self.create_graphics(cog, body.visual))
        graphics_data_list.append(graphics.Basis(inertia.COM(), length=0.5))

        # Create the body
        body_number = self.mbs.CreateRigidBody(
            name = body.id,
            inertia = inertia,
            gravity = self.g,
            referencePosition = position,
            referenceRotationMatrix = orientation,
            graphicsDataList = graphics_data_list,
        )

        attachment_points = {}
        for att in body.attachment_points.values():
            attachment_points[att.id] = self.create_attachment_point(body_number=body_number, attachment_point=att)

        return body, attachment_points


    def create_attachment_point(self: Self, body_number: int, attachment_point: AttachmentPoint) -> exu.MarkerIndex:
        """Create an exudyn marker based on attachment_point."""
        return self.mbs.AddMarker(
            exu.utilities.MarkerBodyRigid(
                name = attachment_point.id,
                bodyNumber = body_number,
                localPosition = attachment_point.position_local.to("m").magnitude,
                visualization = exu.utilities.VMarkerBodyRigid(),
            ),
        )


    def create_sling(self: Self, sling: Sling, representative_mass: float) -> exu.ObjectIndex:
        """Create an exudyn sling from sling."""
        ea = sling.ea.to("N").magnitude
        d = sling.diameter.to("m").magnitude
        l_ultimate = sling.l_ultimate.to("m").magnitude

        color = graphics.color.lawngreen

        damping_fac = self.compute_rope_damping(ea, l_ultimate, representative_mass.to("kg").magnitude)

        # Prepare list of marker numbers
        markers = [
            self.mbs.GetMarkerNumber(sling.end_a.id),
            self.mbs.GetMarkerNumber(sling.end_b.id)
        ]

        # Dummy data
        sheave_axes = exu.Vector3DList()
        r_roll_arm = []
        for _ in markers:
            r_roll_arm.append(0)
            sheave_axes.Append([1, 0, 0])

        sling = self.mbs.AddObject(
            exu.utilities.ReevingSystemSprings(
                name = sling.id,
                markerNumbers = markers,
                hasCoordinateMarkers = False,
                stiffnessPerLength = ea,
                dampingPerLength = damping_fac * ea,
                referenceLength = l_ultimate,
                dampingTorsional = 0.0,
                dampingShear = 0.0,
                sheavesAxes = sheave_axes,
                sheavesRadii = r_roll_arm,
                visualization = exu.utilities.VReevingSystemSprings(
                    ropeRadius = d/2,
                    color = color
                ),
            ),
        )

        return sling


    def create_graphics(self: Self, cog: np.array, visual: dict) -> exu.graphics:
        """Create exudyn graphics from visual."""
        gr = None
        if isinstance(visual, BoxVisual):
            gr = graphics.Brick(
                centerPoint = cog + visual["offset"].to("m").magnitude,
                size = visual["size"].to("m").magnitude,
                addNormals = False,
                addEdges = True,
                addFaces = False,
                roundness = 0,
                nTiles = 12,
            )

        if isinstance(visual, CylinderVisual):
            if visual.get("axis") == "x":
                ax = [1, 0, 0]
            elif visual.get("axis") == "y":
                ax = [0, 1, 0]
            else:
                ax = [0, 0, 1]

            p1 = np.array(ax) * visual.get("length")/2
            p2 = -p1

            gr = graphics.Tube(
                points = [p1, p2],
                axes = [ax, ax],
                radius = visual["diameter"]/2,
                nTiles = 16
            )

        if isinstance(visual, MeshVisual):
            gr = graphics.FromSTLfile(
                fileName = visual.file,
                color = graphics.color.steelblue,
                density = 0.0,
                Aoff = visual.rotation,
                pOff = visual.translation.to("m").magnitude + cog,
                scale = visual.scale,
            )

        return gr


    def compute_rope_damping(self: Self, ea: float, L0: float, mass: float, safety_factor: float=0.8) -> float:
        """Compute near-critical damping factor for ReevingSystemSprings.

        Returns damping_rope_fac such that:
            dampingPerLength = fac * EA

        safety_factor:
            <1 → underdamped (faster)
            =1 → critical
            >1 → overdamped (slower but stable)
        """
        c_crit = 2 * np.sqrt(ea/L0 * mass)   # Ns/m

        # derive factor relative to EA:
        damping_rope_fac = safety_factor * (c_crit / (ea/L0))

        return damping_rope_fac


    def create_constraint(self: Self, constraint: Constraint) -> exu.ObjectIndex:
        """Create an exudyn constraint from constraint."""
        if isinstance(constraint, PinConstraint):
            return self.create_pin_constraint(constraint)
        else:
            return self.create_generic_constraint(constraint)


    def create_pin_constraint(self: Self, constraint: PinConstraint) -> exu.ObjectIndex:
        """Create an exudyn pin constraint from constraint."""
        def get_body(ap: AttachmentPoint) -> exu.ObjectIndex:
            if isinstance(ap, World):
                return self.ground
            return self.mbs.GetObjectNumber(ap.parent.id)

        ap1 = constraint.ap1
        ap2 = constraint.ap2

        body1 = get_body(ap1)
        body2 = get_body(ap2)

        # --- position (global) ---
        p1 = ap1.global_position().to("m").magnitude
        p2 = ap2.global_position().to("m").magnitude

        # robust single position
        p_joint = 0.5 * (p1 + p2)

        # --- axis selection ---
        axis1 = ap1.global_axis() if ap1.axis_local is not None else None
        axis2 = ap2.global_axis() if ap2.axis_local is not None else None

        if axis1 is not None and axis2 is not None:
            a1 = axis1 / np.linalg.norm(axis1)
            a2 = axis2 / np.linalg.norm(axis2)

            dot = np.dot(a1, a2)
            if abs(dot) < 0.999:
                raise ValueError(
                    f"PinConstraint {constraint.id}: axes not aligned (dot={dot})"
                )

            axis = a1  # deterministic

        elif axis1 is not None:
            axis = axis1 / np.linalg.norm(axis1)

        elif axis2 is not None:
            axis = axis2 / np.linalg.norm(axis2)

        else:
            raise ValueError(
                f"PinConstraint {constraint.id}: no axis defined"
            )

        return self.mbs.CreateRevoluteJoint(
            name = constraint.id,
            bodyNumbers = [body1, body2],
            position = p_joint,
            axis = axis,
            useGlobalFrame = True,
        )


    def create_generic_constraint(self: Self, constraint: Constraint) -> exu.ObjectIndex:
        """Greate an exudyn constraint from constraint."""
        def create_marker(ground: int, parent: str, ap: AttachmentPoint) -> exu.Marker:
            return self.mbs.AddMarker(
                exu.utilities.MarkerBodyRigid(
                    name = parent + "." + ap.id,
                    bodyNumber = ground,
                    localPosition = ap.global_position().to("m").magnitude,
                    visualization = exu.utilities.VMarkerBodyRigid(),
                ),
            )

        ap1 = constraint.ap1
        ap2 = constraint.ap2

        marker_numbers = []
        this_marker = [ap1, ap2]
        other_marker = [ap2, ap1]
        for this_ap, other_ap in zip(this_marker, other_marker, strict=True):
            if isinstance(this_ap, World):
                m = create_marker(self.ground, ap1.id, other_ap)
            else:
                m = self.mbs.GetMarkerNumber(this_ap.id)

            marker_numbers.append(m)

        return self.mbs.AddObject(
            exu.utilities.GenericJoint(
                name = constraint.id,
                markerNumbers = marker_numbers,
                constrainedAxes = constraint.constraints,
                visualization = exu.utilities.VGenericJoint(
                    show = True,
                    axesRadius = 0.2,
                    axesLength = 0.2,
               )
            )
        )


    def create_damping(self: Self, problem: LiftProblem) -> None:
        """Add damping to quell body movements."""
        # For each body, add a damper between body and ground
        obj = problem.bodies | problem.shackles
        for body in obj.values():
            # Get body number from name
            b = self.mbs.GetObjectNumber(body.id)

            # Get local and global position of CoG
            o = self.mbs.GetObject(b)
            cog_local = o["physicsCenterOfMass"]
            cog_global = self.get_global_position(b, cog_local)

            # Create a damper
            stiffness = 0
            self.mbs.CreateSpringDamper(
                bodyNumbers = [self.ground, b],
                localPosition0 = cog_global,
                localPosition1 = cog_local,
                stiffness = stiffness,
                damping = 5e4,
                show = True,
                drawSize = 0.5,
            )


    def get_global_position(self: Self, body: exu.ObjectIndex, local_position: np.array) -> np.array:
        """Return the global position of local_position on body 'body'."""
        return self.mbs.GetObjectOutputBody(
            body,
            exu.OutputVariableType.Position,
            localPosition = local_position,
            configuration = exu.ConfigurationType.Reference,
        )
