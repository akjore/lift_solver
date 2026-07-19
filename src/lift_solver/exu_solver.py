"""Class for solving and extracting results for an exudyn simulation."""
import logging
from typing import Self

import exudyn as exu
import numpy as np

from . import ureg
from .attachment_point import AttachmentPoint
from .constraint import Constraint
from .exu_problem import ExuProblem
from .results import Results
from .rigid_body import RigidBody
from .shackle import Shackle
from .sling import Sling

# Exudyn units: SI
# Forces in N
# Mass in kg
# Lengths in m

logger = logging.getLogger(__name__)
STEP_INTERVAL = 10
LOG_INTERVAL = 50

class ExuSolver:
    """Class to solve an exudyn version of a single LiftProblem."""

    def __init__(self: Self, exu_problem: ExuProblem) -> None:
        """Initialize object."""
        self.exu_problem = exu_problem

       # Set up mbs
        self.SC = exu_problem.SC
        self.mbs = exu_problem.mbs
        self.problem = exu_problem.problem
        self.g = exu_problem.g


    def solve(self: Self, simulation_duration: float, time_step: float) -> None:
        """Solve the dynamic problem."""
        self.mbs.Assemble()

        # Get default simulation settings
        ss = exu.SimulationSettings()

        # Explicitly set parameters
        ss.timeIntegration.numberOfSteps = int(simulation_duration/time_step)
        ss.timeIntegration.endTime = simulation_duration
        ss.timeIntegration.generalizedAlpha.spectralRadius = 0.1
        ss.timeIntegration.verboseMode = 1
        ss.timeIntegration.newton.useModifiedNewton = False

        # temp - reduced to help with additional free DoFs at hook
        ss.timeIntegration.newton.relativeTolerance = 1E-4
        ss.timeIntegration.newton.absoluteTolerance = 1E-6
        # end temp

        ss.solutionSettings.sensorsWritePeriod = 0.02

        ss.parallel.numberOfThreads = 4

        self.SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
        self.SC.visualizationSettings.nodes.show = True
        self.SC.visualizationSettings.nodes.drawNodesAsPoint  = False
        self.SC.visualizationSettings.nodes.showBasis = True
        self.SC.visualizationSettings.nodes.basisSize = 0.2

        self.SC.visualizationSettings.openGL.multiSampling = 4
        self.SC.visualizationSettings.openGL.shadow = 0.3*0
        self.SC.visualizationSettings.openGL.light0position = [-50,200,100,0]

        self.SC.visualizationSettings.window.renderWindowSize=[1920,1200]

        ss.displayComputationTime = True

        ## start renderer and dynamic simulation
        self.SC.renderer.Start()
        self.SC.renderer.DoIdleTasks()

        self.mbs.SetPostStepUserFunction(self.post_step_user_function)

        # temp
        print(self.mbs.ComputeSystemDegreeOfFreedom())
        # ss.timeIntegration.verboseMode = 2
#        ss.linearSolverType = exu.LinearSolverType.EigenSparse
#        ss.linearSolverSettings.ignoreSingularJacobian=True
        # end temp

        self.mbs.SolveDynamic(
            simulationSettings = ss,
            updateInitialValues = True,
            showHints = False,
        )

        self.mbs.SolutionViewer()
        self.SC.renderer.Stop()


    def post_step_user_function(self: Self, mbs: exu.MultiBodySystem, t: int) -> bool:
        """After each simulation step, check velocities.

        Tune damping, and indicate whether or not target has been achieved.
        """
        step = mbs.sys.get("step", 0) + 1
        mbs.sys["step"] = step

        # compute residual occasionally
        if step % STEP_INTERVAL == 0:
            v_res = self.compute_residuals(self.problem.bodies.values())
            mbs.sys["v_res"] = v_res
        else:
            v_res = mbs.sys.get("v_res", 1e6)

        # adaptive damping
        if v_res > 0.5:
            factor = 0.5
        elif v_res > 0.05:
            factor = 0.2
        elif v_res > 0.005:
            factor = 0.1
        else:
            factor = 0.0

        if factor > 0:
            coords_t = mbs.systemData.GetODE2Coordinates_t()
            coords_t *= (1 - factor)
            mbs.systemData.SetODE2Coordinates_t(coords_t)

        equilibrium = v_res < 1e-2

        return not equilibrium


    def compute_residuals(self: Self, bodies: list) -> float:
        """Compute residual equilibrium errors using the remaining inertia at the end of the simulation."""
        max_velocity = 0.0

        for body in bodies:
            # Get named body
            b = self.mbs.GetObjectNumber(body.id)
            o = self.mbs.GetObject(b)
            n = o["nodeNumber"]

            # Get the final kinematic state
            vel = np.array(self.mbs.GetNodeOutput(n, exu.OutputVariableType.Velocity))

            # Compute the scalar magnitudes
            velocity_error = np.linalg.norm(vel)

            max_velocity = max(max_velocity, velocity_error)

        return max_velocity


    def get_results(self: Self) -> Results:
        """Get all results for all object types."""
        results = Results()

        # Exudyn has no easy way of extracting forces acting on bodies from slings or in constraints.
        # Need to loop over slings and constraints and add in contributions
        # Prepare holder for attachment_point forces / moments
        world = AttachmentPoint("world", parent=None, position_local = [0, 0, 0])
        for attachment_point in (self.problem.attachment_points | {"world": world}).values():
            results.attachment_points[attachment_point.id] = {
                "force_global": np.array([0.0, 0.0, 0.0]) * ureg("N"),
                "moment_global": np.array([0.0, 0.0, 0.0]) * ureg("N*m"),
            }

        # Collect constraint results
        for connection in self.problem.connections.values():
            res = self.get_constraint_results(connection)

            results.constraints[connection.id] = res

            # Apply constraint forces to relevant attachment points
            results.attachment_points[connection.ap1.id]["force_global"] += res["force_global"]
            results.attachment_points[connection.ap1.id]["moment_global"] += res["moment_global"]

            results.attachment_points[connection.ap2.id]["force_global"] += -res["force_global"]
            results.attachment_points[connection.ap2.id]["moment_global"] += -res["moment_global"]


        # Collect sling results - this only works for slings as weight-less springs between 2 points
        for sling in self.problem.slings.values():
            # Get sling results
            results.slings[sling.id] = self.get_sling_results(sling)

            # Apply sling forces to relevant attachment points
            results.attachment_points[sling.end_a.id]["force_global"] += results.slings[sling.id]["force_global"]
            results.attachment_points[sling.end_b.id]["force_global"] += -results.slings[sling.id]["force_global"]


        # Collect body results
        for body in self.problem.bodies.values():
            results.bodies[body.id] = self.get_body_results(body, results)


        # Collect shackle results
        for shackle in self.problem.shackles.values():
            results.shackles[shackle.id] = self.get_shackle_results(shackle, results)

        return results


    def get_body_state(self: Self, body_number: exu.BodyIndex) -> tuple:
        """Extract global position and rotation matrix from Exudyn."""
        obj = self.mbs.GetObject(body_number)
        node_number = obj["nodeNumber"]

        # position
        p = self.mbs.GetNodeOutput(
            node_number,
            exu.OutputVariableType.Position
        )

        # rotation matrix (flattened → reshape)
        R = np.array(
            self.mbs.GetNodeOutput(
                node_number,
                exu.OutputVariableType.RotationMatrix
            )
        ).reshape((3, 3))

        return p, R


    def rotation_matrix_to_euler(self: Self, R: np.array) -> np.array(3):
        """Convert rotation matrix to XYZ Euler angles (degrees)."""
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-8

        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0

        return np.degrees([x, y, z])


    def get_sling_results(self: Self, sling: Sling) -> dict:
        """Get results for a sling."""
        s = self.mbs.GetObjectNumber(sling.id)
        sling_tension = self.mbs.GetObjectOutput(
            objectNumber = s,
            variableType = exu.OutputVariableType.ForceLocal
        )

        ba = self.mbs.GetObjectNumber(sling.end_a.parent.id)
        end_a_global_position = self.get_global_position(ba, sling.end_a.position_local.to("m").magnitude)

        bb = self.mbs.GetObjectNumber(sling.end_b.parent.id)
        end_b_global_position = self.get_global_position(bb, sling.end_b.position_local.to("m").magnitude)

        vector = end_b_global_position - end_a_global_position
        unit_vector = vector / np.linalg.norm(vector)

        # forces at end_a of sling - tension will be negative
        force_global = sling_tension * unit_vector

        return {
            "id": sling.id,
            "tension": sling_tension * ureg("N"),
            "force_global": force_global * ureg("N"),
        }


    def get_body_results(self: Self, body: RigidBody, results: Results) -> dict:
        """Get results for a body."""
        body_number = self.mbs.GetObjectNumber(body.id)
        o = self.mbs.GetObject(body_number)

        # Position
        position = self.mbs.GetObjectOutputBody(
            objectNumber = body_number,
            variableType = exu.OutputVariableType.Position,
            localPosition = o["physicsCenterOfMass"],
            configuration = exu.ConfigurationType.Current,
        )

        # Tilt x and y
        rotation_matrix = self.mbs.GetObjectOutputBody(
            objectNumber = body_number,
            variableType = exu.OutputVariableType.RotationMatrix,
            localPosition = o["physicsCenterOfMass"],
            configuration = exu.ConfigurationType.Current,
        )

        R = np.array(rotation_matrix).reshape((3, 3))
        tilt_x = np.degrees(np.arcsin(R[2,0]))
        tilt_y = np.degrees(np.arcsin(R[2,1]))

        # Force and moment sums
        force_moment_sums = self.compute_force_and_moment_sums(body, results)

        # Body poses
        p_global, R_global = self.get_body_state(body_number)
        euler_global = self.rotation_matrix_to_euler(R_global)

        parent = body.parent
        p_rel = None
        R_rel = None
        euler_rel = None
        if parent:
            parent_body_number = self.mbs.GetObjectNumber(parent.id)
            p_parent, R_parent = self.get_body_state(parent_body_number)

            # relative transform
            p_rel = R_parent.T @ (p_global - p_parent)
            R_rel = R_parent.T @ R_global

            euler_rel = self.rotation_matrix_to_euler(R_rel)

        return {
            "id": body.id,
            "position": position * ureg("m"),
            "tilt_x": tilt_x * ureg("deg"),
            "tilt_y": tilt_y * ureg("deg"),
            "position_global": p_global * ureg("m"),
            "rotation_matrix_global": R_global,
            "euler_global": euler_global * ureg("deg"),
            "position_relative": p_rel * ureg("m") if p_rel is not None else None,
            "rotation_matrix_relative": R_rel,
            "euler_relative": euler_rel * ureg("deg") if euler_rel is not None else None,
            **force_moment_sums,
        }


    def get_constraint_results(self: Self, connection: Constraint) -> dict:
        """Get forces and moments from constraints."""
        # connections connect bodies to bodies, and bodies to ground
        n = self.mbs.GetObjectNumber(connection.id)
        o = self.mbs.GetObject(n)

        force_local = self.mbs.GetObjectOutput(
            objectNumber = n,
            variableType = exu.OutputVariableType.ForceLocal, # * ureg("N"),
            configuration = exu.ConfigurationType.Current,
        )

        moment_local = self.mbs.GetObjectOutput(
            objectNumber = n,
            variableType = exu.OutputVariableType.TorqueLocal, # * ureg("N"),
            configuration = exu.ConfigurationType.Current,
        )

        m0 = self.mbs.GetMarker(o["markerNumbers"][0])
        R0 = self.mbs.GetObjectOutputBody(
            objectNumber = m0["bodyNumber"],
            variableType = exu.OutputVariableType.RotationMatrix,
            configuration = exu.ConfigurationType.Current,
        )

        force_global = R0.reshape(3,3) @ o["rotationMarker0"] @ force_local
        moment_global = R0.reshape(3,3) @ o["rotationMarker0"] @ moment_local

        # Documentation not crystal clear re sign convention for moments, however simple
        # tests showed this was the only way to get both force and moment equilibrium
        moment_global *= -1

        return {
            "id": connection.id,
            "force_global": force_global * ureg("N"),
            "moment_global": moment_global * ureg("N*m"),
        }


    def compute_force_and_moment_sums(self: Self, body: RigidBody, results: Results) -> dict:
        """Compute load sums on a body, considering gravity and forces from AttachmentPoints."""
        # Get local and global position of CoG
        b = self.mbs.GetObjectNumber(body.id)
        o = self.mbs.GetObject(b)
        cog_local = o["physicsCenterOfMass"]
        cog_global = self.get_global_position(b, cog_local)

        f_sum = o["physicsMass"] * self.g * ureg("N")
        m_sum = 0.0 * ureg("N*m")

        for ap in body.attachment_points.values():
            ap_global_position = self.get_global_position(b, ap.position_local.to("m").magnitude)

            f = results.attachment_points[ap.id]["force_global"]
            m = results.attachment_points[ap.id]["moment_global"]
            f_sum += f

            r = (ap_global_position - cog_global) * ureg("m")
            m_sum += np.cross(r, f) + m

        return {
            "f_sum": f_sum,
            "m_sum": m_sum,
            "f_residual": np.linalg.norm(f_sum),
            "m_residual": np.linalg.norm(m_sum),
        }


    def get_global_position(self: Self, body: exu.BodyIndex, local_position:np.array(3)) -> np.array(3):
        """Return the global position of local_position on body 'body'."""
        return self.mbs.GetObjectOutputBody(
            body,
            exu.OutputVariableType.Position,
            localPosition = local_position,
            configuration = exu.ConfigurationType.Reference,
        )


    def get_shackle_results(self: Self, shackle: Shackle, results: Results) -> dict:
        """Return a dict holding the shackle results."""
        res = self.get_body_results(shackle, results)

        return res | {
            "pin_force_global": results.attachment_points[shackle.pin.id]["force_global"],
            "pin_moment_global": results.attachment_points[shackle.pin.id]["moment_global"],
            "pin_force": np.linalg.norm(res["pin_force_global"]),
            "pin_moment": np.linalg.norm(res["pin_moment_global"]),

            "bow_force_global": results.attachment_points[shackle.bow.id]["force_global"],
            "bow_moment_global": results.attachment_points[shackle.bow.id]["moment_global"],
            "bow_force": np.linalg.norm(res["bow_force_global"]),
            "bow_moment": np.linalg.norm(res["bow_moment_global"]),
        }
