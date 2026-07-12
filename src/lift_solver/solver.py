import logging

import numpy as np

import exudyn as exu
from exudyn import graphics

from .shackle import Shackle
from .rigid_body import RigidBody
from .sling import Sling
from .attachment_point import AttachmentPoint
from .constraint import World, PinConstraint
from .visual_geometry import BoxVisual, CylinderVisual, MeshVisual

from . import ureg
# Exudyn units: SI
# Forces in N
# Mass in kg
# Lengths in m

prb = None
logger = logging.getLogger(__name__)
STEP_INTERVAL = 10
LOG_INTERVAL = 50


def solve(problem, simulation_duration, time_step):
    global SC, mbs, prb

    prb = problem

    # Set up mbs
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    # Create world reference system
    ground = create_ground(mbs)

    # Create model
    setup_from_problem(ground, problem)

    # Add damping
    create_damping(mbs, ground, problem)

    # Add sensors
    create_sensors(mbs, problem)

    # Solve
    run_solver(mbs, simulation_duration=simulation_duration, time_step=time_step)
#    run_solver_to_equillibrium(mbs, simulation_duration=simulation_duration, time_step=time_step)

    # Return results
    results = get_results(mbs, problem)
    print(results)
#    get_sensor_results(mbs, problem)

    # Export poses (position, orientation)
    state = export_initial_state(mbs, problem)
    print(state)

    max_velocity = compute_residuals(mbs, problem.bodies.values())
    print(f"Max residual velocity: {max_velocity}")

    # For debug purposes
#    mbs.Assemble()
#    SC.renderer.Start()
#    SC.renderer.DoIdleTasks()


def setup_from_problem(ground, problem):
    # Set up environment
    g = problem.g.to("m/s/s").magnitude

    # Need a representative mass to tune damping coefficients
    representative_mass = (max([b.mass for b in problem.bodies.values() if isinstance(b, RigidBody)]))

    # Create and place the bodies
    for o in problem.bodies.values():
        create_body(mbs, g, o)

    # Create and place the shackles
    for o in problem.shackles.values():
        create_body(mbs, g, o)

    # Create slings between attachment points
    for sl in problem.rigging.values():
        if isinstance(sl, Sling):
            create_sling(mbs, g, sl, representative_mass)

    # Apply constraints
    for constraint in problem.connections.values():
        create_constraint(mbs, ground, constraint)


def create_body(mbs, g: np.array, body: RigidBody):
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
    graphics_data_list.append(create_graphics(cog, body.visual))
    graphics_data_list.append(graphics.Basis(inertia.COM(), length=0.5))

    # Create the body
    body_number = mbs.CreateRigidBody(
        name = body.id,
        inertia = inertia,
        gravity = g,
        referencePosition = position,
        referenceRotationMatrix = orientation,
        graphicsDataList = graphics_data_list,
    )

    for att in body.attachment_points.values():
        create_attachment_point(body_number=body_number, attachment_point=att)


def create_sling(mbs, g, sling, representative_mass):
    ea = sling.ea.to("N").magnitude
    d = sling.diameter.to("m").magnitude
    l_ultimate = sling.l_ultimate.to("m").magnitude

    color = graphics.color.lawngreen

    damping_fac = compute_rope_damping(ea, l_ultimate, representative_mass.to("kg").magnitude)

    # Prepare list of marker numbers
    markers = [
        mbs.GetMarkerNumber(sling.end_a.id),
        mbs.GetMarkerNumber(sling.end_b.id)
    ]

    sheave_axes = exu.Vector3DList()
    r_roll_arm = []

    # dummy data
    for marker in markers:
        r_roll_arm.append(0)
        sheave_axes.Append([1, 0, 0])

    mbs.AddObject(
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


def create_attachment_point(body_number: int, attachment_point: AttachmentPoint):
    return mbs.AddMarker(
        exu.utilities.MarkerBodyRigid(
            name = attachment_point.id,
            bodyNumber = body_number,
            localPosition = attachment_point.position_local.to("m").magnitude,
            visualization = exu.utilities.VMarkerBodyRigid(),
        ),
    )


def create_graphics(cog: np.array, visual: dict):
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


def compute_rope_damping(EA, L0, mass, safety_factor=0.8):
    """
    Compute near-critical damping factor for ReevingSystemSprings.

    Returns damping_rope_fac such that:
        dampingPerLength = fac * EA

    safety_factor:
        <1 → underdamped (faster)
        =1 → critical
        >1 → overdamped (slower but stable)
    """
    c_crit = 2 * np.sqrt(EA/L0 * mass)   # Ns/m

    # derive factor relative to EA:
    damping_rope_fac = safety_factor * (c_crit / (EA/L0))

    return damping_rope_fac


def create_ground(mbs):
    g_ground = graphics.CheckerBoard(point=[0,0,0], normal = [0,0,1], size=60, nTiles=12)
    ground = mbs.AddObject(
        exu.utilities.ObjectGround(
            visualization=exu.utilities.VObjectGround(
                graphicsData=[g_ground]
            )
        )
    )
    return ground


def create_constraint(mbs, ground, constraint):
    if isinstance(constraint, PinConstraint):
        return create_pin_constraint(mbs, ground, constraint)
    else:
        return create_generic_constraint(mbs, ground, constraint)


def create_pin_constraint(mbs, ground, constraint):
    def get_body(ap):
        if isinstance(ap, World):
            return ground
        return mbs.GetObjectNumber(ap.parent.id)

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

    return mbs.CreateRevoluteJoint(
        name = constraint.id,
        bodyNumbers = [body1, body2],
        position = p_joint,
        axis = axis,
        useGlobalFrame = True,
    )


def create_generic_constraint(mbs, ground, constraint):
    def create_marker(ground: int, parent: str, ap: AttachmentPoint):
        return mbs.AddMarker(
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
    for this_ap, other_ap in zip(this_marker, other_marker):
        if isinstance(this_ap, World):
            m = create_marker(ground, ap1.id, other_ap)
        else:
            m = mbs.GetMarkerNumber(this_ap.id)

        marker_numbers.append(m)

    mbs.AddObject(
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


def create_damping(mbs, ground, problem):
    """Add damping to quell body movements."""

    # For each body, add a damper between body and ground
    obj = problem.bodies | problem.shackles
    for body in obj.values():
        # Get body number from name
        b = mbs.GetObjectNumber(body.id)

        # Get local and global position of CoG
        o = mbs.GetObject(b)
        cog_local = o["physicsCenterOfMass"]
        cog_global = get_global_position(mbs, b, cog_local)

        # Create a damper
        mbs.CreateSpringDamper(
            bodyNumbers = [ground, b],
            localPosition0 = cog_global,
            localPosition1 = cog_local,
            stiffness = 0.,
            damping = 5e4,
            show = True,
            drawSize = 0.5,
        )


def get_global_position(mbs, body, local_position):
    """Return the global position of local_position on body 'body'."""

    return mbs.GetObjectOutputBody(
        body,
        exu.OutputVariableType.Position,
        localPosition = local_position,
        configuration = exu.ConfigurationType.Reference,
    )


def create_sensors(mbs, problem):
    """Specify sensors."""

    for body in problem.bodies.values():
        b = mbs.GetObjectNumber(body.id)
        o = mbs.GetObject(b)
        id = body.id

        mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".position",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Position,
            )
        )

        mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".displacement",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Displacement,
            )
        )

        mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".rotation",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Rotation,
            )
        )

        mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".velocity",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Velocity,
            )
        )

        mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".acceleration",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Acceleration,
            )
        )

        mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".rotationMatrix",
                bodyNumber = b,
                outputVariableType = exu.OutputVariableType.RotationMatrix,
                storeInternal = True,
            )
        )

    for sling in problem.rigging.values():
        id = sling.id
        s = mbs.GetObjectNumber(id)
        mbs.AddSensor(
            exu.utilities.SensorObject(
                name = id + ".force",
                objectNumber=s,
                storeInternal=True,
                outputVariableType=exu.OutputVariableType.ForceLocal,
            )
        )


def compute_residuals(mbs, bodies):
    """Compute residual equilibrium errors using the remaining inertia at the end of the simulation."""
    max_velocity = 0.0

    for body in bodies:
        # Get named body
        b = mbs.GetObjectNumber(body.id)
        o = mbs.GetObject(b)
        n = o["nodeNumber"]

        # Get the final kinematic state
        vel = np.array(mbs.GetNodeOutput(n, exu.OutputVariableType.Velocity))

        # Compute the scalar magnitudes
        velocity_error = np.linalg.norm(vel)

        max_velocity = max(max_velocity, velocity_error)

    return max_velocity


def get_sensor_results(mbs, problem):
    sensors = mbs.GetDictionary()["sensorList"]

    # For now, simply print to stdout
    for sensor in sensors:
        sensor_number = mbs.GetSensorNumber(sensor["name"])
        print(f"Sensor: {sensor["name"]}, value: {mbs.GetSensorValues(sensor_number)}")

        if sensor["sensorType"] == "Body" and sensor["outputVariableType"] == exu.OutputVariableType.Displacement:
            # What is the body offset - for updating .yaml file
            b = mbs.GetObject(sensor["bodyNumber"])
            bdy = problem.bodies[b["name"]]
            ref = bdy.position
            print(f"Sensor: {sensor["name"]}, position after simulation: {ref+mbs.GetSensorValues(sensor_number) * ureg("m")}")

        if sensor["sensorType"] == "Body" and sensor["outputVariableType"] == exu.OutputVariableType.RotationMatrix:
            # tilt about X → rotation causing Z-axis to move in Y-Z plane
            # tilt about Y → rotation causing Z-axis to move in X-Z plane
            R = mbs.GetSensorValues(sensor_number).reshape(3, 3)
            z_body = R[:,2]

            tilt_x = np.arctan2(z_body[1], z_body[2])
            tilt_y = np.arctan2(-z_body[0], z_body[2])

            tilt_x_deg = np.degrees(tilt_x)
            tilt_y_deg = np.degrees(tilt_y)

            tilt_x_pct = np.tan(tilt_x) * 100
            tilt_y_pct = np.tan(tilt_y) * 100

            print(f"Body tilt in degrees: rx: {tilt_x_deg}, ry: {tilt_y_deg}")
            print(f"Body tilt in %: rx: {tilt_x_pct}, ry: {tilt_y_pct}")

        if sensor["sensorType"] == "Object" and sensor["outputVariableType"] == exu.OutputVariableType.ForceLocal:
            k_skl = 1.0
            gamma_h = 1.3       # lifting factor
            gamma_c = 1.3       # consequence factor
            gamma_s = 1.0       # termination factor
            gamma_b = 1.0       # bending factor
            gamma_w = 1.0       # wear factor
            gamma_m = 2.0       # material factor
            gamma_r = max(gamma_s, gamma_b)
            safety_factor = max(gamma_h * gamma_c * gamma_r * gamma_w * gamma_m, 2.3 * gamma_r * gamma_w)
            DAF = 1.25
            print(f"Sensor: {sensor["name"]}, converted to t and including DAF={DAF}, k_skl={k_skl}, SF={safety_factor}: {mbs.GetSensorValues(sensor_number)/9.81/1000 * DAF * k_skl * safety_factor}")


def post_step_user_function(mbs, t):
    step = mbs.sys.get("step", 0) + 1
    mbs.sys["step"] = step

    # compute residual occasionally
    if step % STEP_INTERVAL == 0:
        v_res = compute_residuals(mbs, prb.bodies.values())
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


def run_solver(mbs, simulation_duration, time_step):
    mbs.Assemble()

    # Get default simulation settings
    ss = exu.SimulationSettings()

    # Explicitly set parameters
    ss.timeIntegration.numberOfSteps = int(simulation_duration/time_step)
    ss.timeIntegration.endTime = simulation_duration
    ss.timeIntegration.generalizedAlpha.spectralRadius = 0.1
    ss.timeIntegration.verboseMode = 1
    ss.timeIntegration.newton.useModifiedNewton = False

    ss.solutionSettings.sensorsWritePeriod = 0.02

    ss.parallel.numberOfThreads = 4

    SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
    SC.visualizationSettings.nodes.show = True
    SC.visualizationSettings.nodes.drawNodesAsPoint  = False
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.nodes.basisSize = 0.2

    SC.visualizationSettings.openGL.multiSampling = 4
    SC.visualizationSettings.openGL.shadow = 0.3*0
    SC.visualizationSettings.openGL.light0position = [-50,200,100,0]

    SC.visualizationSettings.window.renderWindowSize=[1920,1200]

    ss.displayComputationTime = True

    ## start renderer and dynamic simulation
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

    mbs.SetPostStepUserFunction(post_step_user_function)

    mbs.SolveDynamic(
        simulationSettings = ss,
        updateInitialValues = True,
        showHints = False,
    )

    mbs.SolutionViewer()
    SC.renderer.Stop()


def export_initial_state(mbs, problem):
    """
    Export solver state into YAML-ready initial_state block.

    Rules:
    - parent=None  → export absolute pose
    - parent!=None → export pose relative to parent
    """

    lines = []
    lines.append("initial_state:")
    lines.append("  # format: [x, y, z, roll, pitch, yaw]")
    lines.append("  #")
    lines.append("  # IMPORTANT:")
    lines.append("  # - Bodies WITHOUT a parent are absolute (global)")
    lines.append("  # - Bodies WITH a parent are relative to their parent")
    lines.append("")

    objects = problem.bodies | problem.shackles
    for obj in objects.values():

        # --- get solver state ---
        body_number = mbs.GetObjectNumber(obj.id)
        p_global, R_global = get_body_state(mbs, body_number)

        # --- ROOT: export absolute ---
        if obj.parent is None:

            euler = rotation_matrix_to_euler(R_global)

            values = [
                f"{p_global[0]:.8g} m",
                f"{p_global[1]:.8g} m",
                f"{p_global[2]:.8g} m",
                f"{euler[0]:.8g} deg",
                f"{euler[1]:.8g} deg",
                f"{euler[2]:.8g} deg",
            ]

        # --- CHILD: export relative ---
        else:
            parent = obj.parent

            parent_body_number = mbs.GetObjectNumber(parent.id)
            p_parent, R_parent = get_body_state(mbs, parent_body_number)

            # relative transform
            p_rel = R_parent.T @ (p_global - p_parent)
            R_rel = R_parent.T @ R_global

            euler = rotation_matrix_to_euler(R_rel)

            values = [
                f"{p_rel[0]:.6g} m",
                f"{p_rel[1]:.6g} m",
                f"{p_rel[2]:.6g} m",
                f"{euler[0]:.6g} deg",
                f"{euler[1]:.6g} deg",
                f"{euler[2]:.6g} deg",
            ]

        values_str = ", ".join(values)
        lines.append(f"  {obj.id}: [{values_str}]")

    return "\n".join(lines)


def get_body_state(mbs, body_number):
    """
    Extract global position and rotation matrix from Exudyn.
    """

    obj = mbs.GetObject(body_number)
    node_number = obj["nodeNumber"]

    # position
    p = mbs.GetNodeOutput(
        node_number,
        exu.OutputVariableType.Position
    )

    # rotation matrix (flattened → reshape)
    R = np.array(
        mbs.GetNodeOutput(
            node_number,
            exu.OutputVariableType.RotationMatrix
        )
    ).reshape((3, 3))

    return p, R


def rotation_matrix_to_euler(R):
    """
    Convert rotation matrix to XYZ Euler angles (degrees).
    """
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


def solve_with_auto_stop(mbs, SC, problem):
    simulationSettings = exu.SimulationSettings()

    h = 0.002
    chunk_time = 0.5   # seconds per chunk

    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.1
    simulationSettings.timeIntegration.verboseMode = 0

    simulationSettings.parallel.numberOfThreads = 4

    # tolerances
    v_tol = 1e-3
    f_tol = 1e3
    m_tol = 1e5

    max_time = 30.0
    t = 0.0

    while t < max_time:

        steps = int(chunk_time / h)
#        simulationSettings.timeIntegration.numberOfSteps = int(chunk_time / h)
#        simulationSettings.timeIntegration.startTime = t
#        simulationSettings.timeIntegration.endTime = t + chunk_time
#        simulationSettings.timeIntegration.endTime = chunk_time
        simulationSettings.timeIntegration.reuseConstantMassMatrix = True


        t += chunk_time
        simulationSettings.timeIntegration.numberOfSteps = int(t / h)
        simulationSettings.timeIntegration.endTime = t
        mbs.SolveDynamic(simulationSettings)

#        mbs.systemData.SetTime(t)

        v_res = compute_residuals(mbs, problem["bodies"])

        print(f"\nTime: {t:.2f} s")
        print(f"  Velocity:        {v_res:.3e} m/s")

        # convergence check
        if v_res < v_tol and f_res < f_tol and m_res < m_tol:
            print("Converged — stopping")
            break


def plot_convergence(mbs, problem):
    import matplotlib.pyplot as plt
    logging.getLogger("matplotlib").setLevel(logging.WARNING)

    v_max = []
    t_vals = []

    sensors = mbs.GetDictionary()["sensorList"]

    vel_sensors = [
        s for s in sensors
        if s["sensorType"] == "Body"
        and s["outputVariableType"] == exu.OutputVariableType.Velocity
    ]

    # collect all velocity histories
    histories = []
    for s in vel_sensors:
        num = mbs.GetSensorNumber(s["name"])
        data = mbs.GetSensorStoredData(num)
        histories.append(data)

    # assume same time grid
    time = histories[0][:,0]

    for i in range(len(time)):
        vmax = 0.0

        for data in histories:
            vx, vy, vz = data[i,1:4]
            v = np.sqrt(vx*vx + vy*vy + vz*vz)
            vmax = max(vmax, v)

        v_max.append(vmax)
        t_vals.append(time[i])

    # plot velocity decay
    plt.figure()
    plt.semilogy(t_vals, v_max)
    plt.xlabel("Time [s]")
    plt.ylabel("Max velocity [m/s]")
    plt.title("Convergence (velocity decay)")
    plt.grid()

    plt.show()


def plot_residuals(times, res_f, res_m):
    import matplotlib.pyplot as plt

    plt.figure()
    plt.semilogy(times, res_f, label="Force residual [N]")
    plt.semilogy(times, res_m, label="Moment residual [Nm]")
    plt.xlabel("Time [s]")
    plt.ylabel("Residual")
    plt.legend()
    plt.grid()

    plt.show()

def get_results(mbs, problem):
    results = {
        "bodies": {},
        "slings": {},
        "shackles": {},
        "attachment_points": {},
    }

    # Collect sling results
    for sling in problem.rigging.values():
        results["slings"][sling.id] = get_sling_results(mbs, sling)

    # Collect body results
    for body in problem.bodies.values():
        results["bodies"][body.id] = get_body_results(mbs, body)

    # Combine forces and moments acting on attachment points
    compute_loads_on_attachment_points(problem, results)

    # Compute force and moment sums on bodies
    compute_body_force_and_moment_sums(problem, results)

    return results


def get_sling_results(mbs, sling):
    results = {}

    s = mbs.GetObjectNumber(sling.id)
    results["sling_tension"] = mbs.GetObjectOutput(
        objectNumber = s,
        variableType = exu.OutputVariableType.ForceLocal # * ureg("N"),
    )

    ba = mbs.GetObjectNumber(sling.end_a.parent.id)
    end_a_global_position = get_global_position(mbs, ba, sling.end_a.position_local.to("m").magnitude)

    bb = mbs.GetObjectNumber(sling.end_b.parent.id)
    end_b_global_position = get_global_position(mbs, bb, sling.end_b.position_local.to("m").magnitude)

    vector = end_b_global_position - end_a_global_position
    unit_vector = vector / np.linalg.norm(vector)

    # forces at end_a of sling - tension will be negative
    results["sling_force_components"] = results["sling_tension"] * unit_vector

    return results


def get_body_results(mbs, body):
    results = {}

    b = mbs.GetObjectNumber(body.id)
    o = mbs.GetObject(b)

    # Position
    results["position"] = mbs.GetObjectOutputBody(
        objectNumber = b,
        variableType = exu.OutputVariableType.Position,
        localPosition = o["physicsCenterOfMass"],
        configuration = exu.ConfigurationType.Current,
    ) * ureg("m")

    # Tilt x and y
    rotation_matrix = mbs.GetObjectOutputBody(
        objectNumber = b,
        variableType = exu.OutputVariableType.RotationMatrix,
        localPosition = o["physicsCenterOfMass"],
        configuration = exu.ConfigurationType.Current,
    )

    R = np.array(rotation_matrix).reshape((3, 3))
    results["tilt_x"] = np.degrees(np.arcsin(R[2,0])) * ureg("degrees")
    results["tilt_y"] = np.degrees(np.arcsin(R[2,1])) * ureg("degrees")

    return results


def compute_loads_on_attachment_points(problem, results):
    def add_sling_force_global(attachment_point, sling, results, fac):
        sling_results = results["slings"][sling.id]
        ap_id = attachment_point.id

        ap = results["attachment_points"].get(ap_id)
        if ap is None:
            results["attachment_points"][ap_id] = {}
            results["attachment_points"][ap_id]["force_global"] = np.array([0.0, 0.0, 0.0])
            results["attachment_points"][ap_id]["moment_global"] = np.array([0.0, 0.0, 0.0])

        results["attachment_points"][ap_id]["force_global"] += (sling_results["sling_force_components"] * fac)

    def add_connection_force_global(attachment_point, force_global, moment_global, results, fac):
        ap_id = attachment_point.id

        ap = results["attachment_points"].get(ap_id)
        if ap is None:
            results["attachment_points"][ap_id] = {}
            results["attachment_points"][ap_id]["force_global"] = np.array([0.0, 0.0, 0.0])
            results["attachment_points"][ap_id]["moment_global"] = np.array([0.0, 0.0, 0.0])

        results["attachment_points"][ap_id]["force_global"] += force_global * fac
        results["attachment_points"][ap_id]["moment_global"] += moment_global * fac


    # slings may be attached directly to bodies
    for sling in problem.rigging.values():
        add_sling_force_global(attachment_point=sling.end_a, sling=sling, results=results, fac=1)
        add_sling_force_global(attachment_point=sling.end_b, sling=sling, results=results, fac=-1)

    # connections connect bodies to bodies, and bodies to ground
    for connection in problem.connections.values():
        n = mbs.GetObjectNumber(connection.id)
        o = mbs.GetObject(n)

        force_local = mbs.GetObjectOutput(
            objectNumber = n,
            variableType = exu.OutputVariableType.ForceLocal, # * ureg("N"),
            configuration = exu.ConfigurationType.Current,
        )

        moment_local = mbs.GetObjectOutput(
            objectNumber = n,
            variableType = exu.OutputVariableType.TorqueLocal, # * ureg("N"),
            configuration = exu.ConfigurationType.Current,
        )

        m0 = mbs.GetMarker(o["markerNumbers"][0])
        R0 = mbs.GetObjectOutputBody(
            objectNumber = m0["bodyNumber"],
            variableType = exu.OutputVariableType.RotationMatrix,
            configuration = exu.ConfigurationType.Current,
        )

        force_global = R0.reshape(3,3) @ o["rotationMarker0"] @ force_local
        moment_global = R0.reshape(3,3) @ o["rotationMarker0"] @ moment_local

        # Documentation not crystal clear re sign convention for moments, however simple
        # tests showed this was the only way to get both force and moment equilibrium
        moment_global *= -1

        add_connection_force_global(attachment_point=connection.ap1, force_global=force_global,
                                    moment_global=moment_global, results=results, fac=1)

        add_connection_force_global(attachment_point=connection.ap2, force_global=force_global,
                                    moment_global=moment_global, results=results, fac=-1)


def compute_body_force_and_moment_sums(problem, results):
    for body in problem.bodies.values():
        f_sum = body.mass.to("kg").magnitude * problem.g.to("m/s/s").magnitude
        m_sum = 0.0

        # Get local and global position of CoG
        b = mbs.GetObjectNumber(body.id)
        o = mbs.GetObject(b)
        cog_local = o["physicsCenterOfMass"]
        cog_global = get_global_position(mbs, b, cog_local)

        for ap in body.attachment_points.values():
            ap_res = results["attachment_points"].get(ap.id)
            if ap_res:
                ap_global_position = get_global_position(mbs, b, ap.position_local.to("m").magnitude)

                f = results["attachment_points"][ap.id]["force_global"]
                m = results["attachment_points"][ap.id]["moment_global"]
                f_sum += f

                r = ap_global_position - cog_global
                m_sum += np.cross(r, f) + m

        results["bodies"][body.id]["f_sum"] = f_sum
        results["bodies"][body.id]["m_sum"] = m_sum
        results["bodies"][body.id]["f_residual"] = np.linalg.norm(f_sum)
        results["bodies"][body.id]["m_residual"] = np.linalg.norm(m_sum)
