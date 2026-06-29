import logging
import math

import numpy as np

import exudyn as exu
from exudyn import graphics

from .shackle import Shackle
from .rigid_body import RigidBody
from .sling import Sling
from .attachment_point import AttachmentPoint
from .constraint import World

from . import ureg, Q_
# Exudyn units: SI
# Forces in N
# Mass in kg
# Lengths in m

prb = None
logger = logging.getLogger(__name__)


def solve(problem, simulation_duration, time_step):
    global SC, mbs, prb

    prb = problem

    # Set up mbs
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    # Create world reference system
    ground = setup_ground(mbs)

    # Create model
    setup_from_problem(ground, problem)

    # Add damping
    setup_damping(mbs, ground, problem)

    # Add sensors
    setup_sensors(mbs, problem)

    # Solve
    run_solver(mbs, simulation_duration=simulation_duration, time_step=time_step)

    # Return results
    get_sensor_results(mbs, problem)

    max_force, max_moment, max_velocity = compute_residuals(mbs, problem.objects.values())
    print(f"Max residual force: {max_force}")
    print(f"Max residual moment: {max_moment}")
    print(f"Max residual velocity: {max_velocity}")

    plot_convergence(mbs, problem)


#    times, rf, rm = compute_residual_history(mbs, problem)
#    plot_residuals(times, rf, rm)

    # For debug purposes
#    mbs.Assemble()
#    SC.renderer.Start()
#    SC.renderer.DoIdleTasks()


def setup_from_problem(ground, problem):
    # Set up environment
    g = problem.g.to("m/s/s").magnitude

    # Need a representative mass to tune damping coefficients
    representative_mass = (max([b.mass for b in problem.objects.values() if isinstance(b, RigidBody)]))

    # Create and place the objects
    for o in problem.objects.values():
        if isinstance(o, RigidBody):
            setup_body(mbs, g, o)
        elif isinstance(o, Shackle):
            setup_body(mbs, g, o)


            p_pin = o.pin.global_position()
            axis = o.pin.global_axis()

            p_bow = o.bow.global_position()

            # distance from axis:
            d = np.linalg.norm(np.cross(p_bow - p_pin, axis))

            print(o.id, d)


        else:
            print(f"Skipped {o}")

    for sl in problem.rigging.values():
        if isinstance(sl, Sling):
            setup_sling(mbs, g, sl, representative_mass)

    # Apply constraints
    for constraint in problem.connections.values():
        setup_constraint(mbs, ground, constraint)


def setup_body(mbs, g: np.array, body: RigidBody):
    mass = body.mass.to("kg").magnitude
    cog = body.cog.to("m").magnitude

    position = body.global_position().to("m").magnitude
    orientation = body.global_rotation()

#    length = 100
#    width = 100
#    height = 100
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

    if isinstance(body, Shackle):
        graphics_data_list.append(
            graphics.FromSTLfile(
                fileName = body.mesh.file,
                color = graphics.color.steelblue,
                density = 0.0,
                Aoff = body.mesh.rotation,
                pOff = body.mesh.translation.to("m").magnitude,
                scale = body.mesh.scale.magnitude,
            )
        )

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
        m = setup_attachment_point(body_number=body_number, attachment_point=att)


def setup_sling(mbs, g, sling, representative_mass):
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

    sl = mbs.AddObject(
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


def setup_attachment_point(body_number: int, attachment_point: AttachmentPoint):
    m = mbs.AddMarker(
        exu.utilities.MarkerBodyRigid(
            name = attachment_point.id,
            bodyNumber = body_number,
            localPosition = attachment_point.position_local.to("m").magnitude,
            visualization = exu.utilities.VMarkerBodyRigid(),
        ),
    )
    return m


def create_graphics(cog: np.array, visual: dict):
    gr = None
    if visual:
        if visual.get("type") == "box":
            gr = graphics.Brick(
                centerPoint = cog + visual["offset"].to("m").magnitude,
                size = visual["size"].to("m").magnitude,
                addNormals = False,
                addEdges = True,
                addFaces = False,
                roundness = 0,
                nTiles = 12,
            )

        if visual.get("type") == "cylinder":
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
    # c = (fac * EA) / L0 = fac * k
    damping_rope_fac = safety_factor * (c_crit / (EA/L0))

    return damping_rope_fac


def setup_ground(mbs):
    g_ground = graphics.CheckerBoard(point=[0,0,0], normal = [0,0,1], size=60, nTiles=12)
    ground = mbs.AddObject(
        exu.utilities.ObjectGround(
            visualization=exu.utilities.VObjectGround(
                graphicsData=[g_ground]
            )
        )
    )
    return ground


#def get_global_marker_position(mbs, marker_num):
#    # Get the marker properties; body and local position on body
#    m = mbs.GetMarker(marker_num)

#    # Return the global position of that local position
#    return mbs.GetObjectOutputBody(
#        m["bodyNumber"],
#        exu.OutputVariableType.Position,
#        localPosition = m["localPosition"],
#        configuration = exu.ConfigurationType.Reference,
#    )

#def get_marker(mbs, ground, ap):
#
#    if isinstance(ap, World):
#        return mbs.AddMarker(
#            exu.MarkerBodyRigid(
#                name = parent + "." + ap.id,
#                bodyNumber = ground,
#                localPosition = ap.global_position().to("m").magnitude
#                visualization = exu.utilities.VMarkerBodyRigid(),
#            )
#        )
#    else:
#        return mbs.GetMarkerNumber(ap.id)


def setup_constraint(mbs, ground, constraint):
    def create_marker(ground: int, parent: str, ap: AttachmentPoint):
        m = mbs.AddMarker(
            exu.utilities.MarkerBodyRigid(
                name = parent + "." + ap.id,
                bodyNumber = ground,
                localPosition = ap.global_position().to("m").magnitude,
                visualization = exu.utilities.VMarkerBodyRigid(),
            ),
        )
        return m

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

    # testing
    if (not isinstance(ap1, World)) and (not isinstance(ap2, World)) and (isinstance(ap1.parent, Shackle) or isinstance(ap2.parent, Shackle)):
       m1 = mbs.GetMarker(marker_numbers[0])
       m2 = mbs.GetMarker(marker_numbers[1])
       mbs.CreateRevoluteJoint(
            bodyNumbers = [m1["bodyNumber"], m2["bodyNumber"]],
            position = [0, 0, 0],
            axis = [1, 0, 0],
            useGlobalFrame = False,
            axisRadius = 0.35/2,
            axisLength = 1,
        )
    else:
        joint = mbs.AddObject(
            exu.utilities.GenericJoint(
                markerNumbers = marker_numbers,
                constrainedAxes = constraint.constraints,
                visualization = exu.utilities.VGenericJoint(
                    show = True,
                    axesRadius = 0.2,
                    axesLength = 0.2,
                )
            )
        )


#    if isinstance(ap1.parent, Shackle):
#        mbs.AddObject(exu.utilities.TorsionalSpringDamper(
#            markerNumbers = marker_numbers,
#            stiffness = 1.0,
#            damping = 0.0
#        ))




def setup_damping(mbs, ground, problem):
    """Add damping to quell body movements."""

    # For each body, add a damper between body and ground
    for body in problem.objects.values():
        # Get body number from name
        b = mbs.GetObjectNumber(body.id)

        # Get local and global position of CoG
        o = mbs.GetObject(b)
        cog_local = o["physicsCenterOfMass"]
        cog_global = get_global_position(mbs, b, cog_local)

        # Create a damper
        oSD = mbs.CreateSpringDamper(
            bodyNumbers = [ground, b],
            localPosition0 = cog_global,
            localPosition1 = cog_local,
            stiffness = 0.,
            damping = 5e4,
            show = True,
            drawSize = 0.5,
        )

#        # Create a damper
#        oSD = mbs.CreateSpringDamper(
#            bodyNumbers = [ground, b],
#            localPosition0 = cog_global + np.array([10, 0, 0]),
#            localPosition1 = cog_local,
#            stiffness = 0.,
#            damping = 5e4,
#            show = True,
#            drawSize = 0.5,
#        )

#        # Create a damper
#        oSD = mbs.CreateSpringDamper(
#            bodyNumbers = [ground, b],
#            localPosition0 = cog_global + np.array([0, 10, 0]),
#            localPosition1 = cog_local,
#            stiffness = 0.,
#            damping = 5e4,
#            show = True,
#            drawSize = 0.5,
#        )


def get_global_position(mbs, body, local_position):
    """Return the global position of local_position on body 'body'."""

    return mbs.GetObjectOutputBody(
        body,
        exu.OutputVariableType.Position,
        localPosition = local_position,
        configuration = exu.ConfigurationType.Reference,
    )


def setup_sensors(mbs, problem):
    """Specify sensors."""

#    for body in problem["bodies"]:
    for body in problem.objects.values():
        b = mbs.GetObjectNumber(body.id)
        o = mbs.GetObject(b)
        id = body.id

        sensor_pos = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".position",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Position,
            )
        )

        sensor_pos = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".displacement",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Displacement,
            )
        )

        sensor_pos = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".rotation",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Rotation,
            )
        )

        sensor_vel = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".velocity",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Velocity,
            )
        )

        sensor_acc = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = id + ".acceleration",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Acceleration,
            )
        )

#        sensor_rotmat = mbs.AddSensor(
#            exu.utilities.SensorBody(
#                name = id + ".rotationMatrix",
#                bodyNumber = b,
#                outputVariableType = exu.OutputVariableType.RotationMatrix,
#                storeInternal = True,
#            )
#        )

#        sensor_f = mbs.AddSensor(
#            exu.utilities.SensorBody(
#                name = body["id"] + ".force",
#                bodyNumber = b,
#                localPosition = o["physicsCenterOfMass"],
#                storeInternal = True,
#                outputVariableType = exu.OutputVariableType.Force,
#            )
#        )

    for sling in problem.rigging.values():
        id = sling.id
        s = mbs.GetObjectNumber(id)
        sensor_f = mbs.AddSensor(
            exu.utilities.SensorObject(
                name = id + ".force",
                objectNumber=s,
                storeInternal=True,
                outputVariableType=exu.OutputVariableType.ForceLocal,
            )
        )


def compute_residuals(mbs, bodies):
    """Compute residual equilibrium errors using the remaining inertia at the end of the simulation."""
    max_force = 0.0
    max_moment = 0.0
    max_velocity = 0.0

    for body in bodies:
        # Get named body
        b = mbs.GetObjectNumber(body.id)
        o = mbs.GetObject(b)
        n = o["nodeNumber"]

        # Fetch parameters
        mass = o["physicsMass"]
        Ixx, Iyy, Izz, Ixy, Iyz, Izx = o["physicsInertia"]

        # Build the symmetric 3x3 matrix about the COM
        inertia = np.array([
            [Ixx, Ixy, Izx],
            [Ixy, Iyy, Iyz],
            [Izx, Iyz, Izz],
        ])


        # Get the final kinematic state
        acc = np.array(mbs.GetNodeOutput(n, exu.OutputVariableType.Acceleration))
        vel = np.array(mbs.GetNodeOutput(n, exu.OutputVariableType.Velocity))
        ang_acc = np.array(mbs.GetNodeOutput(n, exu.OutputVariableType.AngularAcceleration))
        ang_vel = np.array(mbs.GetNodeOutput(n, exu.OutputVariableType.AngularVelocity))

        R = mbs.GetNodeOutput(n, exu.OutputVariableType.RotationMatrix)
        R = np.array(R).reshape((3,3))

        inertia_global = R @ inertia @ R.T

        # Compute residual equilibrium errors
        residual_force = mass * acc
        residual_moment = (inertia_global @ ang_acc) + np.cross(ang_vel, inertia_global @ ang_vel)

        # Compute the scalar magnitudes
        force_error = np.linalg.norm(residual_force)
        moment_error = np.linalg.norm(residual_moment)
        velocity_error = np.linalg.norm(vel)

        max_force = max(max_force, force_error)
        max_moment = max(max_moment, moment_error)
        max_velocity = max(max_velocity, velocity_error)

    return max_force, max_moment, max_velocity


def get_sensor_results(mbs, problem):
    sensors = mbs.GetDictionary()["sensorList"]

    # For now, simply print to stdout
    for sensor in sensors:
        sensor_number = mbs.GetSensorNumber(sensor["name"])
        print(f"Sensor: {sensor["name"]}, value: {mbs.GetSensorValues(sensor_number)}")

        if sensor["sensorType"] == "Body" and sensor["outputVariableType"] == exu.OutputVariableType.Displacement:
            # What is the body offset - for updating .yaml file
            b = mbs.GetObject(sensor["bodyNumber"])
            bdy = problem.objects[b["name"]]
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

            print(f"Body tilt i degrees: rx: {tilt_x_deg}, ry: {tilt_y_deg}")
            print(f"Body tilt i %: rx: {tilt_x_pct}, ry: {tilt_y_pct}")

        if sensor["sensorType"] == "Object" and sensor["outputVariableType"] == exu.OutputVariableType.ForceLocal:
            print(f"Sensor: {sensor["name"]}, converted to t and including DAF=1.2 and k_skl=1.1: {mbs.GetSensorValues(sensor_number)/9.81/1000*1.2*1.1}")


STEP_INTERVAL = 10
LOG_INTERVAL = 50

def post_step_user_function(mbs, t):

    step = mbs.sys.get("step", 0) + 1
    mbs.sys["step"] = step

    # compute residual occasionally
    if step % STEP_INTERVAL == 0:
        f_res, m_res, v_res = compute_residuals(mbs, prb.objects.values())
        mbs.sys["v_res"] = v_res
    else:
        v_res = mbs.sys.get("v_res", 0.0)

    # logging
#    if step % LOG_INTERVAL == 0:
#        print(f"t={t:.2f}, v={v_res:.3e}")

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

    return True



#def post_step_user_function(mbs, t):
#
#    f_res, m_res, v_res = compute_residuals(mbs, prb.objects.values())

    # reduce logging frequency
#    if int(t*50) % 50 == 0:
#        print(f"t={t:.2f}, v={v_res:.3e}")

    # adaptive damping
#    if v_res > 0.5:
#        factor = 0.5
#    elif v_res > 0.05:
#        factor = 0.2
#    elif v_res > 0.005:
#        factor = 0.1
#    else:
#        factor = 0.0   # don't interfere near convergence

#    if factor > 0:
#        coords_t = mbs.systemData.GetODE2Coordinates_t()
#        coords_t *= (1 - factor)
#        mbs.systemData.SetODE2Coordinates_t(coords_t)

#    return True



#def terminate_user_function(mbs, t):
#
#    f_res, m_res, v_res = compute_residuals(mbs, problem["bodies"])

#    if v_res < 1e-3 and f_res < 1e3 and m_res < 1e5:
#        print(f"✅ Converged at t={t:.2f}")
#        return True

#    return False



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
#    ss.timeIntegration.userDefinedTerminate = terminate_user_function


    # attach user function
#    mbs.SetPostStepUserFunction = userFunction

    # attach stop condition
#    ss.timeIntegration.userDefinedTerminate = stopFunction

    mbs.SolveDynamic(
        simulationSettings = ss,
        updateInitialValues = True,
        showHints = False,
    )

#    coords = mbs.systemData.GetODE2Coordinates()
#    coords_t = mbs.systemData.GetODE2Coordinates_t()

#    coords_t[:] = 0

#    mbs.systemData.SetODE2Coordinates(coords)
#    mbs.systemData.SetODE2Coordinates_t(coords_t)

#    mbs.SolveDynamic(
#        simulationSettings = ss,
#        updateInitialValues = True,
#    )

    mbs.SolutionViewer()
    SC.renderer.Stop()






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

        f_res, m_res, v_res = compute_residuals(mbs, problem["bodies"])

        print(f"\nTime: {t:.2f} s")
        print(f"  Force residual:  {f_res:.3e} N")
        print(f"  Moment residual: {m_res:.3e} Nm")
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


def compute_residual_history(mbs, problem):
    times = []
    res_f = []
    res_m = []

    sensor = mbs.GetSensorNumber("load.velocity")  # just to get time grid
    data = mbs.GetSensorStoredData(sensor)

    for row in data:
        t = row[0]
        mbs.systemData.SetTime(t)

        f, m, _ = compute_residuals(mbs, problem["bodies"])

        times.append(t)
        res_f.append(f)
        res_m.append(m)

    return times, res_f, res_m


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


