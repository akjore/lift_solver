import logging
import math

import numpy as np

import exudyn as exu
from exudyn import graphics

from .shackle import Shackle

from . import ureg, Q_
# Exudyn units: SI
# Forces in N
# Mass in kg
# Lengths in m

def setup_logging():
    global logger

    logger = logging.getLogger(__name__)
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)


def setup_body(mbs, g, body):
    mass = Q_(body["mass"])
    cog = Q_.from_list([Q_(param) for param in body["cog"]])

    position = Q_.from_list([Q_(param) for param in body["pose"]["position"]])
    orientation = body["pose"]["orientation"]

    offset = None
    if "visual" in body and "offset" in body["visual"]:
        offset = Q_.from_list([Q_(param) for param in body["visual"]["offset"]])

    size = None
    if "visual" in body and "size" in body["visual"]:
        size = Q_.from_list([Q_(param) for param in body["visual"]["size"]])

    length = 100 * ureg.meter
    width = 100 * ureg.meter
    height = 100 * ureg.meter
    density = mass / (length * width * height)

    inertia = exu.utilities.InertiaCuboid(
        density = density.to("kg/(m*m*m)").magnitude,
        sideLengths = [length.to("m").magnitude, width.to("m").magnitude, height.to("m").magnitude],
    )

    # Shift the cuboid cog from 0, 0, 0 to the CoG
    inertia = inertia.Translated(cog.to("m").magnitude)

    # Create graphics
    gr = None
    if "visual" in body:
        if body["visual"]["type"] == "box":
            gr = graphics.Brick(
                centerPoint = offset.to("m").magnitude,
                size = size.to("m").magnitude,
                addNormals = False,
                addEdges = True,
                addFaces = False,
                roundness = 0,
                nTiles = 12,
            )
        elif body["visual"]["type"] == "cylinder":
            if body["visual"]["axis"] == "x":
                ax = [1, 0, 0]
            elif body["visual"]["axis"] == "y":
                ax = [0, 1, 0]
            else:
                ax = [0, 0, 1]

            p1 = np.array(ax) * body["visual"]["length"]/2
            p2 = -p1

            gr = graphics.Tube(
                points = [p1, p2],
                axes = [ax, ax],
                radius = body["visual"]["diameter"]/2,
                nTiles = 16
            )

    graphics_data_list = [gr, graphics.Basis(inertia.COM(), length=0.5)]

    # Create the body
    mx = exu.utilities.RotationMatrixX(math.radians(orientation[0]))
    my = exu.utilities.RotationMatrixY(math.radians(orientation[1]))
    mz = exu.utilities.RotationMatrixZ(math.radians(orientation[2]))
    referenceRotationMatrix = (mx @ my) @ mz

    body_number = mbs.CreateRigidBody(
        name = body["id"],
        inertia = inertia,
        referencePosition = position.to("m").magnitude,
        gravity = g.to("m/s**2").magnitude,
        referenceRotationMatrix = referenceRotationMatrix,
        graphicsDataList = graphics_data_list,
    )

    # Add markers for each of the PoIs
    for p_name, p_coord in body["points"].items():
        p = Q_.from_list([Q_(param) for param in p_coord])
        m = mbs.AddMarker(
            exu.utilities.MarkerBodyRigid(
                name = body["id"] + "." + p_name,
                bodyNumber = body_number,
#                localPosition = p_coord,
                localPosition = p.to("m").magnitude,
                visualization = exu.utilities.VMarkerBodyRigid()
            ),
        )


def setup_shackle(mbs, g, shackle):
    """Add shackle to problem."""
    sh = Shackle.from_model(shackle["id"], shackle["model"])
    sh.connect_pin_to(shackle["pin_connection"])
    assert 1 == 2


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


# def setup_sling(mbs, g, sling, mass, damping_rope_fac=0.2, damping_rope_torsional_fac=0*1e-4, damping_rope_shear_fac=0*1e-4):
def setup_sling(mbs, g, sling, representative_mass):
    EA = sling.get("EA", None)
    k = sling.get("k", None)
    d = Q_(sling["d"])
    L0 = Q_(sling["L0"])
    from_m = sling["from"]
    to_m = sling["to"]

    color = graphics.color.lawngreen

    EA_prime = next((item for item in [EA, Q_(k)*L0] if item is not None))
    EA_prime = Q_(EA_prime)
    damping_fac = compute_rope_damping(EA_prime, L0, representative_mass)

    # Prepare list of marker numbers
    markers = [
        mbs.GetMarkerNumber(from_m),
        mbs.GetMarkerNumber(to_m)
    ]

    sheave_axes = exu.Vector3DList()
    r_roll_arm = []

    # dummy data
    for marker in markers:
        r_roll_arm.append(0)
        sheave_axes.Append([1, 0, 0])


    sl = mbs.AddObject(
        exu.utilities.ReevingSystemSprings(
            name = sling["id"],
            markerNumbers = markers,
            hasCoordinateMarkers = False,
#            coordinateFactors = coordinate_factors,
#            coordinateFactors = [1,1],           # don't really know what this does (?)
#            stiffnessPerLength = k,          # input isn't k=EA/L0, but EA in N
            stiffnessPerLength = EA_prime.to("N").magnitude,
            dampingPerLength = (damping_fac * EA_prime).magnitude,      # check unit!
            referenceLength = L0.to("m").magnitude,
            dampingTorsional = 0.0,
            dampingShear = 0.0,
            sheavesAxes = sheave_axes,
            sheavesRadii = r_roll_arm,
            visualization = exu.utilities.VReevingSystemSprings(
                ropeRadius = d.to("m").magnitude/2,
                color = color
            ),
        ),
    )



#        return self._mbs.GetObjectOutputBody(
#            self.body_number,
#            exu.OutputVariableType.RotationMatrix,
#            localPosition=[0, 0, 0],
#            configuration=exu.ConfigurationType.Reference,
#        ).reshape(3, 3)

#        shackle_node_no = mbs.GetObject(self.body_number)["nodeNumber"]

#point = mbs.GetObjectOutputBody(
#    ocranehouse,
#    exu.OutputVariableType.Position,
#    localPosition=cranehouse_centre_of_bearing,
#    configuration=exu.ConfigurationType.Reference,
#    )
#print(mbs.GetObject(mbs.GetObjectNumber("LP1")))

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


def get_global_marker_position(mbs, marker_num):
    # Get the marker properties; body and local position on body
    m = mbs.GetMarker(marker_num)

    # Return the global position of that local position
    return mbs.GetObjectOutputBody(
        m["bodyNumber"],
        exu.OutputVariableType.Position,
        localPosition = m["localPosition"],
        configuration = exu.ConfigurationType.Reference,
    )


def get_global_position(mbs, body, local_position):
    """Return the global position of local_position on body 'body'."""

    return mbs.GetObjectOutputBody(
        body,
        exu.OutputVariableType.Position,
        localPosition = local_position,
        configuration = exu.ConfigurationType.Reference,
    )


def setup_constraint(mbs, ground, constraint):
    body = constraint["body"]
    point = constraint["point"]
    constraints = constraint["constraints"]

    m = body + "." + point
    m_num = mbs.GetMarkerNumber(m)
    m_pos = get_global_marker_position(mbs, m_num)

    m_ground = mbs.AddMarker(
        exu.utilities.MarkerBodyRigid(
            bodyNumber = ground,
            localPosition = m_pos,
        )
    )

    joint = mbs.AddObject(
        exu.utilities.GenericJoint(
            markerNumbers = [m_ground, m_num],
            constrainedAxes = constraints,
            visualization = exu.utilities.VGenericJoint(
                axesRadius = 0.5,
                axesLength = 0.5
            )
        )
    )


def setup_damping(mbs, ground, problem):
    """Add damping to quell body movements."""

    # For each body, add a damper between body and ground
    for body in problem["bodies"]:
        # Get body number from name
        b = mbs.GetObjectNumber(body["id"])

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


def setup_sensors(mbs, problem):
    """Specify sensors."""

    for body in problem["bodies"]:
        b = mbs.GetObjectNumber(body["id"])
        o = mbs.GetObject(b)

        sensor_pos = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = body["id"] + ".position",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Position,
            )
        )

        sensor_pos = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = body["id"] + ".displacement",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Displacement,
            )
        )

        sensor_pos = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = body["id"] + ".rotation",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Rotation,
            )
        )

        sensor_vel = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = body["id"] + ".velocity",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Velocity,
            )
        )

        sensor_acc = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = body["id"] + ".acceleration",
                bodyNumber = b,
                localPosition = o["physicsCenterOfMass"],
                storeInternal = True,
                outputVariableType = exu.OutputVariableType.Acceleration,
            )
        )

        sensor_rotmat = mbs.AddSensor(
            exu.utilities.SensorBody(
                name = body["id"] + ".rotationMatrix",
                bodyNumber = b,
                outputVariableType = exu.OutputVariableType.RotationMatrix,
                storeInternal = True,
            )
        )

#        sensor_f = mbs.AddSensor(
#            exu.utilities.SensorBody(
#                name = body["id"] + ".force",
#                bodyNumber = b,
#                localPosition = o["physicsCenterOfMass"],
#                storeInternal = True,
#                outputVariableType = exu.OutputVariableType.Force,
#            )
#        )

    for sling in problem["elements"]:
        s = mbs.GetObjectNumber(sling["id"])
        sensor_f = mbs.AddSensor(
            exu.utilities.SensorObject(
                name = sling["id"] + ".force",
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
        b = mbs.GetObjectNumber(body["id"])
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
            bdy = [bdy for bdy in problem["bodies"] if bdy["id"]==b["name"]][0]

            ref = Q_.from_list([Q_(param) for param in bdy["pose"]["position"]])
#            print(f"Sensor: {sensor["name"]}, position after simulation: {np.array(bdy["pose"]["position"])+mbs.GetSensorValues(sensor_number)}")
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


def setup_from_problem(ground, problem):
    # Set up environment
    g = Q_.from_list([Q_(param) for param in problem["environment"]["gravity"]])

    # Create and place the bodies
    for body in problem["bodies"]:
        setup_body(mbs, g, body)

    # Create and connect the shackles
    for shackle in problem["shackles"]:
        setup_shackle(mbs, g, shackle)

    # Create and connect the slings. Provide mass for calculation of damping
    representative_mass = Q_(max([b["mass"] for b in problem["bodies"]]))
    for sling in problem["elements"]:
        setup_sling(mbs, g, sling, representative_mass)

    # Apply constraints
    for constraint in problem["constraints"]:
        setup_constraint(mbs, ground, constraint)


def post_step_user_function(mbs, t):

    f_res, m_res, v_res = compute_residuals(mbs, problem["bodies"])

    # reduce logging frequency
    if int(t*50) % 50 == 0:
        print(f"t={t:.2f}, v={v_res:.3e}")

    # ✅ adaptive damping
    if v_res > 0.5:
        factor = 0.5
    elif v_res > 0.05:
        factor = 0.2
    elif v_res > 0.005:
        factor = 0.1
    else:
        factor = 0.0   # don't interfere near convergence

    if factor > 0:
        coords_t = mbs.systemData.GetODE2Coordinates_t()
        coords_t *= (1 - factor)
        mbs.systemData.SetODE2Coordinates_t(coords_t)

    return True



def terminate_user_function(mbs, t):

    f_res, m_res, v_res = compute_residuals(mbs, problem["bodies"])

    if v_res < 1e-3 and f_res < 1e3 and m_res < 1e5:
        print(f"✅ Converged at t={t:.2f}")
        return True

    return False



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


def solve(problem, simulation_duration=20, time_step=0.002):
    global SC, mbs

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

    max_force, max_moment, max_velocity = compute_residuals(mbs, problem["bodies"])
    print(f"Max residual force: {max_force}")
    print(f"Max residual moment: {max_moment}")
    print(f"Max residual velocity: {max_velocity}")

    plot_convergence(mbs, problem)


#    times, rf, rm = compute_residual_history(mbs, problem)
#    plot_residuals(times, rf, rm)




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


if __name__ == "__main__":
    import sys
    import yaml

    setup_logging()

    # Get file name from command line
    try:
        filename = sys.argv[1]
    except IndexError as ex:
        logger.error("Please provide path to yaml-file describing problem.")
        raise ex

    # Expectation is that file is in yaml format and describes problem to be solved
    with open(filename, 'r') as file:
#        problem = yaml.safe_load(file)
        problem = file.read()

    # Echo input
#    logger.info(problem)

    from . import lift_problem
    prb = lift_problem.LiftProblem().from_yaml(problem)

    # Set up problem
#    setup_from_problem(problem)
#    solve(problem)

#    mbs.Assemble()

#    mbs.ComputeSystemDegreeOfFreedom(verbose=True)

#    SC.renderer.Start()
#    SC.renderer.DoIdleTasks()

#    simulation_settings = exu.SimulationSettings()
#    simulation_settings.linearSolverSettings.ignoreSingularJacobian = True
#    simulation_settings.linearSolverType = exu.LinearSolverType.EXUdense
#    simulation_settings.linearSolverType = exu.LinearSolverType.EigenDense
#    simulation_settings.staticSolver.loadStepGeometric = True
#    simulation_settings.staticSolver.verboseMode = 1
#    simulation_settings.staticSolver.numberOfLoadSteps  = 200
#    simulation_settings.staticSolver.newton.relativeTolerance = 1e-4 # default 1e-7
#    simulation_settings.displayStatistics = True
#    simulation_settings.staticSolver.newton.absoluteTolerance = 1e-6 # default 1e-10
#    simulation_settings.staticSolver.stabilizerODE2term = 200
#    simulation_settings.staticSolver.newton.maxIterations = 10000
##    success = mbs.SolveStatic(
##        simulation_settings,
##        updateInitialValues = True,
#        showHints = True,
##    )




#    tEnd = 80
#    tEnd = 300
#    tEnd = 500
#    h=0.001
#    h=0.0001
##    tEnd = 20
#    h = 0.005
##    h = 0.002

##    solutionFile = 'solution/coordsCrane.txt'
##    simulationSettings = exu.SimulationSettings() #takes currently set values or default values

##    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
##    simulationSettings.timeIntegration.endTime = tEnd
#    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
##    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.1
#    simulationSettings.solutionSettings.writeSolutionToFile= True #set False for CPU performance measurement
#    simulationSettings.solutionSettings.solutionWritePeriod= 0.2
#    simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFile
##    simulationSettings.solutionSettings.sensorsWritePeriod = 0.02
    # simulationSettings.timeIntegration.simulateInRealtime=True
    # simulationSettings.timeIntegration.realtimeFactor=5
##    SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
##    simulationSettings.parallel.numberOfThreads=4
##    simulationSettings.displayComputationTime = True

##    simulationSettings.timeIntegration.verboseMode = 1

#    simulationSettings.timeIntegration.newton.useModifiedNewton = True
##    simulationSettings.timeIntegration.newton.useModifiedNewton = False

#    if False:
#        #traces:
#        SC.visualizationSettings.sensors.traces.listOfPositionSensors = [sPosTCP]
#        SC.visualizationSettings.sensors.traces.listOfTriadSensors =[sRotTCP]
#        SC.visualizationSettings.sensors.traces.showPositionTrace=True
#        SC.visualizationSettings.sensors.traces.showTriads=True
##        SC.visualizationSettings.sensors.traces.triadSize=2
#        SC.visualizationSettings.sensors.traces.showVectors=False
#        SC.visualizationSettings.sensors.traces.showFuture=False
#        SC.visualizationSettings.sensors.traces.triadsShowEvery=5


#    SC.visualizationSettings.nodes.show = True
#    SC.visualizationSettings.nodes.drawNodesAsPoint  = False
#    SC.visualizationSettings.nodes.showBasis = True
#    SC.visualizationSettings.nodes.basisSize = 0.2

#    SC.visualizationSettings.openGL.multiSampling = 4
#    SC.visualizationSettings.openGL.shadow = 0.3*0
#    SC.visualizationSettings.openGL.light0position = [-50,200,100,0]

#    SC.visualizationSettings.window.renderWindowSize=[1920,1200]
    #SC.visualizationSettings.general.autoFitScene = False #use loaded render state

    ## start renderer and dynamic simulation
#    useGraphics = True
#    if useGraphics:
#        SC.renderer.Start()
#        if 'renderState' in exu.sys:
#            SC.renderer.SetState(exu.sys[ 'renderState' ])
#        SC.renderer.DoIdleTasks()


##    mbs.SolveDynamic(simulationSettings,
##        #solverType=exu.DynamicSolverType.TrapezoidalIndex2 #in this case, drift shows up significantly!
##    )
#    solve_with_auto_stop(mbs, SC, problem)

#    SC.visualizationSettings.nodes.show = True
#    SC.visualizationSettings.nodes.drawNodesAsPoint  = False
#    SC.visualizationSettings.nodes.showBasis = True
#    SC.visualizationSettings.nodes.basisSize = 0.2

#    SC.visualizationSettings.openGL.multiSampling = 4
#    SC.visualizationSettings.openGL.shadow = 0.3*0
#    SC.visualizationSettings.openGL.light0position = [-50,200,100,0]

#    SC.visualizationSettings.window.renderWindowSize=[1920,1200]
    #SC.visualizationSettings.general.autoFitScene = False #use loaded render state

    ## start renderer and dynamic simulation
#    useGraphics = True
#    if useGraphics:
#        SC.renderer.Start()
#        if 'renderState' in exu.sys:
#            SC.renderer.SetState(exu.sys[ 'renderState' ])
#        SC.renderer.DoIdleTasks()


#    mbs.SolveDynamic(simulationSettings,
#                 #solverType=exu.DynamicSolverType.TrapezoidalIndex2 #in this case, drift shows up significantly!
#    )

#    mbs.SolutionViewer()
#    SC.renderer.Stop()
#    import matplotlib
#    matplotlib.use("agg")      # only for non-interactive use
#    matplotlib.use("QT5Agg")
#    matplotlib.use("TkAgg")

#    disp = mbs.GetSensorNumber("load.position")
#    o = mbs.GetSensor(disp)
#    mbs.PlotSensor(
##        sensorNumbers=[disp, disp, disp],
#        components=[0, 1, 2],
#        labels=["displacement (m) x", "displacement (m) y", "displacement (m) z"],
#        colorCodeOffset=0,
#        newFigure=False,
#        figureName="C:\\appl\\development\\lift_solver\\src\\lift_solver\\tst.png",
#    )

#    print(mbs.systemData.Info())
#    print(o)
#    uTip = mbs.GetSensorValues(disp)
#    print(uTip)

#    history = mbs.GetSensorStoredData(disp)
#    print(history)
#    for row in disp:
#        print(row)

#    sensor_vel = mbs.GetSensorNumber("load.velocity")
#    print(mbs.GetSensorValues(sensor_vel))

#    sensor_f = mbs.GetSensorNumber("load.force")
#    print(mbs.GetSensorValues(sensor_f))

#    force, moment, velocity = compute_residuals(mbs, problem["bodies"])
#    print(f"Residual force: {force}")
#    print(f"Residual moment: {moment}")
#    print(f"Residual velocity: {velocity}")

    #compute error for test suite: - copied from cranereeving sample
#    sol2 = mbs.systemData.GetODE2Coordinates();
#    u = np.linalg.norm(sol2);
#    exu.Print('solution of craneReevingSystem=',u)