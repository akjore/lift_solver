import logging

from . import lift_problem
from . import solver


def setup_logging():
    global logger

    logger = logging.getLogger(__name__)
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)


def solve_problem(problem: str):
    # Parse the problem into a model. Problem is a yaml-formatted string.
    prb = lift_problem.LiftProblem().from_yaml(problem)

    sol = solver.solve(problem=prb, simulation_duration=20, time_step=0.002)





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

    solve_problem(problem)
    # Echo input
#    logger.info(problem)

#    prb = lift_problem.LiftProblem().from_yaml(problem)

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