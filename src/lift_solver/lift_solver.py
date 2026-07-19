"""Main entry module for setting up and solving a lift problem."""
import logging

from . import exu_problem, exu_solver, lift_problem
from .code_check import CodeCheck


def setup_logging() -> None:
    """Configure logging."""
    global logger

    logger = logging.getLogger(__name__)
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)


def solve_problem(problem: str) -> None:
    """Set up and solve lifting problem."""
    # Parse the problem into a model. Problem is a yaml-formatted string.
    prb = lift_problem.LiftProblem().from_yaml(problem)

    # Create the problem in exudyn
    exu_prb = exu_problem.ExuProblem(prb)

    # For debug purposes
    # exu_prb.mbs.Assemble()
    # exu_prb.SC.renderer.Start()
    # exu_prb.SC.renderer.DoIdleTasks()

    # Solve and extract results
    exu_slv = exu_solver.ExuSolver(exu_prb)
    exu_slv.solve(simulation_duration=20, time_step=0.002)
    results = exu_slv.get_results()

    # Code check
    #       temp only: manually populating
    code_check = CodeCheck(results, prb)
    print(code_check.results())
    print()
    print(results.export_initial_state())

    print()
#    print(results.to_render_model())

#    print(vars(results))


if __name__ == "__main__":
    import sys

    setup_logging()

    # Get file name from command line
    try:
        filename = sys.argv[1]
    except IndexError as ex:
        logger.error("Please provide path to yaml-file describing problem.")
        raise ex

    # Expectation is that file is in yaml format and describes problem to be solved
    with open(filename) as file:
        problem = file.read()

    solve_problem(problem)
