"""Main entry module for setting up and solving a lift problem."""
import logging

from . import lift_problem, solver


def setup_logging() -> None:
    """Configure logging."""
    global logger

    logger = logging.getLogger(__name__)
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)


def solve_problem(problem: str) -> None:
    """Set up and solve lifting problem."""
    # Parse the problem into a model. Problem is a yaml-formatted string.
    prb = lift_problem.LiftProblem().from_yaml(problem)

    sol = solver.solve(problem=prb, simulation_duration=20, time_step=0.002)


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
