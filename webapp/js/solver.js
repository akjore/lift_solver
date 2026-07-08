const DEV_MODE = true;

export async function loadProblem(problem, pyodide) {
  pyodide.globals.set("problem", problem);

  const resultJSON = await pyodide.runPythonAsync(`
import json
from lift_solver.lift_problem import LiftProblem

prb = LiftProblem().from_yaml(problem)

json.dumps(prb.to_render_model())
`);

  return JSON.parse(resultJSON);
}

export async function solveProblem(problem, pyodide) {

//  const problemJSON = JSON.stringify(problem);

//  pyodide.globals.set("problem_data", problemJSON);
  pyodide.globals.set("problem", problem);

  const resultJSON = await pyodide.runPythonAsync(`
#import json
#from lift_solver.solver import solve
#import lift_solver
# from lift_solver.lift_solver import solve_problem
#import lift_solver
from lift_solver import shackle

#problem = json.loads(problem_data)
#result = solve(problem)
result = solve_problem(
    problem = problem,
  )

json.dumps(result)
`);

  return JSON.parse(resultJSON);
}

export async function initializePyodide(pyodide) {
  console.log("Initializing pyodide");
  await pyodide.loadPackage("micropip");

  const resultJSON = await pyodide.runPythonAsync(`
import micropip
import sys
sys.path.append("/")

await micropip.install("numpy")
await micropip.install("numpy-stl")
#await micropip.install("exudyn")
await micropip.install("pyyaml")
await micropip.install("pint")
await micropip.install("scipy")
`);

  await loadSolver(pyodide);
  console.log("Pyodide initialized.")
}


async function loadSolver(pyodide) {
  console.log("Loading solver");

  if (DEV_MODE) {
    await loadSolverFromSource(pyodide);
  } else {
    await loadSolverFromWheel(pyodide);
  }
}

export async function loadSolverFromSource(pyodide) {

    const manifest = await fetch("/manifest.json")
        .then(r => r.json());

    for (const sourcePath of manifest.files) {

        const response = await fetch(sourcePath);

        if (!response.ok) {
            throw new Error(
                `Failed to load ${sourcePath}`
            );
        }

        const content = await response.text();

        // Remove leading src/

        const targetPath = sourcePath.replace(/^\/?src\//, "/");

        ensureDirectory(pyodide, targetPath);

        pyodide.FS.writeFile(
            targetPath,
            content
        );

        console.log(`Loaded ${targetPath}`);
    }
}

function ensureDirectory(pyodide, filePath) {

    const parts = filePath.split("/");

    // Remove filename
    parts.pop();

    let current = "";

    for (const part of parts) {

        if (!part) continue;

        current += "/" + part;

        try {
            pyodide.FS.mkdir(current);
        }
        catch (err) {
            // Directory already exists
        }
    }
}

async function loadSolverFromWheel(pyodide) {

  await pyodide.runPythonAsync(`
import micropip
await micropip.install("/dist/lift_solver-0.1.0-py3-none-any.whl")
`);
}

