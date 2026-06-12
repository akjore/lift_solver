const DEV_MODE = true;

export async function solveProblem(problem, pyodide) {

  const problemJSON = JSON.stringify(problem);

  pyodide.globals.set("problem_data", problemJSON);

  const resultJSON = await pyodide.runPythonAsync(`
import json
from lift_solver.solver import solve

problem = json.loads(problem_data)
result = solve(problem)

json.dumps(result)
`);

  return JSON.parse(resultJSON);
}

export async function initializePyodide(pyodide) {
  console.log("Initializing pyodide");
//  const resultJSON = await pyodide.runPythonAsync(`
//import sys
//sys.path.append("/src")
//`);

  loadSolver(pyodide);
  await pyodide.loadPackage("numpy");
  await pyodide.loadPackage("scipy")
}

async function loadSolver(pyodide) {
  console.log("Loading solver");

  if (DEV_MODE) {
    await loadSolverFromSource(pyodide);
  } else {
    await loadSolverFromWheel(pyodide);
  }
}

async function loadSolverFromSource(pyodide) {

  console.log("Loading from source");
  const manifest = await fetch("/src/lift_solver/manifest.json")
    .then(r => r.json());

  pyodide.FS.mkdirTree("/lift_solver");

  for (const file of manifest.files) {
    const code = await fetch(`/src/lift_solver/${file}`)
      .then(r => r.text());

    pyodide.FS.writeFile(`/lift_solver/${file}`, code);
  }

  await pyodide.runPythonAsync(`
import sys
sys.path.append("/")
`);
}


async function loadSolverFromWheel(pyodide) {

  await pyodide.runPythonAsync(`
import micropip
await micropip.install("/dist/lift_solver-0.1.0-py3-none-any.whl")
`);
}

