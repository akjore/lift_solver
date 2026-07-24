import { initRenderer, renderProblem } from "./renderer.js";
import { loadProblem, solveProblem, initializePyodide } from "./solver.js";

let problemData = null;
let pyodide;

let problemYaml = null;

// --------------------------------------------------
// Load default YAML from assets
// --------------------------------------------------
async function loadDefaultYaml() {
  try {
    const response = await fetch("assets/sample.yaml");

    if (!response.ok) {
      throw new Error(`Failed to load YAML: ${response.status}`);
    }

    const text = await response.text();
    problemYaml = text;

    const problemJson = await loadProblem(problemYaml, pyodide);
    console.log(problemJson);

    // Trigger rendering
    await renderProblem(problemJson);

  } catch (err) {
    console.error("YAML load error:", err);
  }
}

// --------------------------------------------------
// File upload (optional — keeps flexibility)
// --------------------------------------------------
function setupFileUpload() {

  const input = document.getElementById("fileInput");
  if (!input) return;

  input.addEventListener("change", function (e) {

    const file = e.target.files[0];
    if (!file) return;

    const reader = new FileReader();

    reader.onload = function () {

      problemData = jsyaml.load(reader.result);

      console.log("Uploaded YAML:", problemData);

//      console.log()
//      const output = document.getElementById("output");
//      if (output) {
//        output.textContent = JSON.stringify(problemData, null, 2);
//      }

      // Re-render
//      renderProblem(problemData);
    };

    reader.readAsText(file);
  });
}


async function run(problem) {

//  renderProblem(problem);

  const result = await solveProblem(problemYaml, pyodide);

  console.log("Solver result:", result);

  // later: pass result back into renderer
}

// --------------------------------------------------
// Init app
// --------------------------------------------------
window.addEventListener("load", async () => {

  console.log("App starting...");

  pyodide = await loadPyodide();
  await initializePyodide(pyodide);

  initRenderer("canvas");

  setupFileUpload();

  console.log("Loading default yaml.")
  loadDefaultYaml();

  console.log("Ready.");
});


async function runSolver() {

  console.log("Running solver...");

  const result = await solveProblem(problemData, pyodide);

  console.log("Result:", result);
}

function registerObject(id, object) {
    world.objectMap.set(
        id,
        object
    );
}
