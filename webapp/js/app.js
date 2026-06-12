import { initRenderer, renderProblem } from "./renderer.js";
import { solveProblem, initializePyodide } from "./solver.js";

let problemData = null;
let pyodide;

// --------------------------------------------------
// Load default YAML from assets
// --------------------------------------------------
async function loadDefaultYaml() {
  try {
    const response = await fetch("/webapp/assets/sample.yaml");

    if (!response.ok) {
      throw new Error(`Failed to load YAML: ${response.status}`);
    }

    const text = await response.text();

    problemData = jsyaml.load(text);

    console.log("Problem loaded:", problemData);

    // Optional: display JSON in <pre>
    const output = document.getElementById("output");
    if (output) {
      output.textContent = JSON.stringify(problemData, null, 2);
    }

    // Trigger rendering
    renderProblem(problemData);

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

//      const output = document.getElementById("output");
//      if (output) {
//        output.textContent = JSON.stringify(problemData, null, 2);
//      }

      // Re-render
      renderProblem(problemData);
    };

    reader.readAsText(file);
  });
}


async function run(problem) {

  renderProblem(problem);

  const result = await solveProblem(problem, pyodide);

  console.log("Solver result:", result);

  // later: pass result back into renderer
}

// --------------------------------------------------
// Init app
// --------------------------------------------------
window.addEventListener("load", () => {

  console.log("App starting...");

  // 1. Init Three.js renderer
  initRenderer("canvas");

  // 2. Enable optional file upload
  setupFileUpload();

  // 3. Auto-load default YAML
  loadDefaultYaml();
});



async function init() {
  pyodide = await loadPyodide();
  await initializePyodide(pyodide);

  document.getElementById("solveBtn").addEventListener("click", runSolver);
  console.log("Ready.");
}

async function runSolver() {

  console.log("Running solver...");

  const result = await solveProblem(problemData, pyodide);

  console.log("Result:", result);
}

init();

