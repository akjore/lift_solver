// ======================================================
// Proper ES-module-based Three.js renderer
// ======================================================

//import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js";
import * as THREE from "three";
import { TrackballControls } from "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/TrackballControls.js";

let scene, camera, renderer, controls;
let world;
//, model, helpers;

// ------------------------------------------------------
export function initRenderer(containerId = "canvas") {
  const container = document.getElementById(containerId);

  scene = new THREE.Scene();

  world = new THREE.Group();
  world.rotation.x = -Math.PI / 2;

  // split everything into model and helpers. Things that explode the visual bounding box go into helpers
//  model = new THREE.Group();
//  helpers = new THREE.Group();

//  world.add(model);
//  world.add(helpers);

  scene.add(world);
  scene.background = new THREE.Color(0xf0f0f0);

  camera = new THREE.PerspectiveCamera(
    60,
    container.clientWidth / container.clientHeight,
    0.1,
    1000
  );

  camera.position.set(6, 6, 6);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);

  container.appendChild(renderer.domElement);

  // ---- TrackballControls ----
  controls = new TrackballControls(camera, renderer.domElement);

  controls.rotateSpeed = 3.0;
  controls.zoomSpeed = 1.5;
  controls.panSpeed = 0.8;
  controls.dynamicDampingFactor = 0.15;
  controls.staticMoving = true;

//  controls.target.set(0, 0, 2);
//  controls.update();


  camera.position.set(10, 10, 10);
  controls.target.set(0, 0, 0);
  controls.update();

  // optional but helpful:
  camera.lookAt(0, 0, 0);


  // ---- Lights ----
  world.add(new THREE.AmbientLight(0x888888));

  const light = new THREE.DirectionalLight(0xffffff, 1);
  light.position.set(5, 10, 7);
  world.add(light);

  // ---- Helpers ----
  world.add(new THREE.AxesHelper(2));
  world.add(new THREE.GridHelper(10, 10));

  window.addEventListener("resize", onWindowResize);

//  addHelpers();

  animate();
}

// ------------------------------------------------------
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

// ------------------------------------------------------
function onWindowResize() {

  const container = renderer.domElement.parentElement;

  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();

  renderer.setSize(container.clientWidth, container.clientHeight);

  controls.handleResize();
}

// ------------------------------------------------------
// Helpers
// ------------------------------------------------------
function resolveRef(ref, bodies) {
  const [bodyName, pointName] = ref.split(".");
  const body = bodies[bodyName];

  const local = body.points[pointName];

  return { body, local };
}

//function getGlobalPoint(body, local) {
//  const p = new THREE.Vector3(...local);
//  const pos = new THREE.Vector3(...body.pose.position);

//  return p.add(pos); // no rotation yet
//}

function getGlobalPoint(body, local) {

  const p = new THREE.Vector3(...local);

  const pos = new THREE.Vector3(...body.pose.position);

  // --- rotation from Euler (XYZ assumed) ---
  const rot = body.pose.orientation || [0, 0, 0];

  const euler = new THREE.Euler(
    THREE.MathUtils.degToRad(rot[0]),
    THREE.MathUtils.degToRad(rot[1]),
    THREE.MathUtils.degToRad(rot[2]),
    "XYZ"
  );

  const q = new THREE.Quaternion().setFromEuler(euler);

  // apply rotation, then translation
  p.applyQuaternion(q);

  return p.add(pos);
}

// ------------------------------------------------------
function drawNode(position, color = 0xff0000) {

  const geometry = new THREE.SphereGeometry(0.05, 16, 16);
  const material = new THREE.MeshStandardMaterial({ color });

  const mesh = new THREE.Mesh(geometry, material);
  mesh.position.copy(position);

  world.add(mesh);
}

function drawLine(p1, p2, color = 0x0000ff) {

  const geometry = new THREE.BufferGeometry().setFromPoints([p1, p2]);
  const material = new THREE.LineBasicMaterial({ color });

  world.add(new THREE.Line(geometry, material));
}

// ------------------------------------------------------
function clearScene() {
  scene.children = scene.children.filter(obj =>
    obj.type === "AmbientLight" ||
    obj.type === "DirectionalLight" ||
    obj.type === "AxesHelper" ||
    obj.type === "GridHelper"
  );
}

// ------------------------------------------------------
export function renderProblem(problem) {
  drawNode(new THREE.Vector3(0, 0, 0), 0xff0000);

//  clearScene();
  clearWorld();
//  clearModel();

  const bodies = problem.bodies;

  // --- nodes ---
//  for (const bodyName in bodies) {
//    const body = bodies[bodyName];

//    for (const pointName in body.points) {
//      const local = body.points[pointName];
//      const global = getGlobalPoint(body, local);

//      drawNode(global);
//    }
//  }
  for (const bodyName in bodies) {

    const body = bodies[bodyName];

    // ✅ create a group per body
    const bodyGroup = new THREE.Group();
    drawBodyVisual(bodyGroup, body.visual);

//    mesh.position.set(...visual.offset);
//    drawNodeInGroup(bodyGroup, new THREE.Vector3(1,1,1), 0x00ff00);

    // --- apply body transform ---
    const pos = new THREE.Vector3(...body.pose.position);

    const rot = body.pose.orientation || [0, 0, 0];

    const euler = new THREE.Euler(
      THREE.MathUtils.degToRad(rot[0]),
      THREE.MathUtils.degToRad(rot[1]),
      THREE.MathUtils.degToRad(rot[2]),
      "XYZ"
    );

    bodyGroup.position.copy(pos);
    bodyGroup.setRotationFromEuler(euler);

    // ✅ add to world
    world.add(bodyGroup);

    // ✅ draw coordinate system
    drawBodyAxes(bodyGroup, body);
//    drawBodyAxesHelper(bodyGroup);


    // ✅ draw CoG
//    drawCoG(bodyGroup, body.cog || [0,0,0]);

    // ✅ draw attachment points (in LOCAL coords now!)
    for (const pointName in body.points) {
      const local = body.points[pointName];

      const p = new THREE.Vector3(...local);

      drawNodeInGroup(bodyGroup, p);
    }
  }

  // --- elements ---
  for (const el of problem.elements) {

    const from = resolveRef(el.from, bodies);
    const to   = resolveRef(el.to, bodies);

    const p1 = getGlobalPoint(from.body, from.local);
    const p2 = getGlobalPoint(to.body, to.local);

    drawLine(p1, p2);
  }

  fitCameraToScene(problem);
}

function fitCameraToScene(problem) {

//  const box = new THREE.Box3().setFromObject(world);
  const box = computeRigBounds(problem);
//  const { box, centroid } = computeRigBounds(problem);
//  const box = computeRigBounds(problem);

//  const box = new THREE.Box3();

//  world.traverse((obj) => {
//    if (obj.type === 'AxesHelper') return;   // ✅ ignore axes
//    box.expandByObject(obj);
//  });


//  console.log(box);

  if (box.isEmpty()) {
    console.warn("fitCameraToScene: world is empty");
    return;
  }

  const center = box.getCenter(new THREE.Vector3());
  center.applyEuler(new THREE.Euler(-Math.PI / 2, 0, 0));
  //  const center = centroid;
  const size = box.getSize(new THREE.Vector3());

  const maxDim = Math.max(size.x, size.y, size.z);
  const distance = maxDim * 0.6;
//  const distance = 0
//  const fov = camera.fov * (Math.PI / 180);

  // use vertical size (Z in your engineering system)
//  const height = size.z;

  // compute correct distance
//  const distance = (height / 2) / Math.tan(fov / 2);

  camera.position.set(
    center.x + distance,
    center.y + distance,
    center.z + distance
  );


  const viewDir = new THREE.Vector3(1, 1, 1).normalize();

//  camera.position.copy(
//    center.clone().add(viewDir.multiplyScalar(distance))
//  );


  controls.target.copy(center);
  controls.update();

  console.log("Camera fitted to:", center);
  console.log("Bounding size:", size);
}


function clearWorld() {
  while (world.children.length > 0) {
    world.remove(world.children[0]);
  }
}

//function clearModel() {
//  while (model.children.length > 0) {
//    model.remove(model.children[0]);
//  }
//}


function drawBodyAxes(group, body) {

  const axes = new THREE.AxesHelper(0.5); // size

  //axes.userData.ignoreBounds = true;
  if (body.cog) {
    axes.position.set(...body.cog);
  }


  group.add(axes);
}


function drawCoG(group, cog) {

  const geometry = new THREE.SphereGeometry(0.08, 16, 16);
  const material = new THREE.MeshStandardMaterial({ color: 0xffaa00 });

  const sphere = new THREE.Mesh(geometry, material);

  sphere.position.set(cog[0], cog[1], cog[2]);

  group.add(sphere);
}

function drawNodeInGroup(group, position, color = 0xff0000) {

  const geometry = new THREE.SphereGeometry(0.07, 16, 16);
  const material = new THREE.MeshStandardMaterial({ color });

  const sphere = new THREE.Mesh(geometry, material);

  sphere.position.copy(position);

  group.add(sphere);
}


function drawBodyAxesHelper(bodyGroup) {

  const axes = new THREE.AxesHelper(0.5);

  axes.position.copy(bodyGroup.position);
  axes.quaternion.copy(bodyGroup.quaternion);

//  helpers.add(axes);
}

//function addHelpers() {

  // ✅ grid
//  const grid = new THREE.GridHelper(50, 50);
//  helpers.add(grid);

  // ✅ global axes
//  const axes = new THREE.AxesHelper(5);
//  helpers.add(axes);
//}

function computeRigBounds(problem) {

  const box = new THREE.Box3();
  const bodies = problem.bodies;

  for (const bodyName in bodies) {
    const body = bodies[bodyName];

    for (const pointName in body.points) {
      const local = body.points[pointName];
      const global = getGlobalPoint(body, local);

      box.expandByPoint(global);
    }
  }

  return box;
}

function drawBodyVisual(bodyGroup, visual) {
  if (!visual) return;

  let mesh;

  if (visual.type === "box") {

    const geom = new THREE.BoxGeometry(
      visual.size[0],
      visual.size[1],
      visual.size[2]
    );

    const mat = new THREE.MeshStandardMaterial({
      color: visual.color || 0x999999,
      transparent: visual.opacity < 1,
      opacity: visual.opacity ?? 1
    });

    mesh = new THREE.Mesh(geom, mat);

  } else if (visual.type === "cylinder") {

    const geom = new THREE.CylinderGeometry(
      visual.radius,
      visual.radius,
      visual.length,
      16
    );

    const mat = new THREE.MeshStandardMaterial({
      color: visual.color || 0x999999
    });

    mesh = new THREE.Mesh(geom, mat);

    // align axis
    if (visual.axis === "x") {
      mesh.rotation.z = Math.PI / 2;
    } else if (visual.axis === "z") {
      mesh.rotation.x = Math.PI / 2;
    }
  }

  if (!mesh) return;

  // offset inside body
  if (visual.offset) {
    mesh.position.set(...visual.offset);
  }

  bodyGroup.add(mesh);
}
