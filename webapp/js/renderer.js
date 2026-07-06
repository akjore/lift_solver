// ======================================================
// ES-module-based Three.js renderer
// ======================================================
import * as THREE from "three";
import { TrackballControls } from "https://cdn.jsdelivr.net/npm/three@0.181.1/examples/jsm/controls/TrackballControls.js";
import { createBody } from "./create_body.js";
import { createShackle } from "./create_shackle.js";
import { createSling } from "./create_sling.js";
import { findSelectable, requestZoom, selectObject } from "./selection.js";
import { buildTree, initializeVisibility } from "./tree.js";

let scene, camera, renderer, controls;

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2()

const world = {
    root: new THREE.Group(),

    bodies: new THREE.Group(),
    shackles: new THREE.Group(),
    slings: new THREE.Group(),
    attachmentPoints: new THREE.Group(),
    cogs: new THREE.Group(),

    objectMap: new Map(),

    pickables: []
};

// ------------------------------------------------------
export function initRenderer(containerId = "canvas") {
    const container = document.getElementById("three-container")

    world.root.add(world.bodies);
    world.root.add(world.shackles);
    world.root.add(world.slings);
    world.root.add(world.attachmentPoints);
    world.root.add(world.cogs);

    scene = new THREE.Scene();
    scene.add(world.root);

    camera = new THREE.PerspectiveCamera(
      60,
      container.clientWidth / container.clientHeight,
      0.1,
      1000
    );

    // set z-axis up, position, and view direction
    camera.up.set(0, 0, 1);
    camera.position.set(10, 10, 10);
    camera.lookAt(0, 0, 0);

    // renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);

    container.appendChild(renderer.domElement);

    // trackballControls
    controls = new TrackballControls(camera, renderer.domElement);

    controls.rotateSpeed = 3.0;
    controls.zoomSpeed = 1.5;
    controls.panSpeed = 0.8;
    controls.dynamicDampingFactor = 0.15;
    controls.staticMoving = true;
    controls.target.set(0, 0, 0);
    controls.update();

    // lights
    world.root.add(new THREE.AmbientLight(0x888888));

    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 7);
    world.root.add(light);

    // ---- Helpers ----
    world.root.add(new THREE.AxesHelper(2));
  //  world.root.add(new THREE.GridHelper(10, 10));

    window.addEventListener("resize", onWindowResize);

    renderer.domElement.addEventListener(
      "pointerdown",
      onPointerDown
    );

    renderer.domElement.addEventListener(
      "dblclick",
      onDoubleClick
    );

    document.addEventListener(
      "zoomToObject",
      event => {

        zoomToObject(
            event.detail
        );

      }
    );

    document.getElementById("fit-button").addEventListener(
        "click",
        () => fitCameraToScene(world.root)
    );

    document.getElementById("iso-button").addEventListener(
        "click",
        viewIso
    );

    document.getElementById("top-button").addEventListener(
        "click",
        viewTop
    );

    document.getElementById("front-button").addEventListener(
        "click",
        viewFront
    );

    document.getElementById("side-button").addEventListener(
        "click",
        viewSide
    );

    initializeVisibility(world);

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
export async function renderProblem(problem) {
//  clearScene();
////  clearWorld();
//  clearModel();

  // Bodies
    for (const body of problem.bodies) {
      const group = await createBody(body);
      world.bodies.add(group);

      world.pickables.push(group);
    }

    // Shackles
    for (const shackle of problem.shackles) {
      const group = await createShackle(shackle);
      world.shackles.add(group);

      world.pickables.push(group);
   }

    // Update registrations - needed for slings to find end points
    world.objectMap.clear();
    registerSceneTree(world.root);

    // Slings
    for (const sling of problem.rigging) {
      const group = createSling(sling, world);
      world.slings.add(group);

      world.pickables.push(group);
    }

    // Update registrations
    world.objectMap.clear();
    registerSceneTree(world.root);

    // Build the tree
    buildTree(world);

    fitCameraToScene(world.root);

    console.log("Loaded problem.");
    console.log(world.root);
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
function fitCameraToScene(root) {
  const box = computeFitBox(root);

  const center = new THREE.Vector3();
  box.getCenter(center);

  const size = new THREE.Vector3();
  box.getSize(size);

  const radius = 0.5 * Math.max(
    size.x,
    size.y,
    size.z
  );

  const distance =
    radius /
    Math.tan(
        THREE.MathUtils.degToRad(
            camera.fov * 0.5
        )
    );

  camera.position.set(
    center.x + distance,
    center.y + distance,
    center.z + distance
  );

  controls.target.copy(center);
  controls.update();
}

function computeFitBox(root) {
    const box = new THREE.Box3();

    root.traverse((obj) => {
        if (!obj.userData?.fit) {
            return;
        }

        const p = new THREE.Vector3();
        obj.getWorldPosition(p);

        box.expandByPoint(p);
    });

    return box;
}

function clearWorld() {
  // clear the contents of the groups below world
  for (const child of world.children) {
    while (child.children.length > 0) {
      child.remove(child.children[0]);
    }
  }
}

function registerSceneTree(object) {
    const id = object.userData?.id;

    if (id) {
        world.objectMap.set(id, object);
    }

    for (const child of object.children) {
        registerSceneTree(child);
    }
}

function onPointerDown(event) {
    const rect = renderer.domElement.getBoundingClientRect();

    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;


    world.root.updateMatrixWorld(true);
    camera.updateMatrixWorld(true);
    raycaster.setFromCamera(mouse, camera);

    const hits = raycaster.intersectObjects(
      world.pickables,
      true
    );

    if (hits.length === 0) {
        return;
    }

    const selected = findSelectable(hits[0].object);

    if (selected) {
        selectObject(selected);
    }
}

function onDoubleClick(event) {

    const rect =
        renderer.domElement.getBoundingClientRect();

    mouse.x =
        ((event.clientX - rect.left) /
         rect.width) * 2 - 1;

    mouse.y =
        -((event.clientY - rect.top) /
          rect.height) * 2 + 1;

    raycaster.setFromCamera(
        mouse,
        camera
    );

    const hits =
        raycaster.intersectObjects(
            world.pickables,
            true
        );

    if (hits.length === 0) {
        return;
    }

    const object =
        findSelectable(
            hits[0].object
        );

    if (!object) {
        return;
    }

    selectObject(object);

    document.dispatchEvent(
        new CustomEvent(
            "zoomToObject",
            {
                detail: object
            }
        )
    );
}

function zoomToObject(object) {

    const box =
        new THREE.Box3()
            .setFromObject(object);

    const center =
        new THREE.Vector3();

    const size =
        new THREE.Vector3();

    box.getCenter(center);
    box.getSize(size);

    const radius =
        Math.max(
            size.x,
            size.y,
            size.z
        );

    const direction =
        new THREE.Vector3()
            .subVectors(
                camera.position,
                controls.target
            )
            .normalize();

    camera.position.copy(
        center.clone().add(
            direction.multiplyScalar(
                radius * 3
            )
        )
    );

    controls.target.copy(center);
    controls.update();
}

function setView(direction) {

    const distance =
        camera.position.distanceTo(
            controls.target
        );

    camera.position.copy(
        controls.target.clone().add(
            direction
                .clone()
                .normalize()
                .multiplyScalar(distance)
        )
    );

    camera.up.set(0, 0, 1);

    controls.update();
}

function viewIso() {
    setView(new THREE.Vector3(1, 1, 1));
}

function viewTop() {
    setView(new THREE.Vector3(0, 0, 1));
}

function viewFront() {
    setView(new THREE.Vector3(0, -1, 0));
}

function viewSide() {
    setView(new THREE.Vector3(1, 0, 0));
}