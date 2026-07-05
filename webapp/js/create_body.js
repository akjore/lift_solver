import * as THREE from "three";
import { createAttachmentPoint } from "./create_attachment_point.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const stlCache = new Map();
const stlLoader = new STLLoader();


export async function createBody(body) {
    // create groups
    const bodyGroup = new THREE.Group();
    bodyGroup.name = body.id;

    bodyGroup.userData = {
        id: body.id,
        type: "body"
    };

    const attachmentPoints = new THREE.Group();
    attachmentPoints.name = "attachmentPoints";
    bodyGroup.add(attachmentPoints);

    // set transformation of body and children
    const m = matrix4From3x3(body.rotation);

    bodyGroup.position.set(...body.position);
    bodyGroup.setRotationFromMatrix(m);

    // draw coordinate system marker at CoG
    bodyGroup.add(drawCoG(body.cog));

    // draw attachment points
    for (const point of body.attachment_points) {
        const ap = createAttachmentPoint(point);
        attachmentPoints.add(ap);
    }

    const visualGroup = new THREE.Group();
    visualGroup.position.set(...body.cog);
    bodyGroup.add(visualGroup);

    if (body.visual) {
      const visual = await drawBodyVisual(bodyGroup, body.visual);
      if (visual) {
        visualGroup.add(visual);
      }
    }

    return bodyGroup;
}

function drawCoG(cog) {
//  const axes = new THREE.AxesHelper(1.0);
  const axes = new THREE.AxesHelper(0.2);

  axes.raycast = function () {};

  if (cog) {
    axes.position.set(...cog);
  }

  return axes;
}

async function drawBodyVisual(bodyGroup, visual) {
  if (!visual) return;

  let mesh;

  if (visual.type === "box") {
    mesh = createBoxVisual(visual);
  } else if (visual.type === "cylinder") {
    mesh = createCylinderVisual(visual);
  } else if (visual.type === "stl") {
    mesh = await createSTLVisual(visual);
  }

  if (!mesh) return;

  // offset inside body
  if (visual.offset) {
    mesh.position.set(...visual.offset);
  }

  return mesh;
}

function createBoxVisual(visual) {
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

    return new THREE.Mesh(geom, mat);
}

function createCylinderVisual(visual) {
    const geom = new THREE.CylinderGeometry(
      visual.diameter/2,
      visual.diameter/2,
      visual.length,
      16
    );

    const mat = new THREE.MeshStandardMaterial({
      color: visual.color || 0x999999
    });

    const mesh = new THREE.Mesh(geom, mat);

    // align axis
    if (visual.axis === "x") {
      mesh.rotation.z = Math.PI / 2;
    } else if (visual.axis === "z") {
      mesh.rotation.x = Math.PI / 2;
    }

    return mesh
}

async function createSTLVisual(visual) {

    const geometry = await loadSTLGeometry(visual.file);

    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();

    const material =
        new THREE.MeshStandardMaterial({
//            color: 0xaaaaaa,
            color: 0xffff00,
            metalness: 0.5,
            roughness: 0.5
        });

    const mesh = new THREE.Mesh(
        geometry,
        material
    );

    // scale
    const s = visual.scale;
    mesh.scale.set(s, s, s);

    // transform
    mesh.position.set(
        ...visual.translation
    );

    const m =
        matrix4From3x3(
            visual.rotation
        );

    mesh.setRotationFromMatrix(m);

    return mesh;
}

async function loadSTLGeometry(file) {

    if (stlCache.has(file)) {
        return stlCache.get(file);
    }

    const geometry = await new Promise((resolve, reject) => {

        stlLoader.load(
            `/src/${file}`,
            resolve,
            undefined,
            reject
        );

    });

    stlCache.set(file, geometry);

    return geometry;
}

function matrix4From3x3(rotation3x3) {
  const R = rotation3x3;
  const m = new THREE.Matrix4();

  m.set(
    R[0][0], R[0][1], R[0][2], 0,
    R[1][0], R[1][1], R[1][2], 0,
    R[2][0], R[2][1], R[2][2], 0,
    0,       0,       0,       1,
  );

  return m;
}
