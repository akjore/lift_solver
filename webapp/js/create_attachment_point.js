import * as THREE from "three";

export function createAttachmentPoint(ap) {

    const group = new THREE.Group();

    group.name = ap.id;
    group.userData = ap;

    group.userData = {
        id: ap.id,
        type: "attachmentPoint",
        data: ap,
        fit: true,
    };

    switch (ap.type ?? "unknown") {
        case "padeye":
            group.add(createPadeye(ap));
            break;

        case "pin":
            group.add(createPin(ap));
            break;

        default:
            group.add(createSphere());
            break;
    }

    group.position.set(...ap.position_local.magnitude);

    return group;
}

function createPadeye(ap) {

    const rOuter = ap.outer_diameter.magnitude * 0.5;
    const rInner = ap.hole_diameter.magnitude * 0.5;

    const shape = new THREE.Shape();
    shape.absarc(0, 0, rOuter, 0, Math.PI * 2);

    const hole = new THREE.Path();
    hole.absarc(0, 0, rInner, 0, Math.PI * 2);

    shape.holes.push(hole);

    const geometry = new THREE.ExtrudeGeometry(shape, {
        depth: ap.thickness.magnitude,
        bevelEnabled: false
    });

    // Centre thickness about local origin
    geometry.translate(
        0,
        0,
        -ap.thickness.magnitude / 2
    );

    const material = new THREE.MeshStandardMaterial({
//        color: 0x8080ff,
        color: 0x00ffff,
        transparent: true,
        opacity: 0.6
    });

    const mesh = new THREE.Mesh(
        geometry,
        material
    );

    alignToAxis(mesh, ap.axis_local, new THREE.Vector3(0, 0, 1));

    return mesh;
}

function createPin(ap) {

    const geometry = new THREE.CylinderGeometry(
        ap.diameter.magnitude * 0.5,
        ap.diameter.magnitude * 0.5,
        ap.length.magnitude
    );

    const material = new THREE.MeshStandardMaterial({
        color: 0xffaa00
    });

    const mesh = new THREE.Mesh(
        geometry,
        material
    );

    // Cylinder is Y-aligned in Three.js
    alignToAxis(mesh, ap.axis_local, new THREE.Vector3(0, 1, 0));

    return mesh;
}

function createSphere() {

    return new THREE.Mesh(
//        new THREE.SphereGeometry(0.15, 16, 16),
        new THREE.SphereGeometry(0.03, 16, 16),
        new THREE.MeshBasicMaterial({
            color: 0x00ffff
        })
    );
}

function alignToAxis(object, axis, sourceAxis) {

    const target = new THREE.Vector3(...axis)
        .normalize();

    object.quaternion.setFromUnitVectors(
        sourceAxis,
        target
    );
}