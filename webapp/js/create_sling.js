import * as THREE from "three";

const ROPE_MATERIALS = {
    IWRC: {
//        color: 0xb8bcc2,
        color: 0xB0B0B0,
        roughness: 0.75,
        metalness: 0.35
    },
    CABLE: {
//        color: 0xd0d2d4,
        color: 0xE0E0E0,
        roughness: 0.60,
        metalness: 0.45
    },
    HMPE: {
//        color: 0xf2f2f0,
        color: 0xFFFFFF,
        roughness: 0.95,
        metalness: 0.0
    }
};

export function createSling(sling, world) {

    const slingGroup = new THREE.Group();

    slingGroup.name = sling.id;

    slingGroup.userData = {
        id: sling.id,
        type: "sling",
        data: sling
    };

    const points = [];

    // Start point
    points.push(
        getAttachmentPointWorldPosition(
            sling.end_a.id,
            world
        )
    );

    // Future sheaves
    for (const sheave of sling.sheaves ?? []) {
        points.push(
            new THREE.Vector3(...sheave.position.magnitude)
        );
    }

    // End point
    points.push(
        getAttachmentPointWorldPosition(
            sling.end_b.id,
            world
        )
    );

    const curve = new THREE.CatmullRomCurve3(points);

    const geometry = new THREE.TubeGeometry(
        curve,
        50,                         // segments
        sling.diameter.magnitude / 2,
        16,                         // radial segments
        false                       // closed
    );


    const matProps = ROPE_MATERIALS[sling.rope_kind] ?? ROPE_MATERIALS.IWRC;

    const material = new THREE.MeshStandardMaterial(matProps);

    const tube = new THREE.Mesh(
        geometry,
        material
    );

    slingGroup.add(tube);

    return slingGroup;
}

function getAttachmentPointWorldPosition(
    id,
    world
) {

    const object = world.objectMap.get(id);

    if (!object) {

        console.error(`Cannot find attachment point: ${id}`);

        return new THREE.Vector3();
    }

    const p = new THREE.Vector3();

    object.getWorldPosition(p);

    return p;
}