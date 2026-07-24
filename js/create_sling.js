import * as THREE from "three";

const ROPE_MATERIALS = {
    IWRC: {
        color: 0xB0B0B0,
        roughness: 0.75,
        metalness: 0.35
    },

    CABLE: {
        color: 0xE0E0E0,
        roughness: 0.60,
        metalness: 0.45
    },

    HMPE: {
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

    const ropeRadius =
        sling.diameter.magnitude / 2;

    const matProps =
        ROPE_MATERIALS[sling.rope_kind]
        ?? ROPE_MATERIALS.IWRC;

    const material =
        new THREE.MeshStandardMaterial(
            matProps
        );

    const startPoint =
        getAttachmentPointWorldPosition(
            sling.end_a.id,
            world
        );

    const endPoint =
        getAttachmentPointWorldPosition(
            sling.end_b.id,
            world
        );

    let bodyStart =
        startPoint.clone();

    let bodyEnd =
        endPoint.clone();

    //
    // Eye A
    //

    if (hasRenderableEye(sling.eye_a)) {

        const slingAxis =
            endPoint.clone()
                .sub(startPoint)
                .normalize();

        const eye =
            buildEye(
                sling.end_a,
                slingAxis,
                sling.eye_a,
                ropeRadius,
                material
            );

        if (eye) {

            slingGroup.add(
                eye.group
            );

            bodyStart =
                eye.spliceEnd.clone();
        }
    }

    //
    // Eye B
    //

    if (hasRenderableEye(sling.eye_b)) {

        const slingAxis =
            startPoint.clone()
                .sub(endPoint)
                .normalize();

        const eye =
            buildEye(
                sling.end_b,
                slingAxis,
                sling.eye_b,
                ropeRadius,
                material
            );

        if (eye) {

            slingGroup.add(
                eye.group
            );

            bodyEnd =
                eye.spliceEnd.clone();
        }
    }

    //
    // Main body
    //

    const points = [];

    points.push(bodyStart);

    for (const sheave of sling.sheaves ?? []) {

        points.push(
            new THREE.Vector3(
                ...sheave.position.magnitude
            )
        );
    }

    points.push(bodyEnd);

    const curve =
        new THREE.CatmullRomCurve3(
            points
        );

    const geometry =
        new THREE.TubeGeometry(
            curve,
            50,
            ropeRadius,
            16,
            false
        );

    const tube =
        new THREE.Mesh(
            geometry,
            material
        );

    slingGroup.add(tube);

    return slingGroup;
}

function hasRenderableEye(eye) {

    return (
        eye &&
        eye.separation_angle &&
        eye.apex_offset
    );
}

function buildEye(
    attachmentPoint,
    slingAxis,
    eye,
    ropeRadius,
    material
) {

    const geom =
        buildEyeGeometry(
            attachmentPoint,
            slingAxis,
            eye,
            ropeRadius
        );

    if (!geom) {
        return null;
    }

    const group =
        new THREE.Group();

    const arcPoints = [];

    const n = 16;

    const arcSweep =
        2 * Math.PI -
        geom.separationAngle;

    const startAngle =
        geom.separationAngle / 2;

    for (let i = 0; i <= n; i++) {

        const angle =
            startAngle +
            (i / n) * arcSweep;

        const dir =
            geom.radial.clone()
                .applyAxisAngle(
                    geom.pinDirection,
                    angle
                );

        arcPoints.push(
            geom.pinCenter.clone().add(
                dir.multiplyScalar(
                    geom.effectiveRadius
                )
            )
        );
    }

    group.add(
        createTube(
            arcPoints,
            ropeRadius,
            material
        )
    );

    //
    // Eye leg 1
    //

    group.add(
        createTube(
            [
                geom.sep1,
                geom.apex
            ],
            ropeRadius,
            material
        )
    );

    //
    // Eye leg 2
    //

    group.add(
        createTube(
            [
                geom.sep2,
                geom.apex
            ],
            ropeRadius,
            material
        )
    );

    //
    // Splice
    //

    group.add(
        createTube(
            [
                geom.apex,
                geom.spliceEnd
            ],
            ropeRadius * Math.sqrt(2),
            material
        )
    );

    return {

        group,

        spliceEnd:
            geom.spliceEnd.clone()

    };
}

function buildEyeGeometry(
    attachmentPoint,
    slingAxis,
    eye,
    ropeRadius
) {

    const pinDiameter =
        attachmentPoint.diameter?.magnitude;

    const pinAxis =
        attachmentPoint.axis_global;

    if (
        !pinDiameter ||
        !pinAxis ||
        !eye ||
        !eye.separation_angle ||
        !eye.apex_offset
    ) {
        return null;
    }

    const pinRadius =
        pinDiameter / 2;

    const effectiveRadius = pinRadius + ropeRadius

    const pinCenter =
        new THREE.Vector3(
            ...attachmentPoint
                .position_global
                .magnitude
        );

    const slingDirection =
        slingAxis.clone()
            .normalize();

    const pinDirection =
        new THREE.Vector3(
            ...pinAxis
        ).normalize();

    const separationAngle =
        eye.separation_angle
            .magnitude;

    const apexOffset =
        eye.apex_offset
            .magnitude;

    const spliceLength =
        eye.length_splice
            .magnitude;

    //
    // Apex is on sling axis
    //

    const apex =
        pinCenter.clone().add(
            slingDirection
                .clone()
                .multiplyScalar(
                    apexOffset
                )
        );

    //
    // Direction from pin centre
    // toward apex
    //

    const radial =
        apex.clone()
            .sub(pinCenter)
            .normalize();

    const halfAngle =
        separationAngle / 2;

    const sepDir1 =
        radial.clone()
            .applyAxisAngle(
                pinDirection,
                halfAngle
            );

    const sepDir2 =
        radial.clone()
            .applyAxisAngle(
                pinDirection,
                -halfAngle
            );

    const sep1 =
        pinCenter.clone().add(
            sepDir1.multiplyScalar(
                effectiveRadius
            )
        );

    const sep2 =
        pinCenter.clone().add(
            sepDir2.multiplyScalar(
                effectiveRadius
            )
        );

    const spliceEnd =
        apex.clone().add(
            slingDirection
                .clone()
                .multiplyScalar(
                    spliceLength
                )
        );

    return {

        pinCenter,

        sep1,
        sep2,

        apex,

        spliceEnd,

        separationAngle,
        radial,
        pinDirection,
        effectiveRadius
    };
}

function createTube(
    points,
    radius,
    material
) {

    const curve =
        new THREE.CatmullRomCurve3(
            points
        );

    const geometry =
        new THREE.TubeGeometry(
            curve,
            16,
            radius,
            16,
            false
        );

    return new THREE.Mesh(
        geometry,
        material.clone()
    );
}

function getAttachmentPointWorldPosition(
    id,
    world
) {

    const object =
        world.objectMap.get(id);

    if (!object) {

        console.error(
            `Cannot find attachment point: ${id}`
        );

        return new THREE.Vector3();
    }

    const p =
        new THREE.Vector3();

    object.getWorldPosition(p);

    return p;
}