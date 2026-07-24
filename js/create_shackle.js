import * as THREE from "three";
import { createGenericBody } from "./create_body.js";

export async function createShackle(shackle) {
    // create groups
    const shackleGroup = new THREE.Group();
    shackleGroup.name = shackle.id;

    shackleGroup.userData = {
        id: shackle.id,
        type: "shackle",
        data: shackle
    };

  createGenericBody(shackle, shackleGroup);

  return shackleGroup;
}
