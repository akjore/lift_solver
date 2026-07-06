import * as THREE from "three";
import { updatePropertiesPanel } from "./properties.js";
import { highlightTreeItem } from "./tree.js";

let selectedObject = null;

export function clearSelection() {

    if (!selectedObject) {
        return;
    }

    selectedObject.traverse(child => {

        if (
            child.isMesh &&
            child.userData.originalColor !== undefined
        ) {
            child.material.color.setHex(
                child.userData.originalColor
            );
        }
    });

    selectedObject = null;
}

export function selectObject(object) {

    clearSelection();

    selectedObject = object;

    object.traverse(child => {

        if (child.isMesh && child.material?.color) {

            child.userData.originalColor =
                child.material.color.getHex();

            child.material.color.setHex(0xff6600);
        }
    });

    console.log(
        "Selected:",
        object.userData.id,
        object.userData.type
    );

    updatePropertiesPanel(object);
    highlightTreeItem(object.userData.id);
}

export function getSelectedObject() {
    return selectedObject;
}

export function findSelectable(object) {

    let current = object;

    while (current) {

        if (current.userData?.id) {
            return current;
        }

        current = current.parent;
    }

    return null;
}

export function requestZoom(object) {

    document.dispatchEvent(
        new CustomEvent(
            "zoomToObject",
            {
                detail: object
            }
        )
    );
}

function zoomSelectedObject(object) {

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
