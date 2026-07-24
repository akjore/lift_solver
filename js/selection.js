import * as THREE from "three";
import { updatePropertiesPanel } from "./properties.js";
import { highlightTreeItem } from "./tree.js";

let selectedObject = null;
let hoveredObject = null;

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
//    clearSelection();

    if (selectedObject) {
        setHighlight(selectedObject, false);
    }

    selectedObject = object;

//    object.traverse(child => {

//        if (child.isMesh && child.material?.color) {

//            child.userData.originalColor =
//                child.material.color.getHex();

//            child.material.color.setHex(0xff6600);
//        }
//    });

    setHighlight(selectedObject, true);

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

export function getHoveredObject() {
    return hoveredObject;
}

export function setHoveredObject(object) {

    if (object === hoveredObject) {
        return;
    }

    clearHover();

    hoveredObject = object;

    if (
        hoveredObject &&
        hoveredObject !== selectedObject
    ) {
//        setHighlight(
//            hoveredObject,
//            0xffff00   // yellow
//        );

        setHighlight(
            hoveredObject,
            true
        );
    }
}

export function clearHover() {

    if (
        hoveredObject &&
        hoveredObject !== selectedObject
    ) {
//        removeHighlight(
//            hoveredObject
//        );

        setHighlight(
            hoveredObject,
            false
        )

    }

    hoveredObject = null;
}

function setHighlight(object, highlighted) {
    object.traverse(child => {
        if (!child.isMesh) {
            return;
        }

        if (!child.material) {
            return;
        }

        if (highlighted) {

            if (
                child.userData.originalEmissive === undefined
            ) {
                child.userData.originalEmissive =
                    child.material.emissive?.getHex?.() ?? 0;
            }

            if (child.material.emissive) {
                child.material.emissive.setHex(0x4444ff);
            }

        } else {

            if (
                child.material.emissive &&
                child.userData.originalEmissive !== undefined
            ) {
                child.material.emissive.setHex(
                    child.userData.originalEmissive
                );
            }

        }

    });

}

