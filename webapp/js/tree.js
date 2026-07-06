import { selectObject, requestZoom } from "./selection.js";


export function buildTree(world) {

    const bodies = document.getElementById("tree-bodies");

    const shackles = document.getElementById("tree-shackles");

    const slings = document.getElementById("tree-slings");

    bodies.innerHTML = "";
    shackles.innerHTML = "";
    slings.innerHTML = "";

    for (const object of world.pickables) {
        let container = null;
        switch (object.userData.type) {
            case "body":
                container = bodies;
                break;

            case "shackle":
                container = shackles;
                break;

            case "sling":
                container = slings;
                break;

            default:
                continue;
        }

        addTreeItem(container, object, 2);
    }
}


// function createTreeNode(object, level = 0) {
//    const item = document.createElement("div");

//    item.className = `tree-item indent-${level}`;

//    item.textContent = object.userData.id;

//    for (const child of object.children) {
//        if (child.userData?.id) {
//            createTreeNode(child, level + 1);
//        }
//    }
//}

//function addTreeGroups() {
//    for (const obj of world.pickables) {
//
//        if (obj.userData.type === "body") {
//            addTreeItem(bodiesContainer, obj);
//        }
//
//        if (obj.userData.type === "shackle") {
//            addTreeItem(shacklesContainer, obj);
//        }

//        if (obj.userData.type === "sling") {
//            addTreeItem(slingsContainer, obj);
//        }
//    }
//}

function addTreeItem(container, object) {
    const item = document.createElement("div");

    item.className = "tree-item indent-2";

    item.textContent = object.userData.id;

    item.dataset.objectId = object.userData.id;

    item.addEventListener(
        "click",
        () => selectObject(object)
    );

    item.addEventListener(
        "dblclick",
        () => {

            document.dispatchEvent(
                new CustomEvent(
                    "zoomToObject",
                    {
                        detail: object
                    }
                )
            );
        }
    );

    container.appendChild(item);
}

export function highlightTreeItem(id) {

    document
        .querySelectorAll(".tree-item")
        .forEach(item =>
            item.classList.remove(
                "selected"
            )
        );

    const item =
        document.querySelector(
            `[data-object-id="${id}"]`
        );

    if (item) {
        item.classList.add(
            "selected"
        );
    }
}
