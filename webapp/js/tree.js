import { selectObject, requestZoom } from "./selection.js";

export function buildTree(world) {
    const container = document.getElementById("tree-content");
    container.innerHTML = "";

    const treeModel = buildTreeModel(world);
    container.appendChild(createTreeNode(treeModel, 0));
}

function buildTreeModel(world) {
    return {
        id: "model",
        label: "Model",

        children: [
            {
                id: "bodies",
                label: "Bodies",
                children: [...world.bodies.children.map(buildObjectNode)]
            },
            {
                id: "shackles",
                label: "Shackles",
                children: [...world.shackles.children.map(buildObjectNode)
                ]
            },
            {
                id: "slings",
                label: "Slings",
                children: [...world.slings.children.map(buildObjectNode)]
            }

        ]
    };
}

function buildObjectNode(object) {
    const node = {
        id:
            object.userData?.id ??
            object.name,
        label:
            object.userData?.id ??
            object.name,
        object,

        children: []

    };

    collectChildren(object, node);

    return node;
}


function createTreeNode(node, level = 0) {

    const container = document.createElement("div");
    const header = document.createElement("div");
    header.className = "tree-item";
    header.style.paddingLeft = `${level * 16}px`;

    // -----------------
    // Arrow
    // -----------------
    const hasChildren = node.children && node.children.length > 0;

    const arrow = document.createElement("span");
    arrow.className = "tree-arrow";
    arrow.textContent = hasChildren ? "▶" : "";

    header.appendChild(arrow);

    // -----------------
    // Label
    // -----------------
    const label = document.createElement("span");
    label.textContent = node.label;

    header.appendChild(label);

    // -----------------
    // Selection
    // -----------------
    if (node.object) {
        header.dataset.objectId = node.object.userData.id;

        header.addEventListener(
            "click",
            event => {

                event.stopPropagation();

                selectObject(
                    node.object
                );

            }
        );

        header.addEventListener(
            "dblclick",
            event => {

                event.stopPropagation();

                document.dispatchEvent(
                    new CustomEvent(
                        "zoomToObject",
                        {
                            detail:
                                node.object
                        }
                    )
                );

            }
        );
    }

    container.appendChild(header);

    // -----------------
    // Children
    // -----------------
    if (hasChildren) {
        const childContainer = document.createElement("div");
        childContainer.className = "tree-section";
        childContainer.hidden = true;

        arrow.addEventListener(
            "click",
            event => {

                event.stopPropagation();

                childContainer.hidden =
                    !childContainer.hidden;

                arrow.textContent =
                    childContainer.hidden
                        ? "▶"
                        : "▼";

            }
        );

        node.children.forEach(child => {
            childContainer.appendChild(
                createTreeNode(
                    child,
                    level + 1
                )
            );

        });

        container.appendChild(
            childContainer
        );
    }

    return container;
}

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
    revealTreeItem(id);

    document
        .querySelectorAll(".tree-item")
        .forEach(item =>
            item.classList.remove(
                "selected"
            )
        );

    const item = document.querySelector(
            `[data-object-id="${id}"]`
        );

    if (item) {
        item.classList.add(
            "selected"
        );
    }
}

function initializeTree() {
    document
        .querySelectorAll(".tree-section-header")
        .forEach(header => {

            header.addEventListener(
                "click",
                () => toggleSection(header)
            );

        });
}

function toggleSection(header) {

    const targetId = header.dataset.target;

    const section = document.getElementById(targetId);

    if (!section) {
        console.warn(`Tree section '${targetId}' not found`);
        return;
    }

    const expanded = !section.hidden;

    section.hidden = expanded;


    const arrow = header.querySelector(".tree-arrow");
    arrow.textContent = expanded ? "▶" : "▼";
}

export function revealTreeItem(id) {

    const item = document.querySelector(
        `[data-object-id="${id}"]`
    );

    if (!item) {
        return;
    }

    let current = item.parentElement;

    while (current) {

        if (current.hidden) {
            current.hidden = false;
        }

        current = current.parentElement;
    }
}

function collectChildren(object, node) {

    for (const child of object.children) {

        if (child.userData?.id) {

            node.children.push(
                buildObjectNode(child)
            );

        } else {

            collectChildren(
                child,
                node
            );

        }

    }
}

export function initializeVisibility(world) {
    document
        .getElementById("show-bodies")
        .addEventListener("change", e => {

            world.bodies.visible =
                e.target.checked;

        });

    document
        .getElementById("show-shackles")
        .addEventListener("change", e => {

            world.shackles.visible =
                e.target.checked;

        });

    document
        .getElementById("show-slings")
        .addEventListener("change", e => {

            world.slings.visible =
                e.target.checked;

        });

    const header = document.getElementById("display-header");
    const section = document.getElementById("display-section");
    const arrow = document.getElementById("display-arrow");

//    header.addEventListener(
//        "click",
//        () => {
//            section.hidden = !section.hidden;
//            arrow.textContent =
//                section.hidden
//                    ? "▶"
//                    : "▼";
//        }
//    );
    arrow.addEventListener(
        "click",
        event => {

            event.stopPropagation();

            toggleDisplay(section, arrow);

        }
    );
}

function toggleDisplay(section, arrow) {

    section.hidden = !section.hidden;

    arrow.textContent = section.hidden ? "▶" : "▼";
}