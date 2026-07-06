export function updatePropertiesPanel(object) {

    const panel = document.getElementById("properties-content");

    if (!object) {
        panel.innerHTML =
            propertyRow("Id", "None");

        return;
    }

    const data = object.userData.data;

    let properties = [];


    switch (object.userData.type) {
        case "body":
            properties = {
                Id: data.id,
                Mass: formatValue(data.mass),
                CoG: formatValue(data.cog),
                Position: formatValue(data.position)
            };
            break;
        case "shackle":
            properties = {
                Id: data.id,
                Model: data.model,
                WLL: formatValue(data.wll)
            };
            break;
        case "sling":
            properties = {
                Id: data.id,
                Rope: data.rope_kind,
                Diameter: formatValue(data.diameter)
            };
            break;
        case "attachmentPoint":
            properties = {
                Id: data.id,
                Type: data.type,
                Position: formatValue(data.position_local),
                Axis: data.axis_local
    };

    break;

    }
    panel.innerHTML = renderProperties(properties);
}

function renderProperties(properties) {

    let html = "";

    for (const [label, value] of Object.entries(properties)) {
        html += propertyRow(
            label,
            value
        );
    }

    return html;
}

function propertyRow(label, value) {
    return `
        <div class="property-row">
            <div class="property-label">${label}</div>
            <div>${value}</div>
        </div>
    `;
}

export function formatValue(value) {

    if (value == null) {
        return "";
    }

    if (
        typeof value === "object" &&
        "magnitude" in value &&
        "units" in value
    ) {
        return formatQuantity(value);
    }

    if (Array.isArray(value)) {
        return formatArray(value);
    }

    if (typeof value === "number") {
        return formatNumber(value);
    }

    return String(value);
}

function formatNumber(value) {

    return Number(value.toFixed(3)).toString();

}

function formatArray(values) {

    return `[${values
        .map(v => formatNumber(v))
        .join(", ")}]`;

}

function formatQuantity(quantity) {

    const {
        magnitude,
        units
    } = quantity;

    const formattedMagnitude =
        Array.isArray(magnitude)
            ? formatArray(magnitude)
            : formatNumber(magnitude);

    return `${formattedMagnitude} ${units}`;
}
