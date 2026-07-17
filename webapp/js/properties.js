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
                Position: formatValue(data.position),
                Orientation: formatValue(data.rotation_euler),
            };
            break;
        case "shackle":
            properties = {
                Id: data.id,
                Model: data.model,
                Manufacturer: data.manufacturer,
                WLL: formatValue(data.wll),
                "Safety factor": data.safety_factor,
                Mass: formatValue(data.mass),
                "Pin diameter": formatValue(data.pin_diameter),
                "Bow diameter": formatValue(data.bow_diameter),
                "Inside length": formatValue(data.inside_length),
                Type: data.sub_type
            };
            break;
        case "sling":
            properties = {
                Id: data.id,
                Rope: data.rope_kind,
                Diameter: formatValue(data.diameter),
                EA: formatValue(data.ea),
                k: formatValue(data.k),
                "Ultimate length": formatValue(data.ultimate_lenght),
                Mass: formatValue(data.mass),
                MBL: formatValue(data.mbl)
            };
            break;
        case "attachmentPoint":
            properties = {
                Id: data.id,
                Type: data.type,
                Position: formatValue(data.position_local),
                Axis: data.axis_local,
                Diameter: formatValue(data.diameter),
                Length: formatValue(data.length),
                "Hole dia": formatValue(data.hole_diameter),
                "Outer dia": formatValue(data.outer_diameter),
                Thickness: formatValue(data.thickness)
            };
            break;
    }

    properties = Object.fromEntries(
        Object.entries(properties)
            .filter(([_, value]) =>
                value !== undefined &&
                value !== null &&
                value !== ""
            )
        );

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
