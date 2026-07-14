"""Class for holding simulation results."""
from typing import Self

import numpy as np
import pint

class Results:

    def __init__(self: Self) -> None:
        self.bodies = {}
        self.shackles = {}
        self.slings = {}
        self.attachment_points = {}
        self.constraints = {}


    def export_initial_state(self: Self) -> str:
        """
        Export solver state into YAML-ready initial_state block.

        Rules:
        - parent=None  → export absolute pose
        - parent!=None → export pose relative to parent
        """

        lines = []
        lines.append("initial_state:")
        lines.append("  # format: [x, y, z, roll, pitch, yaw]")
        lines.append("  #")
        lines.append("  # IMPORTANT:")
        lines.append("  # - Bodies WITHOUT a parent are absolute (global)")
        lines.append("  # - Bodies WITH a parent are relative to their parent")
        lines.append("")

        for id, object in (self.bodies | self.shackles).items():

            # Process a root element - use absolute values
            position = object["position_global"] if object["position_relative"] is None else object["position_relative"]
            euler = object["euler_global"] if object["euler_relative"] is None else object["euler_relative"]

            values = [
                f"{position[0]:.8g}",
                f"{position[1]:.8g}",
                f"{position[2]:.8g}",
                f"{euler[0]:.8g}",
                f"{euler[1]:.8g}",
                f"{euler[2]:.8g}",
            ]

            values_str = ", ".join(values)
            lines.append(f"  {id}: [{values_str}]")

        return "\n".join(lines)


    def to_render_model(self: Self) -> dict:
        return {
            "bodies": [cnv_quantity(obj) for obj in self.bodies.values()],
            "shackles": [cnv_quantity(obj) for obj in self.shackles.values()],
            "slings": [cnv_quantity(obj) for obj in self.slings.values()]
        }


def cnv_quantity(value) -> dict:
    if isinstance(value, dict):
        return {
            k: cnv_quantity(v)
            for k, v in value.items()
        }

    if isinstance(value, (list, tuple)):
        return [
            cnv_quantity(v)
            for v in value
        ]

    if isinstance(value, np.ndarray):
        return value.tolist()

    if isinstance(value, pint.Quantity):
        val = value
        if val.check("[length]"):
            val.ito("m")
        elif value.check("[mass]"):
            val.ito("kg")
        elif value.check("[]"):
            val.ito("rad")

        mag = val.magnitude
        unit = val.units
        if isinstance(mag, np.ndarray):
            mag = mag.tolist()

        return {
            "magnitude": mag,
            "units": f"{unit:~P}",
        }

    return value

