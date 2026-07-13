"""Class for holding simulation results."""
from typing import Self


class Results:

    def __init__(self: Self):
        self.bodies = {}
        self.shackles = {}
        self.slings = {}
        self.attachment_points = {}
        self.initial_state = None
        self.constraints = {}


    def export_initial_state(self: Self):
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
