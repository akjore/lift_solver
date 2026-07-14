"""Perform code check of rigging components."""
import logging
from pathlib import Path
from typing import Self

import numpy as np
import pint

from .results import Results
from .sling import Sling, RopeKinds
from .shackle import Shackle
from .lift_problem import LiftProblem

# Create logger
logger = logging.getLogger(__name__)

# Note:
# forces: include masses and factors
# limits: mbls over safety factor

def angle_as_percent(angle: float) -> float:
    """Convert input angle in degrees to %."""
    return np.tan(np.radians(angle)) * 100

JINJA_TEMPLATES_FOLDER_NAME = "templates"
RIGGING_REPORT_TEMPLATE_FILE_NAME = "rigging_report_template.jinja2"

class CodeCheck():
    """Perform code check according to DNV-ST-N001."""

    DEFAULTS = {
        "k_skl": 1.0,
        "gamma_h": 1.3,
        "gamma_c": 1.3,
        "daf": 1.3,
        "gamma_w": 1.0,
        "weight_contingency": 1.0,
        "cog_contingency": 1.0,
    }

    # Default termination factors.
    DEFAULT_GAMMA_S = {
        RopeKinds.IWRC.name: 1.25,          # DNV-ST-N001 Dec 2023, Table 16-3, hand splice
        RopeKinds.CABLE.name: 1.33,         # DNV-ST-N001 Dec 2023, Table 16-3, hand splice
        RopeKinds.HMPE.name: 1.0,           # DNV-ST-N001 Dec 2023, Table 16-3, testing with actual termination
    }

    # Default material factors.
    DEFAULT_GAMMA_M = {
        RopeKinds.IWRC.name: 1.35,          # DNV-ST-N001 Dec 2023, 16.4.9.2
        RopeKinds.CABLE.name: 1.35,         # DNV-ST-N001 Dec 2023, 16.4.9.2
        RopeKinds.HMPE.name: 2.0,           # DNV-ST-N001 Dec 2023, 16.4.9.3 -> 5.9.8.5.2
    }

    def __init__(self: Self, simulation_results: Results, problem: LiftProblem) -> None:
        """Create a rigging analysis object."""
        self.settings = problem.code_check_settings
        self.problem = problem
        self.simulation_results = simulation_results
        self.g = np.linalg.norm(problem.g)

        self.code_check_results = {}



    def results(self: Self) -> dict:
        for id, sling in self.simulation_results.slings.items():
            self.code_check_results[id] = self.check_sling(sling, self.problem.slings[id])

        for id, shackle in self.simulation_results.shackles.items():
            self.code_check_results[id] = self.check_shackle(shackle, self.problem.shackles[id])

        return self.code_check_results



    def check_sling(self: Self, sling_results: dict, sling: Sling):
        """Check sling loading against capacity."""
        settings = self.get_settings(sling.id)

        f_sd = sling_results["tension"] * settings["daf"] * settings["k_skl"] * settings["weight_contingency"] * \
                settings["cog_contingency"] / self.g

        # Check bending of sling body
        gamma_b_body = 1.0
        if sling.sheaves:
            D = min([sheave.diameter for sheave in sling.sheaves])
            gamma_b_body = self.gamma_b(sling.diameter, D)

        # Check bending of sling eyes
        # Half the factor used, as twice the MBL may be considered, Ref. DNV-ST-N001, 16.4.8.4, Guidance note 2
        D = min([sling.end_a.diameter, sling.end_b.diameter])
        gamma_b_eye = self.gamma_b(sling.diameter, D) / 2

        # Governing bending factor
        gamma_b = max(gamma_b_body, gamma_b_eye)

        gamma_r = max(settings["gamma_s"], gamma_b)

        gamma_sf_1 = settings["gamma_h"] * settings["gamma_c"] * gamma_r * settings["gamma_w"] * settings["gamma_m"]
        gamma_sf_2 = 2.3 * gamma_r * settings["gamma_w"]
        gamma_sf = max(gamma_sf_1, gamma_sf_2)

        return {
            "id": sling.id,
            "f_sd": f_sd,
            "daf": settings["daf"],
            "k_skl": settings["k_skl"],
            "weight_contingency": settings["weight_contingency"],
            "cog_contingecy": settings["cog_contingency"],
            "gamma_b": gamma_b,
            "gamma_s": settings["gamma_s"],
            "gamma_r": gamma_r,
            "gamma_sf": gamma_sf,
            "utilisation": f_sd * gamma_sf / sling.mbl,
        }


    def gamma_b(self: Self, d: pint.Quantity, D: pint.Quantity) -> float:
        """Calculate gamma_b for a rope of diameter d bent over a bend of diameter D."""
        return 1 / (1 - 0.5/(D/d)**0.5)


    def check_shackle(self: Self, shackle_results: Results, shackle: Shackle):
        """Check shackle loading against capacity."""
        settings = self.get_settings(shackle.id)

        load = max(shackle_results["pin_force"], shackle_results["bow_force"])
        f_static = load * settings["k_skl"] * settings["weight_contingency"] * settings["cog_contingency"] / self.g
        f_dynamic = f_static * settings["daf"]

        # Static utilisation
        ur_static = f_static / shackle.wll

        # Dynamic utilisation
        ur_dynamic_1 = f_dynamic / (shackle.mbl / 3)
        #ur_dynamic_2 = f_dynamic / shackle.proof_load

        return {
            "id": shackle.id,
            "f_static": f_static.to_base_units(),
            "f_dynamic": f_dynamic.to_base_units(),
#            "ur": max(ur_static, ur_dynamic_1, ur_dynamic_2),
            "ur": max(ur_static, ur_dynamic_1).to_base_units(),
        }


    def get_settings(self, component_id=None):

        # Use the defaults in nothing else is specified
        settings = self.DEFAULTS.copy()

        # Override defaults with problem-level overrides
        for key, value in self.settings.items():

            if not isinstance(value, dict):
                settings[key] = value

        # Override problem-level overrides with any component overrides
        if component_id:
            settings.update(
                self.settings.get(
                    component_id,
                    {}
                )
            )

        # Additional sling settings
        sling = self.problem.slings.get(component_id)
        if sling:
            # gamma_s
            settings["gamma_s"] = self.DEFAULT_GAMMA_S[self.problem.slings[component_id].kind.name]

            # gamma_m
            settings["gamma_m"] = self.DEFAULT_GAMMA_M[self.problem.slings[component_id].kind.name]

        return settings














    @property
    def static_hook_load(self: Self) -> float:
        """Return the static hook load."""
        # If more than one hook present, consider only the first one.
        hoist_mass = sum([hoist.mass for hoist in self.hoists])
        hook_load = -self._scene[self.hoists[0].name].connection_force[2]/self._scene.g
        hook_load -= hoist_mass if hook_load > 0 else 0
        return hook_load

    @property
    def dynamic_hook_load(self: Self) -> float:
        """Return the dynamic hook load, i.e. static hook load with DAF."""
        return self.static_hook_load * self.daf



    def export_report(self: Self, title: str|None=None, filename: str|None=None) -> None:
        """Export report to html file."""
        from jinja2 import Environment, FileSystemLoader, select_autoescape

        # Find and load template
        p = Path(__file__).parent.parent.parent.parent / JINJA_TEMPLATES_FOLDER_NAME
        jinja_env = Environment(loader=FileSystemLoader(p), autoescape=select_autoescape(["html"]))
        jinja_env.add_extension("jinja2.ext.do")
        template = jinja_env.get_template(RIGGING_REPORT_TEMPLATE_FILE_NAME)

        # If no title or filename set, set defaults
        outfile = None
        modelfilename = self._scene.gui_solve_func.__self__.modelfilename
        if not filename and modelfilename:
            p2 = Path(modelfilename)
            outfile = p2.parent / (p2.stem + ".html")
        elif filename:
            outfile = Path(self._scene.current_directory) / Path(filename)
        else:
            outfile = Path(self._scene.current_directory) / "default_rigging_report.html"

        if (not title) and modelfilename:
            title = outfile.stem
        elif not title:
            title = ""

        template_vars = {
            "title" : title,
            "rigging_analysis": self,
            "angle_as_percent": angle_as_percent,
        }
        html_out = template.render(template_vars)

        with outfile.open("w") as fh:
            fh.write(html_out)
