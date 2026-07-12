"""Module for handling ropes, slings, and grommets."""
import logging
import math
from enum import Enum
from typing import Self

import numpy as np
import pint
from scipy.optimize import newton

from . import ureg
from .attachment_point import AttachmentPoint

logger = logging.getLogger(__name__)


class RopeKinds(Enum):
    """Types of ropes that are recognized."""

    IWRC = "Independent wire rope core"
    CABLE = "Cable laid wire rope"
    HMPE = "High modulus polyethylene fibre"


class Rope:
    """Parent class for slings and grommets."""

    # Cable laid slings - single leg - diameter and WLL from NS-EN 13414-3:2003+A1:2008, Annex G.6
    _CABLE_DIAMETER_MM = np.array([24, 27, 30, 33, 36, 39, 42, 48, 54, 60, 66, 72, 78, 84, 90, 96, 102, 108, 114, 120,
                                   126,132, 144, 150, 156, 162, 168, 174, 180, 192, 204, 216, 228, 240, 252, 264, 276,
                                   288, 300,312, 336, 360, 384, 408, 432, 456, 480, 504, 528, 552, 576, 600, 624, 648,
                                   672, 696]) * ureg.millimeter
    _CABLE_WLL_t = np.array([3.35, 4.25, 5.5, 7, 8, 9.5, 11, 14.5, 18, 22.5, 28, 34, 41, 49, 58, 68, 79, 92, 106, 122,
                             139, 158, 204, 230, 250, 270, 290, 315, 335, 410, 460, 510, 555, 610, 665, 720, 780, 840,
                             900, 970, 1100, 1250, 1400, 1550, 1700, 1880, 2050, 2250, 2450, 2600, 2800, 3000, 3200,
                             3400, 3650, 3850]) * ureg.ton
    _SF = 6.33 - 0.022 * _CABLE_DIAMETER_MM.to("mm").magnitude
    _CABLE_MBL_t = _CABLE_WLL_t * _SF


    def __init__(self: Self, id: str) -> None:
        """Initialise rope object."""
        self.id = id
        self.d = 0 * ureg.millimeter
        self.ea = 0 * ureg.newton
        self.rope_kind = None


    def _estimate_mbl(self: Self, kind: RopeKinds, diameter: float) -> float:
        mbl = None
        if kind == RopeKinds.IWRC and diameter:
            mbl = 0.064 * diameter**2 * 1e6
        elif kind == RopeKinds.CABLE and diameter:
            mbl = np.interp(diameter, self._CABLE_DIAMETER_MM, self._CABLE_MBL_t)
        elif kind == RopeKinds.HMPE and diameter:
            mbl = 0.064 * diameter**2 * 1e6
        return mbl


    def _estimate_diameter(self: Self, kind: RopeKinds, mbl: float) -> float:
        diameter = None
        if kind in (RopeKinds.IWRC, RopeKinds.HMPE) and mbl:
            diameter = (mbl / 0.064) ** 0.5 / 1000
        elif kind == RopeKinds.CABLE and mbl:
            diameter = np.interp(mbl, self._CABLE_MBL_t, self._CABLE_DIAMETER_MM)
        return diameter


    def _estimate_area(self: Self, kind: RopeKinds, diameter: float) -> float:
        area = None
        if diameter and kind in (RopeKinds.IWRC, RopeKinds.CABLE, RopeKinds.HMPE):
            area = 0.68 * math.pi / 4 * diameter**2
        return area


    def _estimate_ea(self: Self, kind: RopeKinds, area: float, mbl: float) -> float:
        ea = None
        if kind == RopeKinds.IWRC and area:
            ea = 128e6 * area
        elif kind == RopeKinds.CABLE and area:
            ea = 0.6 * 128e6 * area
        elif kind == RopeKinds.HMPE and mbl:
            ea = 30 * 9.81 * ureg("m/s/s") * mbl.to("t")
        return ea


    def _estimate_mass_per_length(self: Self, kind: RopeKinds, area: float, diameter: float) -> float:
        mass_per_length = None
        if kind in (RopeKinds.IWRC, RopeKinds.CABLE) and area:
            mass_per_length = 7.850 * area
        elif kind == RopeKinds.HMPE and diameter:
            mass_per_length = 0.000419 * diameter**2
        return mass_per_length


    @property
    def diameter(self: Self) -> float:
        """Rope diameter."""
        return self._diameter

    @diameter.setter
    def diameter(self: Self, value: float) -> None:
        self._diameter = value


    @property
    def ea(self: Self) -> float:
        """Rope EA."""
        return self._ea

    @ea.setter
    def ea(self: Self, value: float) -> None:
        self._ea = value


    @property
    def mass_per_length(self: Self) -> float:
        """Mass per length of rope."""
        return self._mass_per_length

    @mass_per_length.setter
    def mass_per_length(self: Self, value: float) -> None:
        self._mass_per_length = value

    def to_dict(self: Self) -> dict:
        return {
            "id": self.id,
            "diameter": self.diameter,
        }


class Sling(Rope):
    """A class representing a sling sling (eyes at either end)."""

    def __init__(self: Self, id: str, ap1: str | AttachmentPoint=None,
                 ap2: str | AttachmentPoint=None, diameter: float | None = None, ea: float | None=None,
                 k: float | None = None, Lultimate: float | None = None, mass: float | None = None,
                 mass_per_length: float | None = None, length_eye_a: float | None = None,
                 length_eye_b: float | None = None, length_splice_a: float | None = None,
                 length_splice_b: float | None = None, sheaves: list | None = None, mbl: float | None = None,
                 kind: RopeKinds | str = RopeKinds.IWRC, **kwargs: dict) -> None:
        """Create a sling object.

        mass_per_length refers to the base rope used to make the sling, i.e. is not mass/ultimate_length.

        There is some redundancy to cater for different preferences when specifying properties:
            mass / length / mass per length
            k / EA / L

        Lultimate: bearing-to-bearing length with (theoretically) 0 mm diameter pins in the eyes.

        Generally, from first principles:
            sigma = E * epsilon
            epsilon = delta_L / L
            F = sigma * A = EA * epsilon = EA * delta_L / L = k * delta_L where k=EA/L
        Note: if EA is provided, it is assumed that it refers to the base rope. The length used when calculating k
        is the length of the rope including twice the length of the eyes and twice the length of the splice.

        If k is provided, the reference length is the bearing-bearing length of the sling.
        """
        Rope.__init__(self, id=id)

        self.kind = kind if isinstance(kind, RopeKinds) else RopeKinds[kind]

        self.end_a = ap1
        self.end_b = ap2

        self.sheaves = sheaves if sheaves else []

        # Calculate sensible defaults for parameters that are not provided
        #   diameter may be estimated from mbl, or mbl from diameter

        self.diameter = diameter if diameter else self._estimate_diameter(self.kind, mbl)
        self.area = self._estimate_area(self.kind, self.diameter)
        self.mbl = mbl if mbl else self._estimate_mbl(self.kind, self._diameter)

        self.length_eye_a = length_eye_a if length_eye_a else self._estimate_eye_length()
        self.length_eye_b = length_eye_b if length_eye_b else self._estimate_eye_length()

        self.length_splice_a = length_splice_a if length_splice_a else self._estimate_eye_splice()
        self.length_splice_b = length_splice_b if length_splice_b else self._estimate_eye_splice()

        self.l_ultimate = Lultimate

        #   stiffness may either be specified by ea or k
        if ea:
            self.ea = ea
        if k:
            self.ea = k*(self._length_eye_a + self._length_eye_b + self._length_splice_a/2 + self._length_splice_b/2 +
                          self._l_body)
        if not (ea or k):
            self.ea = self._estimate_ea(self.kind, self.area, self.mbl)

        if mass_per_length:
            self.mass_per_length = mass_per_length
        if mass:
            self.mass_per_length = mass / self.rope_length()
        if not (mass_per_length or mass):
            self.mass_per_length = self._estimate_mass_per_length(self.kind, self.area, self.diameter)


    @property
    def mass(self: Self) -> float:
        """Mass of sling."""
        return self.mass_per_length * self.rope_length()

    @mass.setter
    def mass(self: Self, value: float) -> None:
        self.mass_per_length = value/self.rope_length()


    @property
    def length_splice_a(self: Self) -> float:
        """Length of splice at end a."""
        return self._length_splice_a

    @length_splice_a.setter
    def length_splice_a(self: Self, value: float) -> None:
        self._length_splice_a = value


    @property
    def length_splice_b(self: Self) -> float:
        """Length of splice at end b."""
        return self._length_splice_b

    @length_splice_b.setter
    def length_splice_b(self: Self, value: float) -> None:
        self._length_splice_b = value


    @property
    def length_eye_a(self: Self) -> float:
        """Length of eye at end a when bent around a pin of dia 0."""
        return self._length_eye_a

    @length_eye_a.setter
    def length_eye_a(self: Self, value: float) -> None:
        self._length_eye_a = value


    @property
    def length_eye_b(self: Self) -> float:
        """Length of eye at end b when bent around a pin of dia 0, i.e. not circumferential length of eye."""
        return self._length_eye_b

    @length_eye_b.setter
    def length_eye_b(self: Self, value: float) -> None:
        self._length_eye_b = value


    @property
    def _l_body(self: Self) -> float:
        """Length of body, excluding eyes and splices."""
        return self.l_ultimate - self.length_eye_a - self.length_eye_b - self.length_splice_a - \
               self.length_splice_b


    @property
    def k(self: Self) -> float:
        """Sling stiffness."""
        return self.ea / (self.length_eye_a + self.length_eye_b + self.length_splice_a/2 + self.length_splice_b/2 +
                 self._l_body)

    @k.setter
    def k(self: Self, value: float) -> None:
        self.ea = value * (self.length_eye_a + self.length_eye_b + self.length_splice_a/2 + self.length_splice_b/2 +
                          self._l_body)


    @property
    def mbl(self: Self) -> float:
        """Sling mbl."""
        return self._mbl

    @mbl.setter
    def mbl(self: Self, value: float) -> None:
        self._mbl = value


    def rope_length(self: Self) -> float:
        """Length of rope to make up sling.

        Equal to ultimate length of sling, plus length where
        sling is doubled (eyes and splice).
        """
        return (self.ultimate_length + self.length_eye_a + self.length_eye_b + self.length_splice_a +
                self.length_splice_b)


    @property
    def eye_a_separation_angle(self: Self) -> pint.Quantity:
        return self._sling_eye_separation_angle(self.length_eye_a, self.end_a, self.diameter)


    @property
    def eye_b_separation_angle(self: Self) -> pint.Quantity:
        return self._sling_eye_separation_angle(self.length_eye_b, self.end_b, self.diameter)


    @property
    def eye_a_apex_offset(self: Self) -> pint.Quantity:
        """Return the distance from the pin centre to the beginning of the splice."""
        pin_diameter = self.end_a.diameter

        if self.end_a.type == "pin" and pin_diameter:
            return pin_diameter / 2 / np.cos(self.eye_a_separation_angle/2)
        return None


    @property
    def eye_b_apex_offset(self: Self) -> pint.Quantity:
        """Return the distance from the pin centre to the beginning of the splice."""
        pin_diameter = self.end_b.diameter

        if self.end_b.type == "pin" and pin_diameter:
            return pin_diameter / 2 / np.cos(self.eye_b_separation_angle/2)
        return None


    def _estimate_eye_length(self: Self) -> float:
        """Ref. NS-EN 13414-3."""
        length = None
        if self.diameter:
            inside_eye_width = 7.5 * self.diameter
            inside_eye_length = 15 * self.diameter

            # contact angle
            alpha = math.acos(self.diameter/2 / (inside_eye_length - 0.5 * inside_eye_width))
            a = ((inside_eye_length - 0.5 * inside_eye_width)**2 - (self.diameter/2)**2)**0.5
            b = (math.pi - alpha) * self.diameter
            length = 2 * a + b
        else:
            length = 1.0 * ureg.meter
        return length


    def _estimate_eye_splice(self: Self) -> float:
        """Ref. NS-EN 13414-3."""
        return 15 * self.diameter if self.diameter else 1. * ureg.meter


    def _sling_eye_separation_angle(self: Self, l_eye: float, end: AttachmentPoint, rope_diameter: float) -> float:
        """Calculate the angle of the point of contact between the sling eye and the sheave."""
        if end.type == "pin" and end.diameter and rope_diameter:
            r = end.diameter/2 + rope_diameter/2

            f = lambda x: math.tan(x) - x - l_eye / r + math.pi
            try:
                alpha = newton(func=f, x0=math.pi / 2 * 0.99)
            except Exception:
                logger.warning(f"{self.name}: could not calculate contact point on end {end}. Setting sensible value.")
                alpha = math.pi / 2 * 0.9
            return 2 * alpha * ureg.radians
        else:
            return None


    def to_dict(self: Self) -> dict:
        ret = super().to_dict()
        ret["rope_kind"] = self.kind.name
        ret["end_a"] = self.end_a.to_dict()
        ret["end_b"] = self.end_b.to_dict()
        ret["sheaves"] = [s.to_dict() for s in self.sheaves]
        ret["eye_a"] = {
            "length_splice": self.length_splice_a,
            "separation_angle": self.eye_a_separation_angle,
            "apex_offset": self.eye_a_apex_offset,
        }
        ret["eye_b"] = {
            "length_splice": self.length_splice_b,
            "separation_angle": self.eye_b_separation_angle,
            "apex_offset": self.eye_b_apex_offset,
        }

        return ret