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

    def __init__(self: Self, id: str) -> None:
        """Initialise rope object."""
        self.id = id
        self.d = 0 * ureg.millimeter
        self.ea = 0 * ureg.newton
        self.rope_kind = None
        self.g = 9.81 * ureg("m/s/s")
        self.default_rope_tensile_strength = 2160 * ureg("N/mm/mm")


    def _estimate_mbl(self: Self, kind: RopeKinds, diameter: float) -> float:
        """Estimate the mbl (mass units) based on the diameter. Ref. ISO 19901-6:2009, Sec. 18.4.2.

        It is assumed that HMPE has the same strength as steel wire rope with the same diameter.
        """
        mbl = None
        if kind in [RopeKinds.IWRC, RopeKinds.HMPE] and diameter:
            if diameter <= 60 * ureg("mm"):
                mbl = (self.default_rope_tensile_strength * 0.346 * diameter**2) / self.g
            else:
                mbl = (8.55 * ureg("kN/mm") * diameter + 0.592 * ureg("kN/mm/mm") * diameter**2 - \
                      0.000615 * ureg("kN/mm/mm/mm") * diameter**3) / self.g
        elif kind == RopeKinds.CABLE and diameter:
            # For simplicity, assume cable made up of core + 6 ropes, all with equal diameter
            mbl = 0.85 * (6+1) * self._estimate_mbl(kind, diameter/3)
        return mbl.to_compact()


    def _estimate_diameter(self: Self, kind: RopeKinds, mbl: float) -> float:
        """Estimate diameter from the mbl (mass-units). See _estimate_mbl for reference material."""
        diameter = None
        if kind in (RopeKinds.IWRC, RopeKinds.HMPE) and mbl:
            diameter = (mbl * self.g / self.default_rope_tensile_strength / 0.346)**0.5

            if diameter > 60 * ureg("mm") and mbl <= 8846*ureg("t"):
                # Note that the polynomial from ISO 19901-6 peaks at approx d=649mm, i.e. if MBL exceeds
                # 86783kN = approx. 8846t, return 649mm. Otherwise, find roots; between 61mm and 649mm,
                # the polynomial has 3 real roots - return the smallest positive root
                # Ref. https://en.wikipedia.org/wiki/Cubic_equation, sections on depressed cubic and 3 real roots
                # ax^3+bx^2+cx+d=0
                a = -0.000615 * ureg("kN/mm/mm/mm")
                b = 0.592 * ureg("kN/mm/mm")
                c = 8.55 * ureg("kN/mm")
                d = -mbl * self.g

                # shift to depressed cubic equation, t^3+pt+q=0, where x = t - b/(3a)
                p = (3*a*c-b**2)/(3*a**2)
                q = (2*b**3 - 9*a*b*c + 27*a**2*d) / (27*a**3)

                t_0 = 2*(-p/3)**0.5 * math.cos(1/3*math.acos(3*q/2/p*(-3/p)**0.5) - 0*2*math.pi/3)
                t_1 = 2*(-p/3)**0.5 * math.cos(1/3*math.acos(3*q/2/p*(-3/p)**0.5) - 1*2*math.pi/3)
                t_2 = 2*(-p/3)**0.5 * math.cos(1/3*math.acos(3*q/2/p*(-3/p)**0.5) - 2*2*math.pi/3)

                # shift roots back
                x_0 = t_0 - b/(3*a)
                x_1 = t_1 - b/(3*a)
                x_2 = t_2 - b/(3*a)

                diameter = min([d for d in [x_0, x_1, x_2] if d>0])
            elif mbl > 8846*ureg("t"):
                diameter = 649 * ureg("mm")

        elif kind == RopeKinds.CABLE and mbl:
            # Assume make-up is core + 6 identical slings
            mbl_single_rope = mbl / 0.85 / (6+1)
            diameter = 3 * self._estimate_diameter(RopeKinds.IWRC, mbl_single_rope)

        return diameter


    def _estimate_area(self: Self, kind: RopeKinds, diameter: float) -> float:
        area = None
        if diameter and kind in (RopeKinds.IWRC, RopeKinds.CABLE):
            # Ref. https://www.vornbaeumen.de/knowhow/calculation-variables/?lang=en, 6x19 IWRC
            area = 0.449 * diameter**2
        elif diameter and kind in [RopeKinds.HMPE]:
            # Assume 12x12 makeup. Ref. https://bexco-cms.lwprod.nl/uploads/1548936358_HL_SUPERIOR_SK78.pdf
            d_strand = diameter / 4
            d_substrand = d_strand / 4
            area = (math.pi / 4 * d_substrand**2 * 12) * 12
        return area


    def _estimate_ea(self: Self, kind: RopeKinds, area: float, mbl: float) -> float:
        ea = None
        if kind == RopeKinds.IWRC and area:
            # Ref. https://www.orcina.com/webhelp/OrcaFlex/Content/html/Ropewire,Axialandbendingstiffness.htm
            ea = 1.13E8 * ureg("kN/m/m") * area
        elif kind == RopeKinds.CABLE and area:
            ea = 1.13E8 * ureg("kN/m/m") * area
        elif kind == RopeKinds.HMPE and mbl:
            # Ref. Amsteel Blue tech info
            # Elongation of 0.7% at 20%MBL, 0.96% at 30%MBL
            # k = EA / L -> EA = kL = F/dx * L = F / (dx/L)
            # EA_20%MBL = 0.2MBL/0.007 = 28.6MBL
            # EA_30%MBL = 0.3MBL/0.0096 = 31.25MBL
            # EA = 30MBL seems to give a reasonable value for typical load range
            ea = 30 * mbl * self.g
        return ea


    def _estimate_mass_per_length(self: Self, kind: RopeKinds, area: float) -> float:
        mass_per_length = None
        if kind in (RopeKinds.IWRC, RopeKinds.CABLE) and area:
            mass_per_length = 7.850 * ureg("t/m/m/m") * area
        elif kind == RopeKinds.HMPE and area:
            # Build a rope from 12x12 substrands and strands, and tune density until linear weight
            # approximately matches the table in https://bexco-cms.lwprod.nl/uploads/1548936358_HL_SUPERIOR_SK78.pdf
            mass_per_length = 1.165 * ureg("t/m/m/m") * area
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
        """Create a dict for export."""
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
            self.mass_per_length = self._estimate_mass_per_length(self.kind, self.area)


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
        return (self.l_ultimate + self.length_eye_a + self.length_eye_b + self.length_splice_a +
                self.length_splice_b)


    @property
    def eye_a_separation_angle(self: Self) -> pint.Quantity:
        """Return the angle between the two tangents of the legs of the eye."""
        return self._sling_eye_separation_angle(self.length_eye_a, self.end_a, self.diameter)


    @property
    def eye_b_separation_angle(self: Self) -> pint.Quantity:
        """Return the angle between the two tangents of the legs of the eye."""
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

            f = lambda x: math.tan(x) - x - l_eye / r + math.pi     # noqa: E731
            try:
                alpha = newton(func=f, x0=math.pi / 2 * 0.99)
            except Exception:
                logger.warning(f"{self.name}: could not calculate contact point on end {end}. Setting sensible value.")
                alpha = math.pi / 2 * 0.9
            return 2 * alpha * ureg.radians
        else:
            return None


    def to_dict(self: Self) -> dict:
        """Create a dict representation of the sling for export."""
        ret = super().to_dict()

        return ret | {
            "rope_kind": self.kind.name,
            "end_a": self.end_a.to_dict(),
            "end_b": self.end_b.to_dict(),
            "sheaves": [s.to_dict() for s in self.sheaves],
            "ea": self.ea.to("kN"),
            "k": self.k.to("kN/m"),
            "ultimate_length": self.l_ultimate,
            "mass": self.mass.to("t"),
            "mbl": self.mbl.to("t"),
            "eye_a": {
                "length_splice": self.length_splice_a,
                "separation_angle": self.eye_a_separation_angle,
                "apex_offset": self.eye_a_apex_offset,
            },
            "eye_b": {
                "length_splice": self.length_splice_b,
                "separation_angle": self.eye_b_separation_angle,
                "apex_offset": self.eye_b_apex_offset,
            },
        }
