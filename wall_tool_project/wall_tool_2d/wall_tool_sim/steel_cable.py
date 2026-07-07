"""Steel cable assembly properties shared by the 2D and 3D wall-tool models."""

from __future__ import annotations

import math
from dataclasses import dataclass


DEFAULT_STEEL_CABLE_DIAMETER_M = 0.0012
DEFAULT_STEEL_YOUNG_MODULUS_PA = 200.0e9
DEFAULT_STEEL_DENSITY_KG_M3 = 7850.0


@dataclass(frozen=True)
class SteelCableSpec:
    """Effective steel cable plus reel/termination compliance model.

    The cable itself is very stiff. The explicit simulation should not pretend
    the reel, anchor, and payload termination are infinitely rigid, so the
    structural compliance term is part of the physical assembly model.
    """

    diameter_m: float = DEFAULT_STEEL_CABLE_DIAMETER_M
    youngs_modulus_pa: float = DEFAULT_STEEL_YOUNG_MODULUS_PA
    density_kg_m3: float = DEFAULT_STEEL_DENSITY_KG_M3
    structural_compliance_m_N: float = 3.0e-4
    damping_ratio: float = 0.22
    payload_weight_fraction: float = 0.50
    min_visual_tension_N: float = 0.15
    max_visual_sag_m: float = 0.080

    def __post_init__(self) -> None:
        if self.diameter_m <= 0.0:
            raise ValueError("steel cable diameter must be positive")
        if self.youngs_modulus_pa <= 0.0:
            raise ValueError("steel cable Young's modulus must be positive")
        if self.density_kg_m3 <= 0.0:
            raise ValueError("steel cable density must be positive")
        if self.structural_compliance_m_N < 0.0:
            raise ValueError("steel cable structural compliance cannot be negative")
        if self.damping_ratio < 0.0:
            raise ValueError("steel cable damping ratio cannot be negative")
        if not 0.0 <= self.payload_weight_fraction <= 1.0:
            raise ValueError("steel cable payload weight fraction must be within [0, 1]")
        if self.min_visual_tension_N <= 0.0:
            raise ValueError("steel cable minimum visual tension must be positive")
        if self.max_visual_sag_m < 0.0:
            raise ValueError("steel cable maximum visual sag cannot be negative")

    @property
    def radius_m(self) -> float:
        return 0.5 * self.diameter_m

    @property
    def area_m2(self) -> float:
        return math.pi * self.radius_m * self.radius_m

    @property
    def mass_per_length_kg_m(self) -> float:
        return self.density_kg_m3 * self.area_m2

    def axial_stiffness_N_m(self, length_m: float) -> float:
        length = max(float(length_m), 1.0e-6)
        steel_compliance = length / (self.youngs_modulus_pa * self.area_m2)
        total_compliance = steel_compliance + self.structural_compliance_m_N
        if total_compliance <= 0.0:
            raise ValueError("steel cable assembly compliance must be positive")
        return 1.0 / total_compliance

    def damping_N_s_m(self, length_m: float, effective_mass_kg: float) -> float:
        stiffness = self.axial_stiffness_N_m(length_m)
        mass = max(float(effective_mass_kg), 1.0e-6)
        return 2.0 * self.damping_ratio * math.sqrt(stiffness * mass)

    def mass_kg(self, length_m: float) -> float:
        return self.mass_per_length_kg_m * max(float(length_m), 0.0)

    def weight_N(self, length_m: float, gravity_m_s2: float) -> float:
        return self.mass_kg(length_m) * max(float(gravity_m_s2), 0.0)
