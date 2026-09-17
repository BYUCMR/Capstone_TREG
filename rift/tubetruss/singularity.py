import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from rift.arraytypes import Vector
from rift.motion import steps
from rift.protocols import HasIncidence, HasPos, StateFunction
from .linalg import get_rigidity_gradient
from .robotics import HasActuation


class Stabilizable(HasPos, HasIncidence, HasActuation, Protocol): ...


def default_dV_de(e: float) -> float:
    return -1e-6 * e**-3


@dataclass(slots=True, frozen=True, kw_only=True)
class SingularityCost(StateFunction[Stabilizable, Vector]):
    """A cost that incentivises singularity avoidance."""
    dV_de: Callable[[float], float] = default_dV_de

    def at(self, state: Stabilizable) -> Vector:
        R, dR_dx = get_rigidity_gradient(state.incidence, state.pos)
        U = state.actuation.unreachable
        A = U @ R
        dA_dx = U @ dR_dx
        e, v = steps.singularity_eig(A)
        de_dx = 2. * np.vecdot(A @ v, dA_dx @ v)
        dV_de = self.dV_de(e)
        return dV_de * de_dx
