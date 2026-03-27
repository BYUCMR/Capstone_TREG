from collections.abc import Iterable
from dataclasses import dataclass
from typing import Self, SupportsIndex

import numpy as np

from rift.arraytypes import Matrix
from .linalg import cokernel, incidence_from_trails

type RealMatrix = Matrix[np.integer | np.floating]


@dataclass(slots=True, frozen=True)
class LengthControl:
    forward: RealMatrix
    inverse: RealMatrix
    unreachable: RealMatrix

    def __post_init__(self) -> None:
        if self.forward.shape != self.inverse.T.shape:
            raise ValueError("Forward and inverse matrices have mismatched shapes")
        if self.inverse.shape[1] != self.unreachable.shape[1]:
            raise ValueError("Inverse and unreachable matrices have unequal column counts")
        if np.any(self.unreachable @ self.forward):
            raise ValueError("Unreachable null space does not contain forward column space")
        round_trip = self.inverse @ self.forward
        identity = np.eye(self.forward.shape[1])
        if not np.allclose(round_trip, identity):
            raise ValueError("Inverse matrix times forward matrix is not an identity")

    @property
    def n_inputs(self) -> int:
        return len(self.inverse)

    @property
    def n_outputs(self) -> int:
        return len(self.forward)

    @classmethod
    def make_simple(cls, n: int) -> Self:
        return cls(np.eye(n), np.eye(n), np.zeros((0, n)))

    @classmethod
    def from_unreachable(cls, unreachable: RealMatrix) -> Self:
        forward = cokernel(unreachable.T).T
        inverse = np.linalg.pinv(forward)
        return cls(forward, inverse, unreachable)

    @classmethod
    def from_forward(cls, forward: RealMatrix) -> Self:
        inverse = np.linalg.pinv(forward)
        unreachable = cokernel(forward)
        return cls(forward, inverse, unreachable)

    @classmethod
    def from_trails(
        cls,
        *trails: Iterable[SupportsIndex],
        n_static: int = 0,
    ) -> Self:
        forward_T = incidence_from_trails(*trails, empty_cols=n_static)
        return cls.from_forward(forward_T.T)
