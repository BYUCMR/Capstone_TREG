from dataclasses import dataclass
from typing import Self

import numpy as np

from rift.arraytypes import Matrix
from .linalg import cokernel


@dataclass(frozen=True)
class LengthControl:
    unreachable: Matrix[np.integer]
    forward: Matrix[np.integer]
    inverse: Matrix

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
    def from_unreachable(cls, unreachable: Matrix[np.integer]) -> Self:
        forward = cokernel(unreachable.T).T
        inverse = np.linalg.pinv(forward)
        return cls(unreachable, forward, inverse)

    @classmethod
    def from_forward(cls, forward: Matrix[np.integer]) -> Self:
        inverse = np.linalg.pinv(forward)
        unreachable = cokernel(forward)
        return cls(unreachable, forward, inverse)
