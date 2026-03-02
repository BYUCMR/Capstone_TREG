import math
from collections.abc import Iterable
from dataclasses import dataclass
from functools import cached_property
from itertools import pairwise
from typing import Self

import numpy as np

from rift.arraytypes import IndexVector, Matrix, MatrixStack, SingleIndex, Vector


@dataclass(frozen=True)
class Truss:
    """A representation of a truss structure."""
    incidence: Matrix[np.int8]

    @classmethod
    def from_trails(cls, *trails: Iterable[SingleIndex]) -> Self:
        n_nodes = 1 + max(node for trail in trails for node in trail)
        rows: list[Vector[np.int8]] = []
        for nodes in trails:
            for i, j in pairwise(nodes):
                row = np.zeros(n_nodes, dtype=np.int8)
                row[i] =  1
                row[j] = -1
                rows.append(row)
        return cls(np.array(rows, dtype=np.int8))

    @cached_property
    def nodes(self) -> Vector[np.intp]:
        return np.unique(np.nonzero(self.incidence)[1])

    @cached_property
    def links(self) -> Matrix[np.intp]:
        return np.array([np.flatnonzero(row) for row in self.incidence])

    @cached_property
    def pos_to_rigidity(self) -> MatrixStack:
        """Return a 3-D matrix that can be used to calculate a rigidity matrix.

        This matrix converts a vector of node positions into a rigidity matrix.
        The rigidity matrix converts a column of node velocities into a column
        comprising the length of each link times its rate of change.
        """
        M = np.array([np.linalg.outer(row, row) for row in self.incidence])
        # We could skip the Kronecker product here and replace
        # `M @ pos.ravel()` with `(M @ pos).reshape(-1, pos.size)`
        # later on, but this keeps the interface simpler.
        return np.kron(M, np.eye(3))

    def rigidity_at(self, pos: Matrix, *, normalize: bool = True) -> Matrix:
        """
        Return the rigidity matrix of the truss in a given state.

        This matrix converts a flattened vector of node velocities into a
        vector comprising the rate of change of each link.

        If `normalize == False`, then each value in the resulting vector will
        be multiplied by the length of the corresponding link.
        """
        rigidity = self.pos_to_rigidity @ pos.ravel()
        if normalize:
            norms = np.linalg.vector_norm(rigidity, axis=1, keepdims=True)
            rigidity *= math.sqrt(2.) / norms
        return rigidity

    def attach(self, other: Self, nodemap: IndexVector | None = None) -> Self:
        """
        Combine this truss with another.

        If specified, `nodemap` should be a vector of the indices that the
        nodes of the other truss should map to in the new truss. The nodes
        of this truss keep their indices in the new one.
        """
        n_links_self, n_nodes_self = self.incidence.shape
        n_links_other, n_nodes_other = other.incidence.shape
        n_links_total = n_links_self + n_links_other
        if nodemap is None:
            nodemap = np.arange(n_nodes_other)
        last_node = max(*range(n_nodes_self), *nodemap)
        incidence = np.zeros((n_links_total, last_node+1), dtype=np.int8)
        incidence[:n_links_self, :n_nodes_self] = self.incidence
        incidence[n_links_self:, nodemap] = other.incidence
        return type(self)(incidence)
