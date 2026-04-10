from typing import Protocol

import numpy as np

from rift.arraytypes import Vector


class Point(Protocol):
    """
    A general representation of a point on a truss.

    `__array__` should return a vector of weights describing how much each
    node of the truss contributes to this point's position.
    """
    def __array__(self, /) -> Vector: ...


def centroid(*points: Point) -> Vector:
    """Return the mean of the input points."""
    return np.average(np.array(points), axis=0)
