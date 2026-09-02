from collections.abc import Iterable
from dataclasses import dataclass
from itertools import pairwise
from typing import Final, Protocol, SupportsIndex

import numpy as np
import pyqtgraph.opengl as gl

from .arraytypes import IndexVector, Matrix, Vector


OKABE_ITO: Final = (
    '#000000',
    '#E69F00',
    '#56B4E9',
    '#009E73',
    '#F0E442',
    '#0072B2',
    '#D55E00',
    '#CC79A7',
)


class AnimationItem(Protocol):
    """A general interface for displaying things."""
    def add_to_view(self, view: gl.GLViewWidget, /) -> None: ...
    def update_pos(self, pos: Matrix, /) -> None: ...


@dataclass(slots=True, frozen=True)
class AnimationBundle(AnimationItem):
    """Multiple `AnimationItem`s wrapped up into one."""
    items: Iterable[AnimationItem]

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        for item in self.items:
            item.add_to_view(view)

    def update_pos(self, pos: Matrix) -> None:
        for item in self.items:
            item.update_pos(pos)


@dataclass(slots=True, frozen=True)
class DrawnLinks(AnimationItem):
    """A set of linked nodes drawn in 3-D space."""
    nodes: IndexVector
    drawing: gl.GLLinePlotItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.drawing)

    def update_pos(self, pos: Matrix) -> None:
        self.drawing.setData(pos=pos[self.nodes])


@dataclass(slots=True, frozen=True)
class NodeTrace(AnimationItem):
    """A trail visualizing a node's recent motion."""
    node: SupportsIndex
    length: SupportsIndex
    drawing: gl.GLScatterPlotItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.drawing)

    def update_pos(self, pos: Matrix) -> None:
        points = () if self.drawing.pos is None else self.drawing.pos
        self.drawing.setData(pos=[*points[-int(self.length):], pos[self.node]])


@dataclass(slots=True, frozen=True)
class Markers:
    """Markers on a link to visualize tube motion."""
    trail: IndexVector
    ts: Iterable[float]
    mark: gl.GLScatterPlotItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.mark)

    def update_pos(self, pos: Matrix) -> None:
        t_iter = iter(self.ts)
        try:
            t = next(t_iter)
        except StopIteration:
            return
        points: list[Vector] = []
        d_accum = 0.
        for i, j in pairwise(self.trail):
            v = pos[j] - pos[i]
            d = np.linalg.norm(v)
            while t < d:
                points.append(pos[i] + v * t / d)
                try:
                    t = next(t_iter) - d_accum
                except StopIteration:
                    t = np.inf
            d_accum += d
            t -= d
        self.mark.setData(pos=points)


@dataclass(slots=True, frozen=True)
class BodyMesh(AnimationItem):
    """A mesh in 3-D space."""
    nodes: IndexVector
    mesh: gl.GLMeshItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.mesh)

    def update_pos(self, pos: Matrix) -> None:
        # `GLMeshItem.opts` might be an implementation detail, but this is the
        # cleanest way to update vertex positions while keeping everything
        # else the same.
        mesh_data: gl.MeshData = self.mesh.opts['meshdata']
        mesh_data.setVertexes(pos[self.nodes])
        self.mesh.setMeshData(meshdata=mesh_data)


def make_default_view(
    *,
    show: bool = True,
    add_grid: bool = True,
) -> gl.GLViewWidget:
    """Create and return a `GLViewWidget`."""
    view = gl.GLViewWidget()
    if add_grid:
        view.addItem(gl.GLGridItem())
    if show:
        view.show()
    return view


def draw_links(nodes: IndexVector, pos: Matrix, **kwargs) -> DrawnLinks:
    """Construct and return an instance of `DrawnLinks`."""
    drawing = gl.GLLinePlotItem(pos=pos[nodes], **kwargs)
    drawing.setGLOptions('opaque')
    return DrawnLinks(nodes, drawing)


def draw_traces(
    nodes: Iterable[SupportsIndex],
    length: SupportsIndex,
    pos: Matrix,
    **kwargs,
) -> list[NodeTrace]:
    """Create and return a list of `NodeTrace`s."""
    traces: list[NodeTrace] = []
    for node in nodes:
        drawing = gl.GLScatterPlotItem(pos=[pos[node]], **kwargs)
        drawing.setGLOptions('opaque')
        trace = NodeTrace(node, length, drawing)
        traces.append(trace)
    return traces
