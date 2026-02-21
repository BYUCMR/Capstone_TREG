import asyncio
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Final, Protocol

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from . import tubetruss as tt
from .arraytypes import IndexVector, Matrix, SingleIndex


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
    def add_to_view(self, view: gl.GLViewWidget, /) -> None: ...
    def update_pos(self, pos: Matrix, /) -> None: ...


@dataclass(slots=True, frozen=True)
class DrawnLinks(AnimationItem):
    nodes: IndexVector
    drawing: gl.GLLinePlotItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.drawing)

    def update_pos(self, pos: Matrix) -> None:
        self.drawing.setData(pos=pos[self.nodes])


@dataclass(slots=True, frozen=True)
class NodeTrace(AnimationItem):
    node: SingleIndex
    drawing: gl.GLScatterPlotItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.drawing)

    def update_pos(self, pos: Matrix) -> None:
        points = () if self.drawing.pos is None else self.drawing.pos
        self.drawing.setData(pos=[*points, pos[self.node]])


@dataclass(slots=True, frozen=True)
class BodyMesh(AnimationItem):
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


def draw_links(links: Iterable[tt.Link], pos: Matrix, *, color: str = 'gray', width: int = 6) -> DrawnLinks:
    nodes = list(np.array(list(links)).flat)
    drawing = gl.GLLinePlotItem(
        pos=pos[nodes],
        width=width,
        color=pg.mkColor(color),
        mode='lines',
    )
    drawing.setGLOptions('opaque')
    return DrawnLinks(nodes, drawing)


def draw_traces(nodes: Iterable[SingleIndex], pos: Matrix, *, size: int = 4) -> list[NodeTrace]:
    traces: list[NodeTrace] = []
    for node in nodes:
        drawing = gl.GLScatterPlotItem(pos=[pos[node]], size=size)
        drawing.setGLOptions('opaque')
        trace = NodeTrace(node, drawing)
        traces.append(trace)
    return traces


@dataclass(slots=True, frozen=True)
class Animator:
    view: gl.GLViewWidget
    items: Iterable[AnimationItem] = ()

    def update_pos(self, pos: Matrix) -> None:
        for item in self.items:
            item.update_pos(pos)

    async def animate(self, positions: asyncio.Queue[Matrix]) -> None:
        self.view.show()
        while True:
            try:
                pos = await positions.get()
            except asyncio.QueueShutDown:
                break
            self.update_pos(pos)
            positions.task_done()
            await asyncio.sleep(0.01)
