import asyncio
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Final, Protocol

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from . import tubetruss as tt
from .arraytypes import Matrix


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
class DrawnTube(AnimationItem):
    tube: tt.Tube
    drawing: gl.GLLinePlotItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.drawing)

    def update_pos(self, pos: Matrix) -> None:
        self.drawing.setData(pos=pos[list(self.tube.nodes)])


@dataclass(slots=True, frozen=True)
class NodeTrace(AnimationItem):
    node: tt.Node
    drawing: gl.GLScatterPlotItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.drawing)

    def update_pos(self, pos: Matrix) -> None:
        points = () if self.drawing.pos is None else self.drawing.pos
        self.drawing.setData(pos=[*points, pos[self.node]])


@dataclass(slots=True, frozen=True)
class PayloadMesh(AnimationItem):
    payload: tt.TubeTruss
    mesh: gl.GLMeshItem

    def add_to_view(self, view: gl.GLViewWidget) -> None:
        view.addItem(self.mesh)

    def update_pos(self, pos: Matrix) -> None:
        nodes = set[tt.Node]()
        nodes.update(*(bar.nodes for bar in self.payload))
        # `GLMeshItem.opts` might be an implementation detail, but this is the
        # cleanest way to update vertex positions while keeping everything
        # else the same.
        mesh_data: gl.MeshData = self.mesh.opts['meshdata']
        mesh_data.setVertexes(pos[list(nodes)])
        self.mesh.setMeshData(meshdata=mesh_data)


def draw_tube(tube: tt.Tube, pos: Matrix, *, color: str = 'gray', width: int = 6) -> DrawnTube:
    tube_vertices = pos[list(tube.nodes)]
    drawing = gl.GLLinePlotItem(
        pos=tube_vertices,
        width=width,
        color=pg.mkColor(color),
    )
    drawing.setGLOptions('opaque')
    return DrawnTube(tube, drawing)


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
