import asyncio
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Final, Protocol, Self

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from . import tubetruss as tt
from .truss_config import TrussConfig
from .typing import Matrix


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


def draw_traces(nodes: Iterable[tt.Node], pos: Matrix, *, size: int = 4) -> list[NodeTrace]:
    traces: list[NodeTrace] = []
    for node in nodes:
        drawing = gl.GLScatterPlotItem(pos=[pos[node]], size=size)
        drawing.setGLOptions('opaque')
        trace = NodeTrace(node, drawing)
        traces.append(trace)
    return traces


def draw_payload_bars(payload: tt.TubeTruss, pos: Matrix) -> list[DrawnTube]:
    return [draw_tube(bar, pos, color='black', width=4) for bar in payload]


def draw_payload_mesh(payload: tt.TubeTruss, pos: Matrix) -> PayloadMesh:
    payload_nodes = set[tt.Node]()
    payload_nodes.update(*(bar.nodes for bar in payload))

    payload_vertices = pos[sorted(payload_nodes)]
    payload_faces = [[0, 1, 2], [3, 4, 5],
                     [0, 3, 5], [0, 2, 5],
                     [1, 4, 5], [1, 2, 5],
                     [0, 1, 4], [0, 3, 4]]

    meshdata = gl.MeshData(
        vertexes=payload_vertices,
        faces=payload_faces,
    )
    mesh = gl.GLMeshItem(
        meshdata=meshdata,
        color=pg.mkColor(OKABE_ITO[-1]),
    )
    mesh.setGLOptions('opaque')
    return PayloadMesh(payload, mesh)


def draw_triangles(triangles: tt.TubeTruss, pos: Matrix) -> list[DrawnTube]:
    drawn_tubes: list[DrawnTube] = []
    for i, tube in enumerate(triangles):
        color = OKABE_ITO[i % (len(OKABE_ITO) - 1) + 1]
        drawn_tube = draw_tube(tube, pos, color=color)
        drawn_tubes.append(drawn_tube)
    return drawn_tubes


def draw_items(config: TrussConfig, pos: Matrix) -> list[AnimationItem]:
    payload_bars = draw_payload_bars(config.payload, pos)
    payload_mesh = draw_payload_mesh(config.payload, pos)
    triangles = draw_triangles(config.triangles, pos)
    traces = draw_traces((0, 1, 6, 7), pos)
    return [payload_mesh, *payload_bars, *triangles, *traces]


@dataclass(slots=True, frozen=True)
class Animator:
    view: gl.GLViewWidget
    items: Iterable[AnimationItem] = ()

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        pg.mkQApp()
        view = gl.GLViewWidget()
        view.addItem(gl.GLGridItem())
        items = draw_items(config, config.initial_pos)
        for item in items:
            item.add_to_view(view)
        return cls(view, items)

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
