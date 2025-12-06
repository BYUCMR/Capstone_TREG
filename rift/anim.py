from collections.abc import Iterable
from dataclasses import dataclass
from typing import Protocol, Self

import plotly.graph_objects as go

from . import tubetruss as tt
from .truss_config import TrussConfig
from .typing import Matrix


class AnimationItem(Protocol):
    def add_to_frame(self, data: list, /) -> None: ...
    def update_pos(self, pos: Matrix, /) -> None: ...


@dataclass(slots=True, frozen=True)
class DrawnTube(AnimationItem):
    tube: tt.Tube
    drawing: go.Scatter3d

    def add_to_frame(self, data: list) -> None:
        data.append(self.drawing)

    def update_pos(self, pos: Matrix) -> None:
        x, y, z = pos[list(self.tube.nodes)].T
        self.drawing.update(x=x, y=y, z=z)


@dataclass(slots=True, frozen=True)
class PayloadMesh(AnimationItem):
    payload: tt.TubeTruss
    mesh: go.Mesh3d

    def add_to_frame(self, data: list) -> None:
        data.append(self.mesh)

    def update_pos(self, pos: Matrix) -> None:
        nodes = set[tt.Node]()
        nodes.update(*(bar.nodes for bar in self.payload))
        x, y, z = pos[sorted(nodes)].T
        self.mesh.update(x=x, y=y, z=z)


def draw_tube(tube: tt.Tube, pos: Matrix, *, color: str = 'gray', width: int = 6) -> DrawnTube:
    x, y, z = pos[list(tube.nodes)].T
    drawing = go.Scatter3d(
        x=x, y=y, z=z,
        mode='lines',
        line=dict(width=width, color=color),
        showlegend=False,
    )
    return DrawnTube(tube, drawing)


def draw_payload_bars(payload: tt.TubeTruss, pos: Matrix) -> list[DrawnTube]:
    return [draw_tube(bar, pos, color='black', width=4) for bar in payload]


def draw_payload_mesh(payload: tt.TubeTruss, pos: Matrix) -> PayloadMesh:
    payload_nodes = set[tt.Node]()
    payload_nodes.update(*(bar.nodes for bar in payload))

    x, y, z = pos[sorted(payload_nodes)].T
    payload_faces = [[0, 1, 2], [3, 4, 5],
                     [0, 3, 5], [0, 2, 5],
                     [1, 4, 5], [1, 2, 5],
                     [0, 1, 4], [0, 3, 4]]

    mesh = go.Mesh3d(
        x=x, y=y, z=z,
        i=[f[0] for f in payload_faces],
        j=[f[1] for f in payload_faces],
        k=[f[2] for f in payload_faces],
        color='lightblue',
        opacity=1,
        flatshading=True,
        name='Prism'
    )
    return PayloadMesh(payload, mesh)


def draw_triangles(triangles: tt.TubeTruss, pos: Matrix) -> list[DrawnTube]:
    triangle_colors = {0: 'blue', 1: 'red', 2: 'orange', 3: 'green', 4: 'brown', 5: 'yellow', 6: 'purple'}
    drawn_tubes: list[DrawnTube] = []
    for i, tube in enumerate(triangles):
        color = triangle_colors.get(i, 'gray')
        drawn_tube = draw_tube(tube, pos, color=color)
        drawn_tubes.append(drawn_tube)
    return drawn_tubes


def draw_items(config: TrussConfig, pos: Matrix) -> list[AnimationItem]:
    payload_bars = draw_payload_bars(config.payload, pos)
    payload_mesh = draw_payload_mesh(config.payload, pos)
    triangles = draw_triangles(config.triangles, pos)
    return [payload_mesh, *payload_bars, *triangles]


def initialize_fig(initial_data) -> go.Figure:
    return go.Figure(
        data=initial_data,
        layout=go.Layout(
            updatemenus=[dict(
                type='buttons',
                buttons=[dict(
                    label='Play',
                    method='animate',
                    args=[None, dict(
                        frame=dict(duration=10, redraw=True),
                        transition=dict(duration=0),
                    )]
                )]
            )],
            scene=dict(
                aspectmode='cube',
                bgcolor='white',
                xaxis=dict(
                    range=(-8, 12),
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                ),
                yaxis=dict(
                    range=(-10, 10),
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                ),
                zaxis=dict(
                    range=(-1, 19),
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                )
            )
        )
    )


@dataclass(slots=True, frozen=True)
class Animator:
    items: Iterable[AnimationItem] = ()

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        items = draw_items(config, config.initial_pos)
        return cls(items)

    def update_pos(self, pos: Matrix) -> None:
        for item in self.items:
            item.update_pos(pos)

    def make_frame(self) -> go.Frame:
        data = []
        for item in self.items:
            item.add_to_frame(data)
        return go.Frame(data=data)

    def animate(self, positions: Iterable[Matrix] = ()) -> None:
        frames: list[go.Frame] = []
        for pos in positions:
            self.update_pos(pos)
            frame = self.make_frame()
            frames.append(frame)
        fig = initialize_fig(frames[0].data)
        fig.frames = frames
        fig.show()
