from dataclasses import dataclass, field

import plotly.graph_objects as go

from . import tubetruss as tt
from .truss_config import TrussConfig
from .typing import Matrix


def draw_tube(tube: tt.Tube, pos: Matrix, *, color: str = 'gray', width: int = 6) -> go.Scatter3d:
    x, y, z = pos[list(tube.nodes)].T
    return go.Scatter3d(
        x=x, y=y, z=z,
        mode='lines',
        line=dict(width=width, color=color),
        showlegend=False,
    )


def draw_payload_bars(payload: tt.TubeTruss, pos: Matrix) -> list[go.Scatter3d]:
    return [draw_tube(bar, pos, color='black', width=4) for bar in payload]


def draw_payload_mesh(payload: tt.TubeTruss, pos: Matrix) -> go.Mesh3d:
    payload_nodes = set[tt.Node]()
    payload_nodes.update(*(bar.nodes for bar in payload))

    x, y, z = pos[sorted(payload_nodes)].T
    payload_faces = [[0, 1, 2], [3, 4, 5],
                     [0, 3, 5], [0, 2, 5],
                     [1, 4, 5], [1, 2, 5],
                     [0, 1, 4], [0, 3, 4]]

    return go.Mesh3d(
        x=x, y=y, z=z,
        i=[f[0] for f in payload_faces],
        j=[f[1] for f in payload_faces],
        k=[f[2] for f in payload_faces],
        color='lightblue',
        opacity=1,
        flatshading=True,
        name='Prism'
    )


def draw_triangles(triangles: tt.TubeTruss, pos: Matrix) -> list[go.Scatter3d]:
    triangle_colors = {0: 'blue', 1: 'red', 2: 'orange', 3: 'green', 4: 'brown', 5: 'yellow', 6: 'purple'}
    drawn_tubes: list[go.Scatter3d] = []
    for i, tube in enumerate(triangles):
        color = triangle_colors.get(i, 'gray')
        drawn_tube = draw_tube(tube, pos, color=color)
        drawn_tubes.append(drawn_tube)
    return drawn_tubes


def generate_data(config: TrussConfig, pos: Matrix) -> list[go.Mesh3d | go.Scatter3d]:
    payload_bars = draw_payload_bars(config.payload, pos)
    payload_mesh = draw_payload_mesh(config.payload, pos)
    triangles = draw_triangles(config.triangles, pos)
    return [payload_mesh, *payload_bars, *triangles]


def initialize_fig(config: TrussConfig, pos: Matrix) -> go.Figure:
    return go.Figure(
        data=generate_data(config, pos),
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
    config: TrussConfig
    frames: list[go.Frame] = field(default_factory=list)

    def add_frame(self, pos: Matrix) -> None:
        data = generate_data(self.config, pos)
        self.frames.append(go.Frame(data=data))

    def make_figure(self) -> go.Figure:
        fig = initialize_fig(self.config, self.config.initial_pos)
        fig.frames = self.frames
        return fig
