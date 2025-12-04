from dataclasses import dataclass, field

import plotly.graph_objects as go

from .truss_config import TrussConfig
from .tubetruss import Tube
from .typing import Matrix


def draw_tube(tube: Tube, pos: Matrix, *, color: str = 'gray', width: int = 6) -> go.Scatter3d:
    x, y, z = ([pos[v][i] for v in tube.nodes] for i in range(3))
    return go.Scatter3d(
        x=x, y=y, z=z,
        mode='lines',
        line=dict(width=width, color=color),
        showlegend=False,
    )


def plot_payload_edges(config: TrussConfig, pos: Matrix) -> list[go.Scatter3d]:
    return [draw_tube(e, pos, color='black', width=4) for e in config.payload]


def plot_payload(config: TrussConfig, pos: Matrix) -> go.Mesh3d:
    payload_ind = set[int]()
    for i in config.payload:
        payload_ind.update(i.nodes)

    payload_pnts = pos[list(payload_ind)]
    x = payload_pnts[:, 0]  # First column
    y = payload_pnts[:, 1]  # Second column
    z = payload_pnts[:, 2]
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


def plot_triangles(config: TrussConfig, pos: Matrix) -> list[go.Scatter3d]:
    triangle_colors = {0: 'blue', 1: 'red', 2: 'orange', 3: 'green', 4: 'brown', 5: 'yellow', 6: 'purple'}
    traces: list[go.Scatter3d] = []
    for t, tri in enumerate(config.triangles):
        color = triangle_colors.get(t, 'gray')
        trace = draw_tube(tri, pos, color=color)
        traces.append(trace)
    return traces


def generate_data(config: TrussConfig, pos: Matrix) -> list[go.Mesh3d | go.Scatter3d]:
    payload_scatter = plot_payload_edges(config, pos)
    payload_mesh = plot_payload(config, pos)
    triangle_scatter = plot_triangles(config, pos)
    return [payload_mesh, *payload_scatter, *triangle_scatter]


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
