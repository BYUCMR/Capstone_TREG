import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import numpy as np
import plotly.graph_objects as go

import rift.anim
from rift.robot import InverseKinematicsError, RobotInverse
from rift.state import RobotState
from rift.truss_config import TrussConfig


def initialize_fig(config: TrussConfig, state: RobotState) -> go.Figure:
    return go.Figure(
        data=rift.anim.generate_data(config, state, np.zeros((0, 3))),
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


def main(
    config: TrussConfig,
    *,
    step_length: float = 0.8,
    cycles: int = 1,
    resolution: int = 50,
):
    robot = RobotInverse(config)
    fig = initialize_fig(config, robot.state)
    frames: list[go.Frame] = []
    try:
        for _ in range(cycles):
            for path in robot.crawl(step_length, resolution=resolution):
                data = rift.anim.generate_data(config, robot.state, path)
                frames.append(go.Frame(data=data))
    except InverseKinematicsError as e:
        print(e.args[0])
    fig.frames = frames
    fig.show()


if __name__ == '__main__':
    from rift.truss_config import CONFIG_ROVER as config
    main(config, cycles=4, resolution=100)
