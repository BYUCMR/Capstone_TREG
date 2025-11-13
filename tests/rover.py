import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import numpy as np
import plotly.graph_objects as go

import rift.anim
from rift.linalg import Matrix, roll_pitch_yaw
from rift.path import make_path
from rift.robot import RobotInverse
from rift.state import RobotState
from rift.truss_config import CONFIG_3D_ROVER1 as config, TrussConfig


def initialize_fig(config: TrussConfig, state: RobotState, path: Matrix) -> go.Figure:
    return go.Figure(
        data=rift.anim.generate_data(config, state, path),
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
                    range=(-20, 30),
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                ),
                yaxis=dict(
                    range=(-25, 25),
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                ),
                zaxis=dict(
                    range=(-2, 48),
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                )
            )
        )
    )


def main():
    path = make_path(xform=roll_pitch_yaw(np.pi/4, np.pi/6, np.pi/4))
    path += config.initial_pos[config.move_node]
    robot = RobotInverse(config)

    fig = initialize_fig(config, robot.state, path)
    frames = [go.Frame(data=rift.anim.generate_data(config, robot.state, path))]
    for path in robot.crawl():
        frames.append(go.Frame(data=rift.anim.generate_data(config, robot.state, path)))
    fig.frames = frames
    fig.show()


if __name__ == '__main__':
    main()
