import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import numpy as np
import plotly.graph_objects as go

import rift.anim
from rift.robot import RobotInverse
from rift.state import RobotState
from rift.truss_config import CONFIG_ROVER as config, TrussConfig


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


def main():
    robot = RobotInverse(config)
    fig = initialize_fig(config, robot.state)
    fig.frames = [
        go.Frame(data=rift.anim.generate_data(config, robot.state, path))
        for path in robot.crawl()
    ]
    fig.show()


if __name__ == '__main__':
    main()
