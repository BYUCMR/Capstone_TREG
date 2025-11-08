import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import numpy as np
import plotly.graph_objects as go

from rift.anim import RoverPlotter3D
from rift.linalg import Matrix, roll_pitch_yaw
from rift.path import make_path
from rift.robot import RobotInverse
from rift.state import RobotState
from rift.truss_config import CONFIG_3D_ROVER1 as config

path = make_path(xform=roll_pitch_yaw(np.pi/4, np.pi/6, np.pi/4))
path += config.initial_pos[config.move_node]
robot = RobotInverse(config)
plot = RoverPlotter3D(config)


def initialize_fig(plotter: RoverPlotter3D, state: RobotState, path: Matrix) -> go.Figure:
    return go.Figure(
        data=plotter.generate_data(state, path),
        layout=go.Layout(
            updatemenus=[dict(
                type='buttons',
                buttons=[dict(
                    label='Play',
                    method='animate',
                    args=[None, dict(
                        frame=dict(duration=50, redraw=True),
                        transition=dict(duration=0),
                    )]
                )]
            )],
            scene=dict(
                bgcolor='white',
                xaxis=dict(
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                ),
                yaxis=dict(
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                ),
                zaxis=dict(
                    backgroundcolor='white',
                    gridcolor='lightgray',
                    zerolinecolor='lightgray',
                )
            )
        )
    )


fig = initialize_fig(plot, robot.state, path)
frames = [go.Frame(data=plot.generate_data(robot.state, path))]

for pos, vel in robot.move_node_along_path(config.move_node, path):
    frames.append(go.Frame(data=plot.generate_data(robot.state, path)))

fig.frames = frames

if __name__ == '__main__':
    fig.show()
