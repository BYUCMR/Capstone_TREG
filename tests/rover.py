import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import dash
import numpy as np
from dash import dcc, html
import plotly.graph_objects as go

from rift.anim import RoverPlotter3D
from rift.linalg import roll_pitch_yaw
from rift.path import make_path
from rift.robot import RobotInverse
from rift.truss_config import CONFIG_3D_ROVER1 as config

path = make_path(xform=roll_pitch_yaw(np.pi/4, np.pi/6, np.pi/4))
path += config.initial_pos[config.move_node]
robot = RobotInverse(config)
plot = RoverPlotter3D(robot)


def initialize_fig(plotter):
    path_scatter = plotter.plot_path(path)
    triangle_scatter = plotter.plot_triangles()
    payload_scatter, payload_mesh = plotter.plot_payload()

    figure = go.Figure(data=[payload_mesh, path_scatter] + payload_scatter + triangle_scatter,
                       layout=go.Layout(
                           xaxis=dict(range=[0, 5], autorange=False),
                           yaxis=dict(range=[0, 5], autorange=False),
                           title=dict(text="Start Title"),
                           font=dict(color="black"),
                           updatemenus=[dict(
                               type="buttons",
                               buttons=[dict(label="Play",
                                             method="animate",
                                             args=[None, {"frame": {"duration": 50, "redraw": True},
                                                          "transition": {"duration": 0}}])],
                               font=dict(color="black")
                           )]
                       )
                       )

    figure.update_layout(
        paper_bgcolor='white',
        plot_bgcolor='white',
        scene=dict(
            bgcolor='white',
            xaxis=dict(
                backgroundcolor='white',
                gridcolor='lightgray',
                zerolinecolor='lightgray'
            ),
            yaxis=dict(
                backgroundcolor='white',
                gridcolor='lightgray',
                zerolinecolor='lightgray'
            ),
            zaxis=dict(
                backgroundcolor='white',
                gridcolor='lightgray',
                zerolinecolor='lightgray'
            ),
        )
    )
    return figure


def create_frame(plotter):
    path_scatter = plotter.plot_path(path)
    triangle_scatter = plotter.plot_triangles()
    payload_scatter, payload_mesh = plotter.plot_payload()

    frame = go.Frame(data=[payload_mesh, path_scatter] + payload_scatter + triangle_scatter)
    return frame


# make fig


app = dash.Dash(__name__)

fig = initialize_fig(plot)
frames = [create_frame(plot)]

for pos, vel in robot.move_node_along_path(config.move_node, path):
    frames.append(create_frame(plot))

fig.frames = frames
app.layout = html.Div([
    dcc.Graph(
        id="the-plot",
        figure=fig  # <--- just put the whole animated figure here
    )
])

if __name__ == '__main__':
    app.run(debug=True)
