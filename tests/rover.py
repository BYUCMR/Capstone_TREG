import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import plotly.graph_objects as go

import rift.anim
from rift.robot import InverseKinematicsError, RobotInverse
from rift.truss_config import TrussConfig


def main(
    config: TrussConfig,
    *,
    step_length: float = 0.8,
    cycles: int = 1,
    resolution: int = 50,
) -> None:
    robot = RobotInverse.from_config(config)
    fig = rift.anim.initialize_fig(config, robot.state)
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
