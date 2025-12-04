import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

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
    animator = rift.anim.Animator(config)
    robot = RobotInverse.from_config(config)
    robot.pos_callback = animator.add_frame
    try:
        for _ in range(cycles):
            robot.crawl(step_length, resolution=resolution)
    except InverseKinematicsError as e:
        print(e.args[0])
    fig = animator.make_figure()
    fig.show()


if __name__ == '__main__':
    from rift.truss_config import CONFIG_ROVER as config
    main(config, cycles=4, resolution=100)
