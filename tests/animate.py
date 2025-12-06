import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import rift.anim
from rift.robot import InverseKinematicsError, RobotInverse
from rift.truss_config import TrussConfig
from rift.typing import Matrix


def main(
    config: TrussConfig,
    *,
    step_length: float = 0.8,
    cycles: int = 1,
    resolution: int = 50,
) -> None:
    animator = rift.anim.Animator.from_config(config)
    robot = RobotInverse.from_config(config)
    positions: list[Matrix] = []
    try:
        for _ in robot.crawl(cycles, step_length, resolution=resolution):
            positions.append(robot.pos.copy())
    except InverseKinematicsError as e:
        print(e.args[0])
    animator.animate(positions)


if __name__ == '__main__':
    from rift.truss_config import CONFIG_ROVER as config
    main(config, cycles=4, resolution=100)
