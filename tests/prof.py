import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import cProfile
import pstats
import time

from rift.robot import RobotInverse
from rift.truss_config import CONFIG_ROVER as config


def main():
    t0 = time.time()
    with cProfile.Profile() as profile:
        robot = RobotInverse.from_config(config)
        for _ in robot.crawl(10):
            pass
    t1 = time.time()
    stats = pstats.Stats(profile).sort_stats(pstats.SortKey.CUMULATIVE)
    stats.print_stats(8)
    print(f"Total time (imprecise): {t1 - t0:.2g}")


if __name__ == '__main__':
    main()
