import cProfile
import pstats

from rift import rover


def main():
    with cProfile.Profile() as profile:
        robot = rover.make_robot()
        for _ in robot.divide_steps(rover.crawl(10), resolution=50):
            pass
    stats = pstats.Stats(profile).sort_stats(pstats.SortKey.TIME)
    stats.print_stats(0.1)


if __name__ == '__main__':
    main()
