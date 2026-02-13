import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import cProfile
import pstats
import time

from rift.robot import RobotInverse
from rift.truss_config import ROVER_CONFIG as config
from conversion import *

SERIAL_PORT = 'COM4'
BAUDRATE = 115200
TICKS_PER_INCH = 1050

# TODO make new configuration for robot on the ground - see desmos basic iso robot
# TODO make file that will make robot move into crawling configuration
# TODO make file that will make robot move into rolling configuration
# TODO file that runs crawling motion
# TODO file that runs rolling motion




def main():
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.2) as ser:
        time.sleep(2.0)  # give Arduino time to reset
        read_positions(ser)  # clear initial buffer
        
        target_hz = 2.0
        period = 1.0 / target_hz



        t0 = time.time()
        with cProfile.Profile() as profile:
            robot = RobotInverse.from_config(config)
            i=0
            for *_,d_roll in robot.crawl(1, resolution=4):
                print(d_roll)
                cmnd = dist_to_ticks(16/12,d_roll)
                send_command(cmnd)
                positions_ticks = read_positions(ser)

                positions_feet = positions_ticks/TICKS_PER_INCH/12
                i+=1
                # need to convert d_roll(distance along tube of P=12) to actual distance along tube to steps by endcoder
                # set time to move along tube
                # turn this into vel_dur commands
                # send to transmitter
                pass
        print("commands = ",i)
        t1 = time.time()
        stats = pstats.Stats(profile).sort_stats(pstats.SortKey.CUMULATIVE)
        stats.print_stats(8)
        print(f"Total time (imprecise): {t1 - t0:.2g}")


if __name__ == '__main__':
    main()
