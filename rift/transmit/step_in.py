import numpy as np




from ..truss_config import TrussConfig,setup_rover_builder

import cProfile
import pstats
import time

import rift.indices as I
from ..robot import RobotInverse
# from conversion imposrt *

linear_step= np.full((12,3),np.nan)
t = 1.5 # seconds

config = setup_rover_builder(1)

def move_node_linear(configuration,final_pos,steps,time_step = 1.5):
    initial_pos = configuration.initial_pos
    change_pos = final_pos-initial_pos
    step_pos = change_pos/steps
    substep_pos = np.repeat(step_pos.reshape(12,3,1),steps,axis = -1)
    _,_,substep_grad = np.gradient(substep_pos)
    step_array = substep_grad[[I.L3,I.R3],:,:]
    print(step_array)


    # with cProfile.Profile() as profile:
    #         robot = RobotInverse.from_config(config)
    #         i=0
    #         for *_,d_roll in robot.take_step(step_array):
    #             print(d_roll)
    #             i+=1
    #             # need to convert d_roll(distance along tube of P=12) to actual distance along tube to steps by endcoder
    #             # set time to move along tube
    #             # turn this into vel_dur commands
    #             # send to transmitter
    #             pass
