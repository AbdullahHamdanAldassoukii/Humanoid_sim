import sys
import time
import gym
import numpy as np
import math as Math
#py#from simulation.vrepSim.Bioloid import Bioloid
#spy#
from Bioloid import Bioloid
import gym_bioloid


ENV_ID="bioloid-v0"

env = gym.make(ENV_ID)


def delay(n):
    time.sleep(n)

# ############################################ POSTUREs ###############################################

INITIAL_POS = [336, 687, 298, 724, 412, 611, 355, 664, 491, 530, 394, 625, 278, 743, 616, 405, 490, 530]

STAND_POS = [512] * 18
STAND_POS[6] = 512 - 151
STAND_POS[7] = 512 + 151  # 151 = 45 degrees

# #####################################################################################################

robot = Bioloid()
robot.restart()

while True:
    robot.set_degree(1,robot.ipm())
    s,sl=robot.get_state_vector()
    #print(robot.get_reward(sl))
    print(s)
