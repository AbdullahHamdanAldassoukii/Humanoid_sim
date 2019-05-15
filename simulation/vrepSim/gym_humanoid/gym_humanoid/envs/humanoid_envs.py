import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
parentdir = os.path.dirname(parentdir)
parentdir = os.path.dirname(parentdir)
print(parentdir)
sys.path.insert(0,parentdir)
#from simulation.vrepSim.Bioloid import Bioloid
from Bioloid import Bioloid

import time

NUM_OF_MOTORS = 18
STATE_VECTOR_LENGTH = 47
INITIAL_POS = [336, 687, 298, 724, 412, 611, 355, 664, 491, 530, 394, 625, 278, 743, 616, 405, 490, 530]


class HumanoidEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.seed()
        self.viewer = None
        self.robot = Bioloid()
        self.reset()
        high = np.array([np.inf] * STATE_VECTOR_LENGTH)
        self.action_space = spaces.Box(np.array([-1] * NUM_OF_MOTORS),
                                       np.array([1] * NUM_OF_MOTORS),
                                       dtype=np.float32)
        self.observation_space = spaces.Box(-high,
                                            high,
                                            dtype=np.float32)
        print("end of gym Humanoid init")
        pass

    # K
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self.robot.set_degree(1, action)
        state_vector, state_list = self.robot.get_state_vector()
        reward = self.robot.get_reward(state_list)

        done = self.robot.isFall(state_list)

        return state_vector, reward, done, {}

    def reset(self):
        self.robot.restart()
        return self.step(INITIAL_POS)[0]

    def render(self, mode='human'):
        pass

    def close(self):
        pass
