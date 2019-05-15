# -*- coding: utf-8 -*-

import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import time

import os, sys, inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
parentdir = os.path.dirname(parentdir)
parentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
from Bioloid import Bioloid

ACTION_LIMIT = 3.14 / 20
NUM_OF_MOTORS = 18
NUM_OF_ACTIVE_MOTORS=8
STATE_VECTOR_LENGTH = 47
INITIAL_POS = [336, 687, 298, 724, 412, 611, 355, 664, 491, 530, 394, 625, 278, 743, 616, 405, 490, 530]


class BioloidEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):

        self.seed()
        self.viewer = None
        self.robot = Bioloid()
        self.reset()
        high = np.array([np.inf] * STATE_VECTOR_LENGTH)
        self.action_space = spaces.Box(np.array([-ACTION_LIMIT] *NUM_OF_ACTIVE_MOTORS ),
                                       np.array([ACTION_LIMIT] * NUM_OF_ACTIVE_MOTORS),
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

    """
    :param :    * action  this var must be a 18-vec (array) in [radian]
    
    :return :   * state_vector : 47-vec (array)
                * reward : (float) reward of the step taken 
                * done : (bool) if the eps ends or not
                * {} no thing
    """

    def step(self,
             action  # this var must be a 8-vec (array) in [radian]
             ):

        if action.__class__ != list:
            action_noise = np.concatenate(action)
        else:
            action_noise = action
        # this is action in  radian
        action_IPM = self.robot.dynamexil2rad(self.robot.ipm())
        action_noise = np.r_[[0]*10,action_noise]
        applied_action = action_IPM + action_noise
        print(applied_action)
        self.robot.set_degree(1, applied_action, mode='rad')
        state_vector, state_list = self.robot.get_state_vector()
        reward = self.robot.get_reward(state_list)

        done = self.robot.isFall(state_list)

        return state_vector, reward, done, {}

    def reset(self):
        self.robot.restart()
        return self.step([0] * 8)[0]

    def render(self, mode='human'):
        pass

    def close(self):
        self.robot.clear()
        pass


if __name__ == "__main__":
    BioloidEnv()
