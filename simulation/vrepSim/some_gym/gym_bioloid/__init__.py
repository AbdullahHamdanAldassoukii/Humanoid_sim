# -*- coding: utf-8 -*-
from gym.envs.registration import register

register(
    id='bioloid-v0',
    entry_point='gym_bioloid.envs:BioloidEnv',
)
