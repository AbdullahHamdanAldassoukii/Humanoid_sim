from gym.envs.registration import register

register(
    id='humanoid-v0',
    entry_point='gym_humanoid.envs:HumanoidEnv',
)
