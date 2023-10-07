'''
This file is used to test a specified wight file in settings.py
'''

from    datetime import  datetime
from    time import sleep
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import  gym

from    stable_baselines.common.policies  import MlpPolicy 
from    stable_baselines.common.callbacks import EvalCallback
from    stable_baselines.common.callbacks import CheckpointCallback
from    stable_baselines.common.callbacks import CallbackList
from    stable_baselines                  import DQN


from Gym_env import tb_env
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("weights") # Pass the name of the weights
args = parser.parse_args()

if __name__ == "__main__":
    env = tb_env(logging=False)
    model = DQN.load('logs/checkpoints/'+args.weights)

    for i in range(100):
        obs = env.reset()
        done = False
        while not done:
            action, _states = model.predict(obs)
            obs, _, done, info = env.step(action)