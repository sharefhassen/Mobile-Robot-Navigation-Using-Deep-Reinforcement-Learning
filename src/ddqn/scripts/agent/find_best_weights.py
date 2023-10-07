'''
This file is to find the best of all weights in the 'logs/checkpoints' directory.

This file loads each weights in the checkpoints directory and runs 6 episode on each weights.
The average reward after runing for 6 episodes is printed on the screen for each weightfiles.

Based in the average reward, the best best weight file can be chosen.
'''

from    datetime import  datetime
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import os

import  gym

from    stable_baselines.common.policies  import MlpPolicy 
from    stable_baselines.common.callbacks import EvalCallback
from    stable_baselines.common.callbacks import CheckpointCallback
from    stable_baselines.common.callbacks import CallbackList
from    stable_baselines                  import DQN


from Gym_env import tb_env

if __name__ == '__main__':
    # get all the files present in 'logs/checkpoints' directory
    path_checkpoints = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + '/agent/logs/checkpoints'
    f = []
    for (dirpath, dirnames, filenames) in os.walk(path_checkpoints):
        f.extend(filenames)
        break
    files = []
    for file in f:
        if '.zip' in file:
            files.append(file)



    env = tb_env(logging=False)

    batch_size = 6
    for filename in files:
        episode_reward = 0
        for i in range(batch_size):
            model = DQN.load('logs/checkpoints/'+filename)
            obs = env.reset()
            done = False
            steps = 0
            while not done:
                action, _states = model.predict(obs)
                obs, rewards, done, info = env.step(action)
                episode_reward += rewards

        average_reward = episode_reward/batch_size
        print(filename, average_reward)
    print('END')

    