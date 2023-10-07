'''
This file resumes the training if a weight file name is specified in settings.py.
Otherwise, it starts training without any weight initialisation.
'''

from   datetime import  datetime
import os
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import argparse

import  gym
from    stable_baselines.common.policies  import MlpPolicy 
from    stable_baselines.common.callbacks import EvalCallback
from    stable_baselines.common.callbacks import CheckpointCallback
from    stable_baselines.common.callbacks import CallbackList
from    stable_baselines                  import DQN


from Gym_env import tb_env
import save_callback




if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument("initial_weights") # Pass the path to resume training or none
    parser.add_argument("n_steps")
    args = parser.parse_args()

    n_steps = int(args.n_steps)

    # Clear all previous weights except the initial weights if specified.
    path = 'logs/checkpoints/'
    f = []
    for (dirpath, dirnames, filenames) in os.walk(path):
        f.extend(filenames)
        break
    for file in f:
        if not file == args.initial_weights:
            os.remove(path+file)


    # to log the td_error
    with open ('logs/td_error.csv','w') as f:
        f.write('This file logs the td error per each update. Agent is trained in each episode.')

    env = tb_env()

    # Callbacks: EvalCallBack stores the best model. 
    # This callback evaluates the agent at specified interval during the training.
    eval_callback = EvalCallback(
                        eval_env = env, n_eval_episodes=3,
                        verbose=0, deterministic=True,
                        log_path='./logs/', eval_freq=2500, 
                        best_model_save_path='./logs/'
                        )
    # Callback to store the latest weights of the model at specified interval.
    checkpoint_callback = CheckpointCallback(
                                save_freq=2000, save_path='./logs/checkpoints/',
                                name_prefix='latest_model',verbose=0
                                )

    # custom callback for save
    save_callback = save_callback.CustomCallback()
    callback = CallbackList([checkpoint_callback, save_callback])

    model = DQN(
                'MlpPolicy', 
                env, 
                double_q=True, 
                gamma=0.99, 
                buffer_size= 1000000,	
                batch_size=64
                )

    start_time = datetime.now()  
    try:     
        # Load trained model    
        model.load(path + args.initial_weights)
        print('------ Started training using '+args.initial_weights+' weights -------')
    except:
        print('------ Started training without weight initialisation -------')

    # Start Training    
    model.learn(total_timesteps = n_steps, callback = callback)
    print('******DONE.******')
    print('TOTAL COMPUTATION TIME: ',datetime.now() - start_time,' HOURS')