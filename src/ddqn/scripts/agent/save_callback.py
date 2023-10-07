'''
This file creates a callback class to save the best weights.
'''

from stable_baselines.common.callbacks import BaseCallback
from best_reward import last_reward, highest_value, n_episodes



class CustomCallback(BaseCallback):

    def __init__(self, log_file_name = 'logs/log_rewards.csv', name = 'ddqn', verbose = 0):
        super(CustomCallback, self).__init__(verbose)
        self.n_episodes = 0
        self.save_count = 0
        self.name = name
        self.file_name = log_file_name


    def _on_step(self) -> bool:
        if self.n_episodes < n_episodes(self.file_name): # i.e., one episode ended
            print('Steps: ',self.num_timesteps)
            self.n_episodes = n_episodes(self.file_name)
            if (highest_value(self.file_name) == last_reward(self.file_name)):
                self.save_count += 1
                print('Saved the best model')
                self.model.save('logs/checkpoints/'+ self.name +'_best_model_'+str(self.num_timesteps))
        return True

