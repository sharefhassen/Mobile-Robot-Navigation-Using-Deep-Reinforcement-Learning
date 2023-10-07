from    gym          import spaces,Env
from math import pi
import numpy as np
import socket
from time import sleep

class tb_env(Env):
    def __init__(self, logging = True, log_name = 'logs/log_rewards.csv'):
        self.logging = logging
        self.log_name = log_name
        self.s = socket.socket()   
        # self.s.settimeout(60)
        # Connect to the server
        for i in range(100):
            try:
                sleep(0.5)
                self.s.connect(('localhost',12340))
                break
            except:
                print('TRIED ------- ',i)
                pass
        self.episode_count = 0

        # Define the lower bound and upperbound for observation space. 
        # Laser range is in between 0.13 and 3.5
        # Angle is between -pi to +pi
        # Goal distance is between 0 and 5
        # Nearest obstacle distance is the lowest reading from laser scan
        # Obstacle angle is the id of the laser beam that hit the nearest object
        self.obs_low = np.array(
                                [0 for i in range(24)] +[0, -pi, 0, 0, 0]
                                )
        self.obs_high = np.array(
                                [3.5 for i in range(24)] +[1, pi, 5,  3.5, 23]
                                )

        # Define the observarion space. They must be gym.spaces objects.
        self.observation_space = spaces.Box(
                                            low = self.obs_low, 
                                            high = self.obs_high,
                                            dtype= np.int32
                                            )

        # Define the action space. They must be gym.spaces objects.
        self.action_space = spaces.Discrete(5)
        if self.logging:
            with open(self.log_name,'w') as f:
                f.write('')


    def parseState(self, data_recieved):
        try:
            # Parse the string recieved. Data is separated by !!
            state = []
            reward = 0
            done = False
            info = {}

            # decode bytes to string
            data = data_recieved.decode()


            # Get each element of the string into data list
            data = data.split('!!')[1:-1]

            # Separate state, reward, done
            state = []
            for i in data[0:-2]:
                state.append(float(i))
            reward = float(data[-2])
            if data[-1] == 'True':
                done = True
            elif data[-1] == 'False':
                done = False
            else:
                print(data)
                raise Exception('Improper state data recieved')

            # Episode reward
            self.episode_reward += reward
            if done:
                print('Episode ended')
                if self.logging:
                    with open(self.log_name,'a') as f:
                        f.write('\n' + str(self.episode_reward))
        except:
            raise Exception('ERROR IN THE STATE RECEIVED : ' + str(data_recieved) )
        return (state, reward, done, info)

        


    def reset(self):
        self.episode_count += 1
        # Episode reward
        self.episode_reward = 0
        print('Episode ', self.episode_count)
        # 9 represents reset
        if not self.s.send(b'9'):
            print('CONNECTION LOST WITH SERVER')
        recv = self.s.recv(300)
        state,_,_,_ = self.parseState(recv)
        return (state)

    def step(self,action):
        action = str(action)
        self.s.send(action.encode())
        recv = self.s.recv(300)
        return (self.parseState(recv))

    




