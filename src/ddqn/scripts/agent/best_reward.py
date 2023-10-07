'''
This file contains methods for save_callback.py
'''


def read_file(file_name):
    with open(file_name,'r') as f:
            data = f.readlines()
            data = data[1:] # to remove the first line
            return(data)

def highest_value(file_name):
    data = read_file(file_name)
    highest_value = -100000
    for reward in data:
        if float(reward) > float(highest_value):
            highest_value = reward
    return (float(highest_value))

def last_reward(file_name):
    data = read_file(file_name)
    return (float(data[-1]))

def n_episodes(file_name):
    data = read_file(file_name)
    return(len(data))

if __name__ == "__main__":
    print('Highest reward : ',highest_value(file_name))
