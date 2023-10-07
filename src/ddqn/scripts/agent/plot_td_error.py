# import matplotlib
# matplotlib.use("agg")
import matplotlib.pyplot as plt

filename = 'logs/td_error.csv'
with open(filename,'r') as f:
    data = f.readlines()
    data = data[1:]

batch_size = 100    # How many rewards must be averaged

average_rewards = [] # List of average rewards
gamma_legends = []
average_reward = 0


# Compute the average rewards
for index, reward in enumerate(data):
    if index%batch_size == 0 and index != 0:
        average_rewards.append(average_reward)
        average_reward = 0
    average_reward += float(reward)/batch_size
plt.plot(average_rewards)
plt.title('Td error vs episode')
plt.xlabel('Episodes x' + str(batch_size) + ' (each point is averaged over ' + str(batch_size) + ' values to reduce noise)')
plt.ylabel('TD error')

plt.show()
