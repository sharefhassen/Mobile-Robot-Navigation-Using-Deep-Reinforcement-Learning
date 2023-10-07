# import matplotlib
# matplotlib.use("agg")
import matplotlib.pyplot as plt

filename = 'logs/log_rewards.csv'
with open(filename,'r') as f:
    data = f.readlines()
    data = data[1:]
print(data)

batch_size = 150    # How many rewards must be averaged

average_rewards = [] # List of average rewards
average_reward = 0

# Compute the average rewards
for index, reward in enumerate(data):
    if index%batch_size == 0:
        average_rewards.append(average_reward)
        average_reward = 0
    average_reward += float(reward)/batch_size
plt.plot(average_rewards)
plt.title('Reward vs episode plot')
plt.xlabel('Episodes x' + str(batch_size) + ' (each point is averaged over ' + str(batch_size) + ' values to reduce noise)')
plt.ylabel('Reward')

plt.show()
