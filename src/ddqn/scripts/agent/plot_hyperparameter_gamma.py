# import matplotlib
# matplotlib.use("agg")
import matplotlib.pyplot as plt

gammas = [0.75,0.85,0.9,0.95,0.99]

for gamma in gammas:
    filename = 'hyperparameter_gamma/log_' + str (gamma) + '_gamma_rewards.txt'
    with open(filename,'r') as f:
        data = f.readlines()
        data = data[1:1000]

    batch_size = 100    # How many rewards must be averaged

    average_rewards = [] # List of average rewards
    gamma_legends = []
    average_reward = 0


    # Compute the average rewards
    for index, reward in enumerate(data):
        if index%batch_size == 0 and index!=0:
            average_rewards.append(average_reward)
            average_reward = 0
        average_reward += float(reward)/batch_size
    plt.plot(average_rewards)
    gamma_legends.append('discount factor = ' + str(gamma))
    plt.legend(gammas)
plt.title('Reward vs episode for different values of discount factor (gamma) during training')
plt.xlabel('Episodes x' + str(batch_size) + ' (each point is averaged over ' + str(batch_size) + ' values to reduce noise)')
plt.ylabel('Reward')

plt.show()
