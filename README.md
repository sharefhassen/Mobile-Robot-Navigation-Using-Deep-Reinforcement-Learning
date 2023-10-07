# Mobile-Robot-Navigation-Using-Deep-Reinforcement-Learning
This work is implemented in paper Mobile Robot Navigation Using Deep Reinforcement Learning published in Processes Journal. 
# Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.
# Prerequisites
Ubuntu 16.04
ROS Kinetic
Python2
python3.6
tensorflow-gpu==1.14 (python3)
stable-baselines
pyyaml, rospkg, gym, & tensorflow==1.13.2
tensorflow-gpu==1.14 (python2)
# Author 
Sharef Yusuf
# Paper
If you use this code in your research, please cite our Processes paper
@Article{pr10122748,
AUTHOR = {Lee, Min-Fan Ricky and Yusuf, Sharfiden Hassen},
TITLE = {Mobile Robot Navigation Using Deep Reinforcement Learning},
JOURNAL = {Processes},
VOLUME = {10},
YEAR = {2022},
NUMBER = {12},
ARTICLE-NUMBER = {2748},
URL = {https://www.mdpi.com/2227-9717/10/12/2748},
ISSN = {2227-9717},
ABSTRACT = {Learning how to navigate autonomously in an unknown indoor environment without colliding with static and dynamic obstacles is important for mobile robots. The conventional mobile robot navigation system does not have the ability to learn autonomously. Unlike conventional approaches, this paper proposes an end-to-end approach that uses deep reinforcement learning for autonomous mobile robot navigation in an unknown environment. Two types of deep Q-learning agents, such as deep Q-network and double deep Q-network agents are proposed to enable the mobile robot to autonomously learn about collision avoidance and navigation capabilities in an unknown environment. For autonomous mobile robot navigation in an unknown environment, the process of detecting the target object is first carried out using a deep neural network model, and then the process of navigation to the target object is followed using the deep Q-network or double deep Q-network algorithm. The simulation results show that the mobile robot can autonomously navigate, recognize, and reach the target object location in an unknown environment without colliding with static and dynamic obstacles. Similar results are obtained in real-world experiments, but only with static obstacles. The DDQN agent outperforms the DQN agent in reaching the target object location in the test simulation by 5.06%.},
DOI = {10.3390/pr10122748}
}



