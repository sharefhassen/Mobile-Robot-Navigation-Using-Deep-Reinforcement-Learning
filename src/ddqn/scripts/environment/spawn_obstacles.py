import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class spawnObstacles():
    def __init__(self, stage=2):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('ddqn/scripts/environment',
                                                'turtlebot_simulator/turtlebot_gazebo/models/cardboard_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = stage
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.obstacle_0 = -1.34, 1.65
        self.obstacle_1 = -1.58, -0.97
        self.obstacle_2 = 0.61, -2.08
        self.obstacle_3 = 0.67, -0.45
        self.obstacle_4 = 1.74, 1.05
        self.obstacle_5 = 2.76, -1.06
        self.positions = [self.obstacle_0, self.obstacle_1, self.obstacle_2, self.obstacle_3, self.obstacle_4, self.obstacle_5]
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.check_model = False
        self.index = 0


        for i in range(6):
            if i is not 3:
                self.modelName = 'box_' + str(i) +'lower'
                self.goal_position.position.x = self.positions[i][0]
                self.goal_position.position.y = self.positions[i][1]
                self.goal_position.position.z = 0.15
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")


                self.modelName = 'box_' + str(i) +'upper'
                self.goal_position.position.z = 0.45
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")


    