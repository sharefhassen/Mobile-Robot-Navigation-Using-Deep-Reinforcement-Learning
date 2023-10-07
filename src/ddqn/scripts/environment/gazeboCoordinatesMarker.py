'''
This file is used to spawn a circle at the goal point 
identified by the vision system to visualise in gazebo.

Required only if gazebo simulator is used.

Use markCoordinates()
'''
import time
import os

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates



class gazeboCoordinatesMarker():
    def __init__(self, confidence_threshold, debug = False):
        self.confidence_threshold = confidence_threshold
        self.debug = debug
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('ddqn/scripts/environment',
                                                'turtlebot_simulator/turtlebot_gazebo/models/circle_mat_yellow/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model_yellow = self.f.read()
        self.f.close()

        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('ddqn/scripts/environment',
                                                'turtlebot_simulator/turtlebot_gazebo/models/circle_mat_green/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model_green = self.f.read()
        self.f.close()
        
        self.model = self.model_yellow
        self.goal_position = Pose()
        self.modelName = 'feedback'
        self.x_old = 0
        self.y_old = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.episode_initial = True 
        self.check_model = False


    def checkModel(self, model):
        '''
        This is a callback method.It works on a separate thread non stop.
        It constantly checks is the model with modelName exists in 
        gazebo world. 
        '''
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == self.modelName:
                self.check_model = True
        # rospy.sleep(0.5)



    def spawnMarker(self):
        if self.debug:
            print('spawnMarker')
        # while True:
        if not self.check_model:
            if self.debug:
                print('ok')
            rospy.wait_for_service('gazebo/spawn_sdf_model')
            spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                # break
            # else:
            #     rospy.sleep(0.2)


    def deleteMarker(self):
        '''
        Delete marker
        '''
        if self.debug:
            print('deleteMarker')
        # for iterations in range(5):
        if self.check_model:
            if self.debug:
                print('ok')
            rospy.wait_for_service('gazebo/delete_model')
            del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            del_model_prox(self.modelName)

            self.x_old = 0
            self.y_old = 0
            self.goal_position.position.x = None
            self.goal_position.position.y = None
            #     break
            # else:
            #     rospy.sleep(0.05)



    def markCoordinates(self, coordinates, confidence):
        ''' 
        Marks the coke coordinates identified by vision system position in gazebo.
        param coordinates : (list) coke can coordinates data recieved from 
            the object detector.
        param confidence : certainity of the object detector that the object 
            detected is a coke can.
        '''
        if confidence: 
            # Select the colour of the marker
            if confidence >= self.confidence_threshold:
                self.model = self.model_green
            else:
                self.model = self.model_yellow
    

            x,y,z = coordinates
            # # If the coordinates did not change atleast a threashold value (say 5cm)
            # # Need not respawn the goal point marker
            # if ( abs(self.x_old - x) > 0.05 ) or (abs(self.y_old - y) > 0.05 ):
            # if not self.episode_initial:
            #     print('Initial episode')
            #     self.deleteMarker()
            
            self.episode_initial = False
            self.goal_position.position.x = x
            self.goal_position.position.y = y

            self.spawnMarker()
            # self.x_old = x
            # self.y_old = y


        else:
            print('No coke can detected')



# if __name__ == "__main__":
#     rospy.init_node('coke_tf_lisener')

#     coke = cokeCoordinatesListener(gazebo=True)
#     while not rospy.is_shutdown():
#         coke.getCokePosition()
