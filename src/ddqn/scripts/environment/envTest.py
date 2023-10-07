'''
This environment is to test the agent.

Always zero reward is returned by the step() function.
'''

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
from gazeboCoordinatesMarker import gazeboCoordinatesMarker

from spawn_obstacles import spawnObstacles

class Env():
    def __init__(
                self,
                action_size,
                confidence_threshold,
                gazebo=False,
                mark_goals=False,
                verbose=False,
                debug=False,
                report_freq=10,
                timeout = 200
                ):
        '''
        param action_size: (int) Size of action space
        param confidence_threshold : (float) Minimum confidence required
            for the object detecter prediction to fix the coordinates as
            corresponding to the coke can.
        param gazebo : (bool) Is gazebo simulator being run or not.
        param mark_goals : (bool) Whether the goal points must be
            marked in green/yellow circles in gazebo or not.
        param verbose : (bool) Should the data be printed on the screen.
        param debug : (bool) Run the code in debug mode.
        param report_freq : (int) frequency in which summary must be printed
        param timeout : number of steps in which the robot must grab a coke can
        '''
        self.debug = debug
        self.timeout = timeout
        self.verbose = verbose
        self.action_size = action_size
        self.confidence_threshold = confidence_threshold
        self.gazebo = gazebo
        if self.gazebo:
            self.mark_goals = mark_goals
        else:
            self.mark_goals = False
        self.goal_x = None
        self.goal_y = None
        self.i_goal_x = None
        self.i_goal_y = None
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn(2)
        self.episode_count = 0
        self.timer_goal = 0 # To count the time steps to reach a goal
        self.i_confidence = None # Intermediate confidance: when the robot thinks there may be a coke a specified point.




        if self.gazebo:
            o = spawnObstacles()

        if self.mark_goals:
            self.gazebo_marker = gazeboCoordinatesMarker(self.confidence_threshold, debug=self.debug)
        self.detected = False

        # To print the number of cokes picked, timeouts and collisions in n episodes frequency
        self.report_freq = report_freq
        self.n_cokes = 0
        self.n_timeouts = 0
        self.n_collisions = 0

    def getGoalDistace(self, x, y):
        '''
        Method to return the distance to the goal point from the current
        location of the robot. If the goal point is not identified, it
        returns a large distance (5 meters) as the distance value.
        '''
        if type(x) is type(None):
            # i.e if the coke bottle is not located yet, return 5meteres as distance
            return (5)
        else:
            goal_distance = round(math.hypot(x-self.position.x, y-self.position.y),2)
        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        if self.goal_x:
            goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

            heading = goal_angle - yaw
            if heading > pi:
                heading -= 2 * pi
            elif heading < -pi:
                heading += 2 * pi
            self.heading = round(heading, 2)
        elif self.i_goal_x:
            goal_angle = math.atan2(self.i_goal_y - self.position.y, self.i_goal_x - self.position.x)

            heading = goal_angle - yaw
            if heading > pi:
                heading -= 2 * pi
            elif heading < -pi:
                heading += 2 * pi
            self.heading = round(heading, 2)
        else:
            self.heading = 0



    def getState(self, coordinates, confidence, scan):

        '''

        param coordinates : (list) coordinates data recieved in this iteration.
                If the coordinates reccieved is [0,0,0], it means that the
                coordiates of coke is not identified.
        param confidence : (float) certainity of object detector that the
                detected object is be a coke can
        param scan : laser scan data

        How the goal point is set based on the data from the vision system:
            1. Once the turtlebot detects a coke can (even with low confidance)
                set that as an intermediate goal (if its confidance is more than prev confidance)
            2. If the confidance of the coke can detection is above a
                threshold value, fix that object as the goal point.
            3. Find the distance to the intermediate goal/finalised goal point
                and update 'heading' and 'current_distance'. Now state is set for this iteration.
            4. If goal point is not finalised and still in intermediate
                (low confidence) one, the turtlebot is close to the
                intermediate goal (say less than 1 meter), check the
                confidance value:
                    If the confidance is more than threshold_confidence value,
                        set the goal point as that goal point.
                    Else, discard the intermediate coordinates.
            5. Once the goal pont is set, provide a reward for finding the goal point.

        '''

        done = False

        # Kinect (goal point) related:
        if type(self.goal_x) is type(None):
            if confidence >= self.confidence_threshold:
                if self.verbose:
                    print('Fixed goal point (confidence : ' + str(round(confidence,2)) + ') at (' + str(round(coordinates[0],2)) + ', ' + str(round(coordinates[1],2))+')')
                self.goal_distance = self.getGoalDistace(self.goal_x, self.goal_y)
                self.i_confidence = confidence
                # Show the coordinates detected in gazebo
                if self.mark_goals:
                    # Remove the coke vision marker
                    if self.gazebo_marker.check_model:
                        self.gazebo_marker.deleteMarker()
                        rospy.sleep(0.5)
                    # Spawn green marker
                    self.gazebo_marker.markCoordinates(coordinates, confidence)
                self.detected = True
                self.goal_x = coordinates[0]
                self.goal_y = coordinates[1]
                current_distance = self.getGoalDistace(self.goal_x, self.goal_y)

            else:
                if confidence > self.i_confidence:
                    if self.verbose:
                        print('Assumed goal point (confidence : '+ str(round(confidence,2)) + ') is at (' + str(round(coordinates[0],2)) + ', ' + str(round(coordinates[1],2))+')')
                    # Show the coordinates detected in gazebo
                    if self.mark_goals:
                        # Remove the coke vision marker
                        if self.gazebo_marker.check_model:
                            self.gazebo_marker.deleteMarker()
                        self.gazebo_marker.markCoordinates(coordinates, confidence)
                    self.i_confidence = confidence
                    self.i_goal_x = coordinates[0]
                    self.i_goal_y = coordinates[1]
                    current_distance = self.getGoalDistace(self.i_goal_x, self.i_goal_y)
                else:
                    current_distance = self.getGoalDistace(self.i_goal_x, self.i_goal_y)


                # If the robot is close to the target and yet if it has no
                # confidence over threshold value, reset the goal point.
                if current_distance <= 1:
                    if self.verbose:
                        print('Rejected the assumed goal at (' + str(round(self.i_goal_x,2)) + ', ' + str(round(self.i_goal_y,2))+')' +' since the agent is not confident enough')

                    self.i_confidence = 0
                    self.i_goal_x = None
                    self.i_goal_y = None

                    if self.mark_goals:
                        # Remove the coke vision marker
                        if self.gazebo_marker.check_model:
                            self.gazebo_marker.deleteMarker()
                            self.gazebo_marker.episode_initial = True
        else:
            current_distance = self.getGoalDistace(self.goal_x, self.goal_y)

        # Laser related:
        scan_range = []
        min_range = 0.2 # Minimum collision distance
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = True

        if current_distance < 0.3:
            self.get_goalbox = True

        if self.timer_goal >= self.timeout:
            self.n_timeouts += 1
            done = True
            if self.verbose:
                print("Timed out to reach the goal !")
        return (scan_range + [self.i_confidence, self.heading, current_distance, obstacle_min_range, obstacle_angle], done)


    def checkSuccess(self, state, done, action):

        # yaw_reward = []
        obstacle_min_range = state[-2]
        current_distance = state[-3]
        heading = state[-4]

        for i in range(5):
            if self.goal_x:
                angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
                tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            else:
                tr = 0
            # yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)

        # if obstacle_min_range < 0.5:
        #     ob_reward = -5
        # else:
        #     ob_reward = 0

        # reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) + ob_reward

        if done:
            self.n_collisions += 1
            if self.verbose:
                print("Collision !")
            # reward = -500
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            self.n_cokes += 1
            self.timer_goal = 0
            if self.verbose:
                print("Reached goal !")
            # reward = 1000
            self.pub_cmd_vel.publish(Twist())
            self.respawn_goal.getPosition(True, delete=True)
            if self.mark_goals:
                self.gazebo_marker.deleteMarker()
                self.gazebo_marker.episode_initial= True
            self.goal_x = None
            self.goal_y = None
            self.i_goal_x = None
            self.i_goal_y = None
            self.i_confidence = 0
            self.goal_distance = self.getGoalDistace(self.goal_x, self.goal_y)
            self.get_goalbox = False

        if self.detected:
            # reward += 500
            self.detected =False






    def getScanData(self):
        '''
        Method to read the laser scanner data
        Returns the data recieved from scan topic and a flag indicating if
        the scan data was successfully retrieved or not.
        '''
        scan_data = None
        for i in range(50):
            try:
                scan_data = rospy.wait_for_message('scan', LaserScan, timeout=1)
                scan_flag = True
                break
            except:
                scan_flag = False
                print('SCAN ERROR')
                rospy.sleep(0.03)
        return(scan_data, scan_flag)




    def step(self, action, coordinates, confidence):
        '''
        To be called for each iteration of turtlebot agent.
        Decides the working of the system based in the action
        and object detection.
        '''
        self.timer_goal = self.timer_goal + 1
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.13
        vel_cmd.angular.z = ang_vel

        self.pub_cmd_vel.publish(vel_cmd)
        scan_data, scan_flag = self.getScanData()
        if (scan_flag):
            state, done = self.getState(coordinates, confidence, scan_data)
            self.checkSuccess(state, done, action)
        else:
            print('reset due to scan error')
            state = self.reset()
            # reward = 0
            done = True
        reward = 0
        return np.asarray(state), reward, done


    def reset(self):
        '''
        Reset the environment and global variables
        '''
        # Print the report at every specified frequency
        if self.verbose:
            print('--- Reset episode ---')
            if self.episode_count and self.episode_count % self.report_freq == 0:
                print('-- SUMMARY OF LAST '+ str(self.episode_count) + ' EPISODES -- ')
                print(' - Number of cokes grabbed = '+str(self.n_cokes))
                print(' - Number of collisions = '+str(self.n_collisions))
                print(' - Number of timeouts = '+str(self.n_timeouts))
        self.episode_count +=1
        self.timer_goal = 0
        self.detected =False
        self.i_confidence = 0
        self.goal_x = None
        self.goal_y = None
        self.i_goal_x = None
        self.i_goal_y = None
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            pass
        scan_data,_ = self.getScanData()
        if self.mark_goals:
            # Remove the coke vision marker
            if self.gazebo_marker.check_model:
                self.gazebo_marker.deleteMarker()

            # Set the reset flag for gazebo
            self.gazebo_marker.episode_initial = True

        self.respawn_goal.getPosition(True, delete=True)
        self.goal_distance = self.getGoalDistace(self.goal_x, self.goal_y)
        state, done = self.getState([0,0,0],0,scan_data)
        return np.asarray(state)


if __name__ == "__main__":
    rospy.init_node('tb_env')
    env = Env(5)
    print('*****RESET****')
    env.reset()
    print('DONE')
