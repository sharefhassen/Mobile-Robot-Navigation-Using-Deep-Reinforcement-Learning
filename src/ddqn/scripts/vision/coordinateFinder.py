'''
This file extracts the coordinates of a specified pixel (u,v) using a 3D camera (kinect).
cooordinates_fetcher.getxyz(u,v) returns the real XYZ coordinates of given u,v pixels.

XYZ frame is same as the XYZ frame of the kinect (Z is the depth).

'''
import rospy
from  sensor_msgs.msg import Image
from  sensor_msgs.msg import CameraInfo
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import numpy as np

class cooordinateFinder:
    '''
    This class is to finds the xyz coordinates from the rgbd camera.
    - It reads the xyz values using PinholeCameraModel
    - It also reads the depth data of a given point using depth/image_raw topic subsciption
    - x,y,z retireved from pinhole camera model * depth / z = real XYZ coordinates

    '''
    def __init__(self, 
                topic_camera_info = '/camera/rgb/camera_info', 
                topic_depth = '/camera/depth/image_raw'):

        '''
        param topic_camera_info : topic for camera_info. This topic has data for camera matrix
        param depth_topic : topic to subscribe to to get the depth data (not pointcloud data)
        '''
        self.camera_model =PinholeCameraModel()
        self.bridge = CvBridge()
        self.topic_camera_info = topic_camera_info
        self.topic_depth = topic_depth
        self.depth = None

        # Get the camera matrix data for transformation
        self.get_camera_info()
        # Start depth data subscriber
        self.get_depth()



    def get_depth(self):
        ''' Updates self.depth with the depth data of a specified pixel '''
        rospy.Subscriber(self.topic_depth, Image, self.callback_get_depth)



    def callback_get_depth(self, depth_data):
        '''
        Callback for get_depth()
        This method updates self.depth with the depth data of (self.u, self.v) pixels.
        '''
        try:
            # Convert ROS image to CV image
            self.depth_image = self.bridge.imgmsg_to_cv2(
                                                        depth_data, 
                                                        "passthrough")
        except CvBridgeError, e:
            print e



    def get_camera_info(self):
        ''' 
        Get the camera information for camera matrix.
        This process needs to be done only once. Once the data is recieved, unsubscribe the topic.
        '''
        subs_cam = rospy.Subscriber(
                                    self.topic_camera_info, 
                                    CameraInfo, 
                                    self.callback_get_camera_info
                                    )
        # wait until data is recieved
        for iteration in range(1000):
            if self.camera_model.tf_frame == None:
                rospy.sleep(0.01)
            else:
                break

        # Check if data was recieved.
        if self.camera_model.tf_frame == None:
            raise Exception ('Could not recieve camera matrix information. Try restarting gazebo.')
        else:
            # Unsubscribe the topic once the data is recieved
            subs_cam.unregister()



    def callback_get_camera_info(self, camera_info):
        '''
        Callback for get_camera_info()
        This method retrives the camera information required for camera matrix.
        '''
        self.camera_model.fromCameraInfo(camera_info)



    # def get_data(self):
    #     ''' 
    #     Method to read depth information and camera properties 
    #     '''
    #     for iter in range(100):     
    #         if type(self.depth_image) is type(None):
    #             sleep(0.05)
    #         else:
    #             break
    #     # Check if the loop failed and report.
    #     if type(self.depth_image) is type(None):
    #         raise Exception('Failed to retrieve depth data')

    #     # Read the depth data of (u, v) pixel from the array as (v, u)
    #     self.depth = self.depth_image[self.v][self.u]



    def get_xyz(self,uv,d_image):
        ''' 
        Method to calculate real X Y Z coordinates from the depth data, 
        pixel coordinates and camera properties. 
        
        param uv : (list) x,y coordinate of the pixel of interest
        
        try 'rosrun image_view image_view image:=/camera/depth/image_raw' to 
        find the pixel coordinates of the depth image
        '''
        self.u = uv[0]
        self.v = uv[1]
        # self.depth_image = None
        # self.get_data()
        self.depth = d_image[self.v][self.u]
        x,y,z = self.camera_model.projectPixelTo3dRay([self.u,self.v])
        x = self.depth * x / z
        y = self.depth * y / z
        z = self.depth * z / z
        return (x,y,z)




# if __name__ == '__main__':
#     rospy.init_node('rgbd_data_listener', anonymous=True)
#     kinect = cooordinatesFetcher()
#     x,y,z = kinect.get_xyz(118,296)
#     print(x,z)


