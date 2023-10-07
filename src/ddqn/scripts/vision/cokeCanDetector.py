'''
This file returns the coordinates of a coke can with respect to odom 
frame using kinect 3D camera topics.

Functionalities:
- Identify pixels of coke using deeplearning techniques (imageClassifier.py)
- Get the real XYZ coordinates from the pixel and depth data (coordinateFetcher.py)
- Publish the transform of new frame /coke  (tfManager.py)

'''

import rospy

from coordinateFinder import cooordinateFinder
from tfManager import tfManager
from objectDetector import objectDetector


if __name__ == "__main__":        
    # Initialise node
    rospy.init_node( 'coke_tf_broadcaster')
    br = tfManager(new_frame_name = 'coke', ref_frame='camera_depth_optical_frame')
    cokefinder = objectDetector()
    kinect = cooordinateFinder()



    while not rospy.is_shutdown():
        # Get the coordinates of coke
        location = cokefinder.detectCoke()
        if not type(location) is type(None):
            # Get the xyz cordinates in space for the pixels u, v
            x,y,z = kinect.get_xyz(location)
            # Publish transforms
            br.broadcast(xyz=[x,0,z])

        else:
            br.ref_frame = 'odom'
            br.broadcast(xyz=[0,0,0])
            br.ref_frame='camera_depth_optical_frame'
            rospy.sleep(0.1)

