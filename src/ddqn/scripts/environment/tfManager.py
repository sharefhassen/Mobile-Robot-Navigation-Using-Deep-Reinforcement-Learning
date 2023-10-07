'''
This file publishes and listens tf of a desired point in space.
Use tf_broadcaster.broadcast() to publish tf of any point in space.
Use tf_broadcaster.listen() to get tf of any point in space.

'''
import roslib
import rospy
import tf


class tfManager:
    ''' Class to broadcast tf of any custom point '''
    def __init__(self, new_frame_name, ref_frame ):
        '''
        param new_frame_name : name of the new frame to be published
        param ref_frame : name of the frame with respect to which the 
                          coordinates are provided to this code.
        '''
        self.new_frame_name = new_frame_name
        self.ref_frame = ref_frame

        # Initialise tf broadcaster
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        

    def broadcast(self, xyz=[0,0,0], rpy=[0,0,0], quaternion=False):
        '''
        This method publishes transform of given pose wrt to referance frame.

        param xyz : (list) x,y,z coordinates of the new frame wrt ref_frame
        param rpy : (list) r,p,y values of new frame
        param quaternion : (bool) True if the rotation is input as quaternion
        '''
        if quaternion:
            rot = tuple(rpy)
        else:
            rot = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
            
        self.broadcaster.sendTransform(
                                        tuple(xyz),
                                        rot,
                                        rospy.Time.now(),
                                        self.new_frame_name,
                                        self.ref_frame    
                                        )

    
    def listen(self):
        '''
        This method listens to the published transform and returns the transform to ref_frame
        '''
        trans = None
        for iteration in range(5000):
            try:
                (trans,rot) = self.listener.lookupTransform(self.ref_frame, self.new_frame_name, rospy.Time(0))
                return(trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(0.01)
                continue
        raise ('Failed to listen to transform')
            
        


# if __name__ == '__main__':
#     # Initialise node
#     rospy.init_node( new_frame_name + 'tf_broadcaster')
#     br = tfManager(new_frame_name = 'coke', ref_frame='odom')
#     while not rospy.is_shutdown() :
#         br.broadcast()
#         rospy.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node('test')
    br = tfManager(new_frame_name = 'coke', ref_frame='odom')
    br.broadcast()
    lis = tfManager(new_frame_name = 'coke', ref_frame='odom')
    t,r = lis.listen()              
    print(t,r)