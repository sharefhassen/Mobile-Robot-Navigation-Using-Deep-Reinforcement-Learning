import numpy as np
import os
import sys
import tensorflow as tf
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from copy import deepcopy




class objectDetector:
	def __init__(self, 
				image_topic = '/camera/rgb/image_raw',
				confidence = 0.2
				):
		'''
		param image_topic : topic to subscribe to to get the image data from ros
		param confidence : minimum probability to filter weak detections
		'''
		self.image_ = None
		self.image_topic = image_topic
		self.confidence = confidence

		# Path to frozen detection graph. This is the actual model that is 
		# used for the object detection.

		PATH_TO_CKPT = p = os.path.dirname(os.path.abspath(__file__)) + '/frozen_inference_graph.pb'
		self.detection_graph = tf.Graph()
		with self.detection_graph.as_default():
			od_graph_def = tf.GraphDef()
			with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
				serialized_graph = fid.read()
				od_graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(od_graph_def, name='')

		self.bridge = CvBridge()
		# Initialise image subscriber
		rospy.Subscriber(self.image_topic, Image, self.imageCallback)



	def imageCallback(self, msg):
		''' Callback function to read the image topic form ros'''
		self.image_ = self.bridge.imgmsg_to_cv2(msg, "bgr8")



	def detectCoke(self):
		'''
		This method classifies the objects seen by the kinect camera 
		and identifies the coke.

		return: pixel coordinates of the bottle (x,y) if coke is detected.
		'''
    	# Get the image from the topic
		self.image_ = None
		for iteration in range(1000): 
			if type(self.image_) is type(None):
				time.sleep(0.015)

			else:
				self.image = deepcopy(self.image_)
				break
		if type(self.image_) == type(None):
			raise Exception('Failed to subscribe to '+ self.image_topic +' topic.')

		with self.detection_graph.as_default():
			with tf.Session(graph=self.detection_graph) as sess:
				# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
				image_np_expanded = np.expand_dims(self.image, axis=0)
				image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
				# Each box represents a part of the image where a particular object was detected.
				boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
				# Each score represent how level of confidence for each of the objects.
				# Score is shown on the result image, together with the class label.
				scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
				self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
				num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
				# Actual detection.
				(boxes, scores, self.classes, num_detections) = sess.run(
					[boxes, scores, self.classes, num_detections],
					feed_dict={image_tensor: image_np_expanded})

				score_index = np.where(np.squeeze(scores)>self.confidence)
				box = np.squeeze(boxes)[score_index]
				category =np.squeeze(self.classes)[score_index]
				size = category.size

				detection_confidence = 0
				x = 0
				y = 0
				for i in range(size):
					self.classes = ['coke_can','plate','table','pan']
					item = self.classes[int(category[i])-1]
					c = np.squeeze(scores)[score_index]

					if item == 'coke_can':
						# print('confidence : ', c[i])
						# print('confidence : ', c[int(category[i])-1])
						detection_confidence = c[int(category[i])-1]
						y_start = box[i][0] * 480
						x_start = box[i][1] * 640
						y_end = box[i][2] * 480
						x_end = box[i][3] * 640 
						# Return a point on the coke can.
						# It has to be an integer
						x = int((x_start + x_end)/2)
						y = int((y_end - y_start) * 0.8 + y_start)
						break
				return([x,y], detection_confidence)

if __name__=="__main__":
	rospy.init_node('image_listener')
	bottlefinder = objectDetector()
	while not rospy.is_shutdown():
		location = bottlefinder.detectCoke()
		print(location)