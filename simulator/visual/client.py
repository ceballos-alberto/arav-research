#!/usr/bin/env python3

""" ----------------------------------------------------------------------------
    ---------------------- Gazebo Client - ARAV Simulator ----------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """

# Import required libraries #

import rospy
import numpy as np
import cv_bridge
import cv2
import sys
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

# Required paths #

level = 0
i = 0

for letter in sys.argv[1]:

	i+=1

	if letter == "/":

		level+=1

	if level == 3:

		INIT_PATH = "../" + sys.argv[1][i:] + "/visual/init.png"
		WAIT_PATH = "../" + sys.argv[1][i:] + "/visual/waiting.png"
		SAVE_PATH = "../" + sys.argv[1][i:] + "/visual/output"
		break

# Input topic #

INPUT_TOPIC = "/arav/sensors/visualCamera/image";

DETECTION_TOPIC = "/arav/visual/detection";
DEPTH_TOPIC = "/arav/visual/depth";
CLOUD_TOPIC = "/arav/visual/cloud";

STATUS_TOPIC = "/arav/EPM/AIFilterStatus";

# Script modes #

if sys.argv[2] == "true":

	save = True

else:

	save = False

# Update rate #

UPDATE_RATE = float (sys.argv[3]) 	# Hz #

# Image Size --> HD 720p #

WIDTH = 1280		# This line can be modified #
HEIGHT = 720		# This line can be modified #
CROP1 = 81 		# This line can be modified #
CROP2 = 144		# This line can be modified #

# Listener class definition #

class Listener:

	# Constructor method #

	def __init__ (self):

		self.bridge = cv_bridge.CvBridge()
		self.image = cv2.imread(INIT_PATH)
		self.detection = cv2.imread(WAIT_PATH)
		self.depth = cv2.imread(WAIT_PATH)
		self.cloud = cv2.imread(WAIT_PATH)
		self.status = 0

	# Class methods #
	
	def statusCallback (self, msg):
	
		# To be executed when an status message is received #
		
		self.status = msg.data

	def imageCallback (self, msg):

		# To be executed when an image is received #

		try:

			self.image = self.bridge.imgmsg_to_cv2 (msg, "bgr8")

		except CvBridgeError as e:

			print ("[Client] Error while loading frame")
			
	def detectionCallback (self, msg):

		# To be executed when an image is received #

		try:

			self.detection = self.bridge.imgmsg_to_cv2 (msg, "bgr8")

		except CvBridgeError as e:

			print ("[Client] Error while loading frame")
			
	def depthCallback (self, msg):

		# To be executed when an image is received #

		try:

			self.depth = self.bridge.imgmsg_to_cv2 (msg, "bgr8")

		except CvBridgeError as e:

			print ("[Client] Error while loading frame")
			
	def cloudCallback (self, msg):

		# To be executed when an image is received #

		try:

			self.cloud = self.bridge.imgmsg_to_cv2 (msg, "bgr8")

		except CvBridgeError as e:

			print ("[Client] Error while loading frame")

# Listener declaration #

listener = Listener ()

# Initializing ROS node #

rospy.init_node('visualizer')

# ROS subscriber #

sub = rospy.Subscriber(INPUT_TOPIC, Image, listener.imageCallback, queue_size = 1)
sub_status = rospy.Subscriber(STATUS_TOPIC, Int8, listener.statusCallback, queue_size = 1)
sub_detection = rospy.Subscriber(DETECTION_TOPIC, Image, listener.detectionCallback, queue_size = 1)
sub_depth = rospy.Subscriber(DEPTH_TOPIC, Image, listener.depthCallback, queue_size = 1)
sub_cloud = rospy.Subscriber(CLOUD_TOPIC, Image, listener.cloudCallback, queue_size = 1)
rate = rospy.Rate(UPDATE_RATE)

# Main loop #

init_time = time.time()

while not rospy.is_shutdown():

    # Load Frame #
    frame = listener.image
    detection = listener.detection
    depth = listener.depth
    depth = depth[CROP1:(720-CROP1), CROP2:(1280-CROP2)]
    depth = cv2.resize(depth, (WIDTH, HEIGHT))
    cloud = listener.cloud
    status = listener.status
    compound = np.concatenate((np.concatenate((frame,detection),axis=0),np.concatenate((depth,cloud),axis=0)),axis=1)

    # Resize Frame #
    compound = cv2.resize(compound, (WIDTH, HEIGHT))

    if status < 2:
    	# Display #
    	cv2.imshow("ARAV Simulator", frame)
    	cv2.waitKey(1)
    
    else:
    	# Display #
    	cv2.imshow("ARAV Simulator", compound)
    	cv2.waitKey(1)

    # Save #
    if save:
        cv2.imwrite(SAVE_PATH + "/output_" + str(time.time()-init_time)[:4] + "_segs.png", compound)

    # Wait until next iteration #
    rate.sleep ()

# Close OpenCV windows #
cv2.destroyAllWindows()

""" ----------------------------------------------------------------------------
    ---------------------- Gazebo Client - ARAV Simulator ----------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """
