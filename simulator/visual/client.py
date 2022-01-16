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
import cv_bridge
import cv2
import sys
import time
from sensor_msgs.msg import Image

# Required paths #

level = 0
i = 0

for letter in sys.argv[1]:

	i+=1

	if letter == "/":

		level+=1

	if level == 3:

		INIT_PATH = "../" + sys.argv[1][i:] + "/visual/init.png"
		SAVE_PATH = "../" + sys.argv[1][i:] + "/visual/output"
		break

# Input topic #

INPUT_TOPIC = "/ARAV/sensors/visualCamera/image";

# Script modes #

if sys.argv[2] == "true":

	save = True

else:

	save = False

# Update rate #

UPDATE_RATE = float (sys.argv[3]) 	# Hz #

# Image Size --> Full HD 1080p #

WIDTH = 1920		# This line can be modified #
HEIGHT = 1080		# This line can be modified #

# Listener class definition #

class Listener:

	# Constructor method #

	def __init__ (self):

		self.bridge = cv_bridge.CvBridge()
		self.image = cv2.imread(INIT_PATH)

	# Class methods #

	def imageCallback (self, msg):

		# To be executed when an image is received #

		try:

			self.image = self.bridge.imgmsg_to_cv2 (msg, "bgr8")

		except CvBridgeError as e:

			print ("[Client] Error while loading frame")

# Listener declaration #

listener = Listener ()

# Initializing ROS node #

rospy.init_node('visualizer')

# ROS subscriber #

sub = rospy.Subscriber(INPUT_TOPIC, Image, listener.imageCallback, queue_size = 100)
rate = rospy.Rate(UPDATE_RATE)

# Main loop #

init_time = time.time()

while not rospy.is_shutdown():

    # Load Frame #
	frame = listener.image

    # Display #
    #cv2.imshow("ARAV Simulator", frame)
    cv2.waitKey(15)

    # Save #
    cv2.imwrite(SAVE_PATH + "/output_" + str(time.time()-init_time)[:4] + "_segs.png", frame)

    # Wait until next iteration #
    rate.sleep ()

""" ----------------------------------------------------------------------------
    ---------------------- Gazebo Client - ARAV Simulator ----------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """
