#!/usr/bin/env python3

""" ----------------------------------------------------------------------------
    ---------------- Artificial Intelligence Filter (AI-Filter) ----------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """

# Environment Configuration --> Hide TF Warnings #

import os
import sys
import time
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

# Import required libraries #

import tensorflow as tf
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int8
import cv_bridge
import numpy as np
import cv2
import rospy

# Paths #

level = 0
i = 0

for letter in sys.argv[1]:

	i+=1

	if letter == "/":

		level+=1;

	if level == 3:

		MODEL_PATH = "../" + sys.argv[1][i:] + "/model"
		INIT_PATH = "../" + sys.argv[1][i:] + "/data/init.png"
		SAVE_PATH = "../" + sys.argv[1][i:] + "/output/network"
		break

# Topic names #

INPUT_TOPIC = "/arav/EPM/AIFilterInput"

OUTPUT_TOPIC = "/arav/EPM/AIFilterOutput"

STAT_TOPIC = "/arav/EPM/AIFilterStatus"

VISUAL_TOPIC = "/arav/visual/detection"

# Script modes #

if sys.argv[2] == "true":

	display = True

else:

	display = False

if sys.argv[3] == "true":

	save = True

else:

	save = False

# Constants #

THRESHOLD = 0.5			   # This line can be modified #
FINAL_FRAME = 3		       # This line can be modified #
UPDATE_RATE = float (sys.argv[4]) 	# Hz #

# Image Size --> HD 720p #

WIDTH = 1280		       # This line can be modified #
HEIGHT = 720		       # This line can be modified #

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

			print ("[AI-Filter] Error while loading frame")

# Listener declaration #

listener = Listener ()

# List to store the bounding boxes #

BoundingBoxes = []

# Initializing ROS node #

rospy.init_node('detection_model')

# ROS Publishers & Subscribers #

sub = rospy.Subscriber(INPUT_TOPIC, Image, listener.imageCallback, queue_size=10)		# Subscriber #

pub = rospy.Publisher(OUTPUT_TOPIC, Int16MultiArray, queue_size=10)				        # Publisher #
output_msg = Int16MultiArray ()

pub_stat = rospy.Publisher(STAT_TOPIC, Int8, queue_size=10)				                # Publisher #
status_msg = Int8 ()

pub_bridge = cv_bridge.CvBridge()
pub_visual = rospy.Publisher(VISUAL_TOPIC, Image, queue_size=10)

rate = rospy.Rate(UPDATE_RATE)

# Publish status --> init #

status_msg.data = 0
pub_stat.publish(status_msg)

# Load detection model #

model = tf.saved_model.load(MODEL_PATH)

print ("[AI-Filter] Inference model has been successfully loaded")

# Publish status --> model loaded #

status_msg.data = 1
pub_stat.publish(status_msg)

init_time = time.time()
currentFrame = 1

# Main loop #

while not rospy.is_shutdown():

	start_time = time.time()

	# ------------------------------------------------------- #

	BoundingBoxes.clear()

	# Load Frame #

	frame = listener.image

	# Process Frame #

	tensor = tf.convert_to_tensor(frame)
	tensor = tensor[tf.newaxis, ...]

	results = model(tensor)

	# Transform Output data #

	nbCorrectResults = np.argwhere(results['detection_scores'].numpy()[0] > THRESHOLD).shape[0]

	for i in range (1):	# Change by nbCorrectResults if more than one #

		BoundingBoxes.append(int(results['detection_boxes'][0][i][0]*HEIGHT))	# Bounding Box Y min #
		BoundingBoxes.append(int(results['detection_boxes'][0][i][1]*WIDTH))		# Bounding Box X min #
		BoundingBoxes.append(int(results['detection_boxes'][0][i][2]*HEIGHT))	# Bounding Box Y max #
		BoundingBoxes.append(int(results['detection_boxes'][0][i][3]*WIDTH))		# Bounding Box X max #

	output_msg.data = BoundingBoxes

	# Publish data #

	pub.publish(output_msg)

	# Stop the node #

	if (currentFrame >= FINAL_FRAME):

		print ("[AI-Filter] Final Frame Processed >> Killing node")

        # Publish status --> Detection finished #

		status_msg.data = 2
		pub_stat.publish(status_msg)

		break

	else:

		currentFrame += 1
		
	# Publish image #
	
	cv2.rectangle (frame, (BoundingBoxes[1+4*i]-5,BoundingBoxes[2+4*i]+5), (BoundingBoxes[3+4*i]+5,BoundingBoxes[0+4*i]-5), (77, 163, 232), 3)
	visual_msg = pub_bridge.cv2_to_imgmsg (frame, "bgr8")
	pub_visual.publish(visual_msg)

	# Display Output #

	if display:

		for i in range (1):	# Change by nbCorrectResults if more than one #

			cv2.rectangle (frame, (BoundingBoxes[1+4*i]-5,BoundingBoxes[2+4*i]+5), (BoundingBoxes[3+4*i]+5,BoundingBoxes[0+4*i]-5), (77, 163, 232), 3)

		cv2.imshow("AI-Filter Detection", frame)

		cv2.waitKey(15)

	# Save Output #

	if save:

		if display:

			cv2.imwrite(SAVE_PATH + "/output_" + str(time.time()-init_time)[:4] + "_segs.png", frame)

		else:

			cv2.rectangle (frame, (BoundingBoxes[1+4*i]-5,BoundingBoxes[2+4*i]+5), (BoundingBoxes[3+4*i]+5,BoundingBoxes[0+4*i]-5), (77, 163, 232), 3)

			cv2.imwrite(SAVE_PATH + "/output_" + str(time.time()-init_time)[:4] + "_segs.png", frame)

	# ------------------------------------------------------- #

	end_time = time.time()

	elapsed_time = end_time - start_time

	print ("[AI-Filter] Frame Processed >> Time = {} segs".format(elapsed_time))

	# Wait until next iteration #

	rate.sleep ()

# Close all OpenCV windows #

cv2.destroyAllWindows()

""" ----------------------------------------------------------------------------
    ---------------- Artificial Intelligence Filter (AI-Filter) ----------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """
