#!/usr/bin/env python3

""" ---------------------------------------------------------------------------- 
    --------------------- Path Selector Module - ARAV --------------------------
    ----------------------------------------------------------------------------
    ------------------------ Author : Joan Bessa Sanz -------------------------- 
    --------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ----------- 
    ------------- (c) Copyright 2022 Joan Bessa All Rights Reserved ------------  
    ---------------------------------------------------------------------------- """
    
# Import required libraries #

import rospy
import sys
from std_msgs.msg import Float32
from std_msgs.msg import Bool

# Script configuration #

if (len(sys.argv) == 2):

	ground_factor = float(sys.argv[1])
	aerial_factor = float(sys.argv[2])

else:	

	# Default Values #
	ground_factor = 0.8
	aerial_factor = 2.0
	
# Topic names #
		
groundLengthTopic = "/arav/path_planning/output/path_length_ground"
aerialLengthTopic = "/arav/path_planning/output/path_length_aerial"
groundActivatorTopic = "/arav/path_selector/ground_activation"
aerialActivatorTopic = "/arav/path_selector/aerial_activation"

# Listener class --------------------------------------------------------------- #

class Listener:

	# Constructor method #

	def __init__ (self):		

		self.receivedGround = False
		self.receivedAerial = False
		self.activateGround = Bool ()
		self.activateGround.data = False
		self.activateAerial = Bool ()
		self.activateAerial.data = False
		self.groundCost = -1.0
		self.aerialCost = -1.0

		self.groundActivatorPub = rospy.Publisher (groundActivatorTopic, Bool, queue_size=1)
		self.aerialActivatorPub = rospy.Publisher (aerialActivatorTopic, Bool, queue_size=1)

	# Class methods #
	
	def groundCallback (self, msg):
	
		# To be executed when ground path length is received #
		self.receivedGround = True

		if msg.data != -1:
			self.groundCost = msg.data*ground_factor

		if self.receivedAerial:
			self.publishSelection()


	def aerialCallback (self, msg):
		# To be executed when aerial path length is received #
		self.receivedAerial = True

		if msg.data != -1:
			self.aerialCost = msg.data*aerial_factor

		if self.receivedGround:
			self.publishSelection()

	def publishSelection (self):
	
		# To be executed when the path cost is determined #
		
		if (self.groundCost != -1) and (self.aerialCost != -1):
			
			if self.groundCost <= self.aerialCost:
				self.activateGround.data = True
			else:
				self.activateAerial.data = True
		elif (self.aerialCost != -1):
			self.activateAerial.data = True
		else:
			self.activateGround.data = True
	
		print("Ground Path Activation = " + str(self.activateGround))
		print("Aerial Path Activation = " + str(self.activateAerial))
		self.groundActivatorPub.publish(self.activateGround)
		self.aerialActivatorPub.publish(self.activateAerial)
		
listener = Listener ()
	
# Main function ----------------------------------------------------------------- #

# Parameters #

# Init node #

rospy.init_node("path_selector")

# ROS Publishers & Subscribers #

groundLengthSub = rospy.Subscriber (groundLengthTopic, Float32, listener.groundCallback, queue_size=1)
aerialLengthSub = rospy.Subscriber (aerialLengthTopic, Float32, listener.aerialCallback, queue_size=1)

rospy.spin()

""" ---------------------------------------------------------------------------- 
    --------------------- Path Selector Module - ARAV --------------------------
    ----------------------------------------------------------------------------
    ------------------------ Author : Joan Bessa Sanz -------------------------- 
    --------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ----------- 
    ------------- (c) Copyright 2022 Joan Bessa All Rights Reserved ------------  
    ---------------------------------------------------------------------------- """
