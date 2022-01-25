#!/usr/bin/env python3

""" ---------------------------------------------------------------------------- 
    --------------------- Path Control Module (PCM) - ARAV ---------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez -------------------- 
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- 
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------  
    ---------------------------------------------------------------------------- """
    
# Import required libraries #

import rospy
import math
import sys
import time
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

# Sleep 10 seconds to load gazebo #

time.sleep(10)

# Script configuration #

if (len(sys.argv) == 8):

	gain = float(sys.argv[1])
	margin = float(sys.argv[2])
	limit = float(sys.argv[3])
	velocity = float(sys.argv[4])
	updateRate = float(sys.argv[5])

else:	

	# Default Values #
	gain = 5.0
	margin = 0.2
	limit = 1000.0
	velocity = 0.2
	updateRate = 5
	
# Topic names #
		
positionTopic = "/arav/gazebo/model_states"
commandTopic = "/arav/control/ground/cmd"
waypointTopic = "/arav/path_planning/output/path_ground"
groundActivatorTopic = "/arav/path_selector/ground_activation"

# WayPoints #

waypointList = []

# Listener class --------------------------------------------------------------- #

class Listener:

	# Constructor method #

	def __init__ (self):		

		self.realValueX = 0.0
		self.realValueY = 0.0
		self.realYaw = 0.0
		self.index = 0
		self.robotName = "arav"
		self.activation = False
		
	# Class methods #
	
	def activationCallback (self, msg):
		
		# To be executed when an activation signal is received #
		
		self.activation = msg.data
	
	def waypointCallback (self, msg):
	
		# To be executed when a waypoint is received #
	
		global waypointList
	
		xValue = msg.data[0]
		yValue = -msg.data[1]
		
		waypointList.append([xValue, yValue])

	def poseCallback (self, msg):
	
		# To be executed when a position is received #
		
		self.index = msg.name.index(self.robotName)
		self.realValueX = msg.pose[self.index].position.x
		self.realValueY = msg.pose[self.index].position.y
		
		orientObj = msg.pose[self.index].orientation
		orientList = [orientObj.x, orientObj.y, orientObj.z, orientObj.w]
		self.realYaw = euler_from_quaternion (orientList)[2] - math.pi
		if self.realYaw < 0:
			self.realYaw += 2 * math.pi
		
listener = Listener ()
		
# Compute Angle Function ------------------------------------------------------- #

def computeAngle (deltaX, deltaY, previous, debug):

	# Internal variables #
	
	deltaXmin = 0.0001
	deltaYmin = 0.0001
	angle = 0.0
	
	# Internal Logics #
	
	if abs(deltaX) < deltaXmin:
	
		if (deltaY > 0 and abs(deltaY) > deltaYmin):
			angle = math.pi/2
			
		elif (deltaY < 0 and abs(deltaY) > deltaYmin):
			angle = 3*math.pi/2
			
		else:
			angle = previous
			
		# Test #
		if (debug == True):
			print ("Desired Angle >> ", angle * (180/math.pi), "deg")
			
		return angle
			
	else:
	
		angle = math.atan(deltaY/deltaX)
		
	if (deltaX < 0):
		angle += math.pi
		
	elif (deltaX > 0 and deltaY < 0):
		angle += 2 * math.pi
		
	# Test #
	if (debug == True):
		print ("Desired Angle >> ", angle * (180/math.pi), "deg")
		
	return angle
	
# ------------------------------------------------------------------------------- #

# Compute Error Function -------------------------------------------------------- #

def computeError (desiredAngle, realAngle, debug):

	# Internal variables #
	
	error = 0.0
	
	# Internal Logics #
	
	error = desiredAngle - realAngle
	
	if (error > math.pi):
	
		error = (2*math.pi - error)*(-1)
		
	elif (error < -math.pi):
	
		error = 2*math.pi + error
		
	# Test #
	if (debug == True):
		print ("Error >> ", error * (180/math.pi), "deg")
		
	return error

# ------------------------------------------------------------------------------- #

# Proportional Controller Function ---------------------------------------------- #

def proportionalController (error, debug):

	# Internal variables #
	
	global gain
	global limit
	out = 0.0
	
	# Internal Logics #
	
	out = gain * error
	
	if out > limit:
		out = limit
		
	if out < -limit:
		out = -limit
	
	# Test #
	if (debug == True):
		print ("Yaw rate >> ", out, "rad/s")
	
	return out
	
# ------------------------------------------------------------------------------- #

# Select Next Waypoint ---------------------------------------------------------- #

def selectNextWP (waypointList, xReal, yReal, debug):

	# Internal variables #
	
	global margin
	deltaX = 0.0
	deltaY = 0.0
	status = True
	
	# Internal Logics #
	
	if (len (waypointList) == 0):
	
		status = False
		
	else:
	
		deltaX = waypointList[0][0] - xReal
		deltaY = waypointList[0][1] - yReal
		
		if ((abs(deltaX) < margin) and (abs(deltaY) < margin)):
		
			waypointList.pop(0)
			
			if (len (waypointList) == 0):
			
				deltaX = 0.0
				deltaY = 0.0
				status = False
			
			else:
			
				deltaX = waypointList[0][0] - xReal
				deltaY = waypointList[0][1] - yReal
			
	# Test #
	if (debug == True):
		if (len (waypointList) == 0):
			print ("Objective WP reached !!")
		else:
			print ("Next WayPoint >> ", waypointList[0][0], waypointList[0][1])
			
	return deltaX, deltaY, status
	
# ------------------------------------------------------------------------------- #
	
# Main function ----------------------------------------------------------------- #

# Parameters #

realValueX = 0.0
realValueY = 0.0
realYaw = 0.0
yawRate = 0.0
deltaX = 0.0
deltaY = 0.0
angle = 0.0
error = 0.0
debug = True
status = False
statusPrev = False
activation = True

# Init node #

rospy.init_node("PathControlModule")

# Loop rate (Hz) #

rate = rospy.Rate(updateRate) 

# ROS Publishers & Subscribers #

cmdPub = rospy.Publisher (commandTopic, Twist, queue_size=1)

poseSub = rospy.Subscriber (positionTopic, ModelStates, listener.poseCallback, queue_size=1)

waypointSub = rospy.Subscriber (waypointTopic, Float64MultiArray, listener.waypointCallback, queue_size=50)

activSub = rospy.Subscriber (groundActivatorTopic, Bool, listener.activationCallback, queue_size=1)

outputMsg = Twist ()

outputMsg.linear.x = 0.0
outputMsg.linear.y = 0.0
outputMsg.linear.z = 0.0
outputMsg.angular.x = 0.0
outputMsg.angular.y = 0.0
outputMsg.angular.z = 0.0
	
# Pre Node Loop #

activation = listener.activation

while not rospy.is_shutdown() and not activation:

	# Update activation #
		
	activation = listener.activation
	
	# Wait until next iteration #	
	
	rate.sleep ()
	
# Node Main Loop #

while not rospy.is_shutdown() and activation:

	# Update values #

	realValueX = listener.realValueX
	realValueY = listener.realValueY
	realYaw = listener.realYaw
	
	# Compute speed #
	
	statusPrev = status
	
	deltaX, deltaY, status = selectNextWP (waypointList, realValueX, realValueY, debug = False)
	
	if (status == False):
	
		outputMsg.linear.x = 0.0
		outputMsg.angular.z = 0.0
		
		if statusPrev:
			break
		
	else:
		
		angle = computeAngle (deltaX, deltaY, angle, debug = False)
		error = computeError (angle, realYaw, debug = False)
		yawRate = proportionalController (error, debug = False)
		
		outputMsg.linear.x = velocity
		outputMsg.angular.z = yawRate
	
	# Publish message #
	 
	cmdPub.publish(outputMsg)
	
	# Update activation #
	
	activation = listener.activation

	# Wait until next iteration #	
	
	rate.sleep ()
	
# Kill node when activation becomes false #

outputMsg.linear.x = 0.0
outputMsg.angular.z = 0.0
cmdPub.publish(outputMsg)
time.sleep(5)

""" ---------------------------------------------------------------------------- 
    --------------------- Path Control Module (PCM) - ARAV ---------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez -------------------- 
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- 
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------  
    ---------------------------------------------------------------------------- """
    
