#!/usr/bin/env python3

""" ---------------------------------------------------------------------------- 
    ------------------------- Waypoint Publisher - ARAV ------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez -------------------- 
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- 
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------  
    ---------------------------------------------------------------------------- """
    
# Import required libraries #

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

# Topic names #

waypointTopic = "/arav/path_planning/output/path_ground"
groundActivatorTopic = "/arav/path_selector/ground_activation"

# WayPoints #

waypointList = [

[0.25,0.00],[0.50,0.00],[0.75,0.10],[1.00,0.20],[1.25,0.30],[1.50,0.40],[1.75,0.50],[2.00,0.75],
[2.25,1.00],[2.50,1.25],[2.75,1.50],[3.00,1.75],[3.25,2.00],[3.50,2.10],[3.75,2.20],[4.00,2.30],
[4.25,2.40],[4.50,2.50],[4.75,2.50],[5.00,2.50],[5.25,2.50],[5.50,2.50],[5.75,2.50],[6.00,2.40],
[6.25,2.30],[6.50,2.20],[6.75,2.10],[7.00,2.00],[7.25,1.75],[7.50,1.50],[7.75,1.25],[8.00,1.00],
[8.25,0.75],[8.50,0.50],[8.75,0.40],[9.00,0.30],[9.25,0.20],[9.50,0.10],[9.75,0.00],[9.99,0.00],

]    
    
# Init node #

rospy.init_node("WaypointPublisher")

# Loop rate (Hz) #

updateRate = 2
rate = rospy.Rate(updateRate)

# Publishers #

groundActivatorPub = rospy.Publisher (groundActivatorTopic, Bool, queue_size=1)

waypointPub = rospy.Publisher (waypointTopic, Float64MultiArray, queue_size=50)
outputMsg = Float64MultiArray ()

# Variables #

iterator = -1
waypoint = [0.0,0.0]

# Node Main Looop #

activateGround = Bool ()
activateGround.data = True
groundActivatorPub.publish(activateGround)

while not rospy.is_shutdown() and iterator<len(waypointList):

	waypoint[0] = waypointList[iterator][0]
	waypoint[1] = waypointList[iterator][1]
	
	# Publish message #
	
	outputMsg.data = waypoint
	 
	waypointPub.publish(outputMsg)

	rate.sleep ()
	
	iterator += 1

""" ---------------------------------------------------------------------------- 
    ------------------------- Waypoint Publisher - ARAV ------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez -------------------- 
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- 
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------  
    ---------------------------------------------------------------------------- """
    
