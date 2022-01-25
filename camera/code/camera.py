#!/usr/bin/env python3

""" ---------------------------------------------------------------------------- 
    ---------------------------- Camera Module - ARAV --------------------------
    ----------------------------------------------------------------------------
    ------------------------ Author : Joan Bessa Sanz -------------------------- 
    --------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ----------- 
    ------------- (c) Copyright 2022 Joan Bessa All Rights Reserved ------------  
    ---------------------------------------------------------------------------- """
    
# Import required libraries #

import rospy
import cv2
import datetime
import keyboard

# Init node #

rospy.init_node("camera")

# Main function #

cam0 = cv2.VideoCapture(0)
cam1 = cv2.VideoCapture(2)

while True:
	ret0, image0 = cam0.read()
	cv2.imshow('Image 0',image0)
	ret1, image1 = cam1.read()
	cv2.imshow('Image 1',image1)
	print("Press p to take a picture, x to stop the node")
	if keyboard.read_key() == "p":
		date = datetime.datetime.today()
		date_string = '{:%m/%d/%y-%H:%M:%S}'.format(date)
		cv2.imwrite('../images/image0_'+date_string+'.jpg', image0)
		cv2.imwrite('../images/image1_'+date_string+'.jpg', image1)
		print("Images stored")
	elif keyboard.read_key() == "x":
		print("Stopping node")
		break

cam0.release()
cam1.release()
cv2.destroyAllWindows()

""" ---------------------------------------------------------------------------- 
    ---------------------------- Camera Module - ARAV --------------------------
    ----------------------------------------------------------------------------
    ------------------------ Author : Joan Bessa Sanz -------------------------- 
    --------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ----------- 
    ------------- (c) Copyright 2022 Joan Bessa All Rights Reserved ------------  
    ---------------------------------------------------------------------------- """
