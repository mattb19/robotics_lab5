#!/usr/bin/env python3
#import all necessary modules
import rospy
import math
import numpy as np
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

XYZarray2 = XYZarray()

#function to get points
def get_xyz(XYZarray):
	global XYZarray2
	XYZarray2 = XYZarray
	

def process_data(XYZarray2):
	B = []
	A = []
	# Get the list of points from the XYZarray object
	point_list = XYZarray2.points
	
	# Fill A and B with all points, then solve for P
	for point in point_list:
		A.append([2*point.x, 2*point.y, 2*point.z, 1])
		B.append([point.x**2 + point.y**2 + point.z**2])
	P = np.linalg.lstsq(A, B, rcond = None)[0]
	
	return P


if __name__ == '__main__':
	# Define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# Define a subscriber to get xyz of cropped ball
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_xyz)
	# Define a publisher to publish the position and radius through SphereParams
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)

	# Set the loop frequency
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():	
		# Exception handling to avoid empty/null data
		if len(XYZarray2.points) == 0:
			continue
		
		# Exception handling to ensure data being collected is accurate
		try:
			P = process_data(XYZarray2)
		except:
			print("Invalid Data")
			continue
		
		# Collect P and determine xc, yc and zc from it
		P = np.array(P)
		xc = P[0]
		yc = P[1]
		zc = P[2]
		# Uses np array P to calculate the radius of the ball
		radius = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
		
		
		# Display coordinates
		print(xc, yc, zc, radius)
		
		# Set up variable to publish, publish the data
		sphere_params = SphereParams()
		sphere_params.xc = xc
		sphere_params.yc = yc
		sphere_params.zc = zc
		sphere_params.radius = radius
		sphere_pub.publish(sphere_params)
		
		# Pause until the next iteration			
		rate.sleep()

