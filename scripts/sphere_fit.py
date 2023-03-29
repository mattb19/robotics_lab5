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
	
	

if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# define a subscriber to get xyz of cropped ball
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_xyz)
	# define a publisher to publish position
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)

	# set the loop frequency
	rate = rospy.Rate(10)
	
	x = 0
	
	
	while not rospy.is_shutdown():
		#test to see values
		
		if len(XYZarray2.points) == 0:
			continue
		
		
		if XYZarray2.points[x].x > 1:
			x += 1
			continue
			
		x += 1
		
		
		print("-------------------------------------------------------------------------------------------------")
		#print(XYZarray2.points[x].x)
		#print(XYZarray2.points[x].y)
		#print(XYZarray2.points[x].z)
		#print(list(str(XYZarray2.points))[x])
		
		
		
		B = np.array([XYZarray2.points[x].x**2 + XYZarray2.points[x].y**2 + XYZarray2.points[x].z**2])
		A = [[2*XYZarray2.points[x].x, 2*XYZarray2.points[x].y, 2*XYZarray2.points[x].z, 1]]
		
		P = np.linalg.lstsq(A, B, rcond = None)[0]
		
		xc = P[0]
		yc = P[1]
		zc = P[2]
		radius = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
		
		print(xc, yc, zc, radius)
		
		
		
		
		
		
		
		
		
		# Calculates the center of the ball
		#B = [[Point.x**2 + Point.y**2 + Point.z**2],
		     #[Point.x**2 + Point.y**2 + Point.z**2],
		     #[Point.x**2 + Point.y**2 + Point.z**2],
		     #[Point.x**2 + Point.y**2 + Point.z**2]]
		    
		#A = [[2*Point.x, 2*Point.y, 2*Point.z, 1],
		     #[2*Point.x, 2*Point.y, 2*Point.z, 1],
		     #[2*Point.x, 2*Point.y, 2*Point.z, 1],
		     #[2*Point.x, 2*Point.y, 2*Point.z, 1]]
		#Uses formula to calculate P
		#P = np.true_divide(B, A)
		
		
		#xc = P[0]
		#yc = P[1]
		#zc = P[2]
		#Uses coords to calculate the radius of the ball
		#radius = math.sqrt(P[3][0] + xc**2 + yc**2 + zc**2)
		
		#set up variable to publish
		sphere_params = SphereParams()
		sphere_params.xc = xc
		sphere_params.yc = yc
		sphere_params.zc = zc
		sphere_params.radius = radius
		sphere_pub.publish(sphere_params)
		# pause until the next iteration			
		rate.sleep()

