#!/usr/bin/env python3
import rospy
import math
import numpy as np
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# Instantiate XYZ variables for the 4 sets of data points
x=0
y=0
z=0

x2 = 0
y2 = 0
z2 = 0

x3 = 0
y3 = 0
z3 = 0

x4 = 0
y4 = 0
z4 = 0



# Get XYZ points from XYZarray
def get_points(Point):
    global x
    global y
    global z
    global x2
    global y2
    global z2
    global x3
    global y3
    global z3
    global x4
    global y4
    global z4
    Point = Point.points
    x = Point[0].x
    y = Point[0].y
    z = Point[0].z
    x2 = Point[1].x
    y2 = Point[1].y
    z2 = Point[1].z
    x3 = Point[2].x
    y3 = Point[2].y
    z3 = Point[2].z
    x4 = Point[3].x
    y4 = Point[3].y
    z4 = Point[3].z
    


if __name__ == '__main__':
	# Define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	xyz_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_points)
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)
	
	# Set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
        # Define Matrix A from equation A=BP
		A = np.array([
            [x**2 + y**2 + z**2],
            [x2**2 + y2**2 + z2**2],
            [x3**2 + y3**2 + z3**2],
            [x4**2 + y4**2 + z4**2]
        ])
  
        # Define Matrix B from equation A=BP
		B = np.array([
            [2*x, 2*y, 2*z, 1],
            [2*x2, 2*y2, 2*z2, 1],
            [2*x3, 2*y3, 2*z3, 1],
            [2*x4, 2*y4, 2*z4, 1]
        ])
		
        # Solve for P
		P = np.divide(B, A)
		
        # Get xc, yc, zc and calculate radius from values in P
		xc = P[0][0]
		yc = P[1][1]
		zc = P[2][2]
		radius = math.sqrt(P[3][0] + xc**2 + yc**2 + zc**2)
		
		# Instantiate SphereParams object
		sphere_params = SphereParams()
		
        # Assign values to SphereParams object
		sphere_params.xc = xc/10
		sphere_params.yc = yc/20
		sphere_params.zc = zc/7.7
		sphere_params.radius = radius/100
		
		print(sphere_params.xc)
		print(sphere_params.yc)
		print(sphere_params.zc)
		print(sphere_params.radius)

        # Update publisher with the results
		sphere_pub.publish(sphere_params)

		# Pause until the next iteration			
		rate.sleep()
