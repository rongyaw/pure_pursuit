#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 2.0 # meters
VELOCITY = 1.0 # m/s


###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


#############
# FUNCTIONS #
#############
    
# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /pf/viz/inferred_pose.
# Runs pure pursuit and publishes velocity and steering angle.
def callback(data):

    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
    # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.    
    P = PoseStamped()
    x = P.pose.position.x
    y = P.pose.position.y
    yaw = P.pose.orientation.z
    #information = str(x) + ' ' + str(y) + str(yaw)
    #rospy.loginfo(information)

    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    x_lookahead = x + LOOKAHEAD_DISTANCE * math.cos(yaw) # Find x coordinate of lookahead point
    y_lookahead = y + LOOKAHEAD_DISTANCE * math.sin(yaw) # Find y coordinate of lookahead point
    point_lookahead = np.array([x_lookahead, y_lookahead])
        
        # Change the path_points to numpy array for easy application
    path_points_refer = np.asarray(path_points)
    path_points_refer = path_points_refer[:,0:2]    

    dist_refer = np.array([])

    for point_interest in path_points_refer:
        dist_refer = np.append(dist_refer, dist(point_lookahead, point_interest))
    
        # find the minimum point
    min_index = np.argmin(dist_refer)
    path_point = path_points(min_index)

    # 3. Transform the goal point to vehicle coordinates. 
    
    

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.spin()

