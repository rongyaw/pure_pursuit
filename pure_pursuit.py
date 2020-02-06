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

LOOKAHEAD_DISTANCE = 0.75 # meters
VELOCITY = 0.5 # m/s


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
path_points_x = [float(point[0]) for point in path_points]
path_points_y = [float(point[1]) for point in path_points]
path_points_w = [float(point[2]) for point in path_points]

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
    x = data.pose.position.x
    y = data.pose.position.y
    qx=data.pose.orientation.x
    qy=data.pose.orientation.y
    qz=data.pose.orientation.z
    qw=data.pose.orientation.w

    quaternion = (qx,qy,qz,qw)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2] 

    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
        # Find the distance between current position to tracking point
    dist_array = np.zeros(len(path_points_x))

    for i in range(len(path_points_x)):
        dist_array[i] = dist((path_points_x[i], path_points_y[i]), (x,y))
    
    goal = np.argmin(dist_array) # Assume the closet point as the goal point at first
    goal_array = np.where((dist_array < (LOOKAHEAD_DISTANCE + 0.3)) & (dist_array > (LOOKAHEAD_DISTANCE - 0.3)))[0]
    for id in goal_array:
        v1 = [path_points_x[id] - x, path_points_y[id] - y]
        v2 = [math.cos(yaw), math.sin(yaw)]
        diff_angle = find_angle(v1,v2)
        if abs(diff_angle) < np.pi/4: # Check if the one that is the cloest to the lookahead direction
            goal = id
            break

    L = dist_array[goal]

    # 3. Transform the goal point to vehicle coordinates. 

    glob_x = path_points_x[goal] - x 
    glob_y = path_points_y[goal] - y 
    goal_x_veh_coord = glob_x*np.cos(yaw) + glob_y*np.sin(yaw)
    goal_y_veh_coord = glob_y*np.cos(yaw) - glob_x*np.sin(yaw)
    

    # 4. Calculate the curvature = 1/r = 2x/l^2
    diff_angle = path_points_w[goal] - yaw # Find the turning angle
    r = L/(2*math.sin(diff_angle)) # Calculate the turning radius
    angle = 2 * math.atan(0.4/r) # Find the wheel turning radius

    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)

def find_angle(v1, v2):
        cosang = np.dot(v1, v2)
        sinang = LA.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.spin()

