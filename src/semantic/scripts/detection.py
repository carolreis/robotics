#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, atan, pi, acos, sqrt, atan2

# from math import sin, cos, atan, pi, acos, sqrt, atan2
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import numpy as np
# import json


THETA = None
D = None
X = None
y = None

def get_object_centroid(theta, d, x, y):
    try:
        b = cos(theta) * d
        a = sin(theta) * d
        centroid = (b + x, a + y)
    except BaseException as e:
        print("Exception: %s " % e)

def darknet_callback(msg):
    # print('\nmsg x_min: %s ' % msg.bounding_boxes[0].xmin)
    # print('msg y_min: %s ' % msg.bounding_boxes[0].ymin)
    # print('msg x_max: %s ' % msg.bounding_boxes[0].xmax)
    # print('msg y_max: %s ' % msg.bounding_boxes[0].ymax)
    # print('msg id: %s ' % msg.bounding_boxes[0].id)
    # print('msg class: %s ' % msg.bounding_boxes[0].Class)
    pass

def odom_callback(msg):

    global THETA, X, Y

    # Posição do robô
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y

    # Pose do robô
    THETA_q = msg.pose.pose.orientation
    THETA_list = [THETA_q.x, THETA_q.y, THETA_q.z, THETA_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (THETA_list)
    print("\nyaw: %s " % yaw)

    THETA = yaw

def main():

    rospy.init_node('detection')

    darknet_topic = '/darknet_ros/bounding_boxes'
    rospy.Subscriber(darknet_topic, BoundingBoxes, darknet_callback)

    odom_topic = '/p3dx3_TESTE/odom'
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
