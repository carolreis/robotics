#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi, sin, cos, radians
import random

def clbk_laser(msg):

    range1 = msg.ranges[290:363]
    range2 = msg.ranges[364:438]
    
    minimo = min(msg.ranges[290:438])

    msg = Twist()
    msg.linear.x = 0.0

    if minimo < 1:
        subst_inf = 3

        range1 = [x if x != float('inf') else subst_inf for x in range1]
        range2 = [x if x != float('inf') else subst_inf for x in range2]

        soma1 = sum(range1)
        soma2 = sum(range2)

        print('|a-b|: ', abs(soma1-soma2))

        if (abs(soma1-soma2)) > 15 or minimo > 0.5:
            if soma1 < soma2:
                angularz = 0.4
            else:
                angularz = -0.4
        else:
            # angularz = 1.5 # rad
            angularz = pi

        # Velocidade linear
        msg.linear.x = 0.1

        # Velocidade angular
        msg.angular.z = angularz

    else:
        msg.linear.x = 0.3
        msg.angular.z = 0.0

    pub_.publish(msg)

def main():
    import sys
    global pub_

    rospy.init_node(f'control_robot')
    laser_topic = '/p3dx' + '/laser/scan'
    vel_topic = '/p3dx' + '/cmd_vel'
    pub_ = rospy.Publisher(vel_topic, Twist, queue_size=10)
    rospy.Subscriber(laser_topic, LaserScan, clbk_laser)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
