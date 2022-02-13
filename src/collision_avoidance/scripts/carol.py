#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi, sin, cos, radians, atan
import random

def clbk_laser(msg):

    vectors = msg.ranges
    somax = 0
    somay = 0

    # angle = msg.angle_min + msg.ranges[0] * msg.angle_increment

    vectors = [x if x < 1 else 1 for x in vectors]

    for i in range(0,727):
        somax += vectors[i] * cos ((i-363) * 0.0057) # até 2 rad
        somay += vectors[i] * sin ((i-363) * 0.0057) # até 2 rad

    msg = Twist()

    print('somax: ', somax)
    print('somay: ', somay)

    # se o objeto estiver bem na frente, vai dar um x pequeno
    if abs(somax) > 0.1:
        angulo = atan(somay/somax) # qual o min e max desse angulo?
        print('angulo: ', angulo)
    else:
        angulo = pi/2


    # print('angulo: ', angulo)

    # Velocidade angular
    # msg.angular.z = angulo

    # Testar por tempo

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
