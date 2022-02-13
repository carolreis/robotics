#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi, sin, cos, radians
import random

def clbk_laser(msg):
    # print(f"LASER size {len(msg.ranges)}")
    # print(f"LASER size {msg.ranges[360]}")
    # print(f"LASER {msg.ranges}")
    # print('all data: ', msg)
    print('\n')
    print('DADOS LASER: ', msg.ranges)

    menor_valor = min(msg.ranges)
    print('MENOR VALOR: ', menor_valor)

    angle = msg.angle_min + msg.ranges.index(menor_valor) * msg.angle_increment
    angle = angle * (180/pi)

    print('ANGULO DO MENOR VALOR: ', angle)

    msg = Twist()
    msg.linear.x = 0.0
    
    # if menor_valor < 0.7:

        # seno = sin(radians(angle))
        # cosseno = cos(radians(angle))
        # print('Seno do ângulo: ', seno)
        # print('Cos do angulo: ', cosseno)
        # print('x,y: ', (menor_valor * cosseno), (menor_valor * seno))

        # Velocidade linear
        # msg.linear.x = 0.05
        # ang = 0.4
        # ang = angle

        # Velocidade angular
        # msg.angular.z = 0.4

    # elif menor_valor < 0.2:
    #     msg.linear.x = 0.0
    #     msg.angular.z = 0.5

    # else:
    #     msg.linear.x = 0.1
    #     msg.angular.z = 0.0

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
