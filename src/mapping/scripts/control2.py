#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import pi
import pygame

def main():
    global PUBLISHER

    rospy.init_node(f'control_robot')

    cmd_vel_topic = '/p3dx/cmd_vel'

    pygame.init()

    PUBLISHER = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    msg = Twist()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        keyboard = pygame.key.get_pressed()
        print('keyboard: ', keyboard)
        if keyboard[pygame.K_UP]:
            msg.linear.x = 0.8
        elif keyboard[pygame.K_DOWN]:
            msg.linear.x = -0.8
        else:
            msg.linear.x = 0.0

        if keyboard[pygame.K_LEFT]:
            msg.angular.z = pi/4
        elif keyboard[pygame.K_RIGHT]:
            msg.angular.z = -pi/4
        else:
            msg.angular.z = 0.0

        PUBLISHER.publish(msg)

if __name__ == '__main__':
    main()
