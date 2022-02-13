#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import pi
import getch

def main():
    global PUBLISHER

    rospy.init_node(f'control_robot')

    cmd_vel_topic = '/p3dx/cmd_vel'

    PUBLISHER = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    msg = Twist()

    while True:
        keyb = getch.getch()

        if keyb == 'd':
            msg.linear.x = 0.5
            msg.angular.z = -pi/4
            PUBLISHER.publish(msg)
        elif keyb == 'a':
            msg.linear.x = 0.5
            msg.angular.z = pi/4
            PUBLISHER.publish(msg)
        elif keyb == 'w':
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            PUBLISHER.publish(msg)
        elif keyb == 's':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            PUBLISHER.publish(msg)
        elif keyb == 'q':
            break

    PUBLISHER.publish(msg)

if __name__ == '__main__':
    main()
