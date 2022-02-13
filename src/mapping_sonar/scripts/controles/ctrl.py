#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import pi

def vel_callback(ms):
    print('MESSAGE : ', ms)
    msg = Twist()
    msg.linear.x = msg.linear.x
    msg.linear.z = msg.angular.z
    PUBLISHER.publish(msg)

def main():
    global PUBLISHER

    rospy.init_node(f'control_robot')


    odom_topic = '/my_vel/'
    rospy.Subscriber(odom_topic, Twist, vel_callback)

    cmd_vel_topic = '/p3dx3/cmd_vel'

    PUBLISHER = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
