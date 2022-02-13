#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

from math import pi, sin, cos, radians, atan
import numpy as np
import getch

data = {
    'training': [],
    'label': []
}


def sonar_callback(msg):

    data['training'].append(msg.range)

    msgTwist = Twist()

    keyb = getch.getch()

    if keyb == 'd':
        linear = 0.5
        angular = -pi/4
    elif keyb == 'a':
        linear = 0.5
        angular = pi/4
    elif keyb == 'w':
        linear = 0.5
        angular = 0.0
    elif keyb == 's':
        linear = 0.0
        angular = 0.0
    else:
        linear = 0.0
        angular = 0.0

    data['label'].append([angular, linear])
    msgTwist.linear.x = linear
    msgTwist.angular.z = angular

    PUBLISHER.publish(msgTwist)

def save_data():
    with open('training_data', 'wb') as f:
        np.savez(f, np.array(data['training']))

    with open('label_data', 'wb') as f:
        np.savez(f, np.array(data['label']))

    print('SHUTDOWNNNNNNNNNNNNNNNNNNNNNNNNNNNNN')

# def clock_callback():
#     print(data)

def main():
    global PUBLISHER

    rospy.init_node(f'controlp3dx3')
    PUBLISHER = rospy.Publisher('/p3dx3/cmd_vel', Twist, queue_size=10)

    for i in range(0,8):
        sonar_topic = '/p3dx3/sonar' + str(i)
        rospy.Subscriber(sonar_topic, Range, sonar_callback)

    # rospy.Subscriber('p3dx3/cmd_vel', Twist, vel_callback)

    # rospy.Subscriber('clock', Twist, clock_callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        keyb = getch.getch()
        if keyb == 'z':
            rospy.signal_shutdown('shutdown')
            rospy.on_shutdown(save_data)
        rate.sleep()





if __name__ == '__main__':
    main()