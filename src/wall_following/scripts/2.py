#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi, sin, cos, radians, atan
import random

def laser_callback(msg):
    print('\n')

    vectors = msg.ranges

    vel = Twist()

    vectors = [x if x < 1 else 1 for x in vectors]

    front_min_vetor = min(vectors[192:534])
    print('front_min_vector: ', front_min_vetor)

    left = vectors[534:727]
    right = vectors[0:192]
    min_left = min(left)
    min_right = min(right)

    ## dividir em mais setores 
    right_bottom_vector = vectors[0:137] # [0:136]
    right_top_vector = vectors[137:275] # [0:274]
    front_vector = vectors[275:459] # [275:458]
    left_top_vector = vectors[459:597] # [459:596]
    left_bottom_vector = vectors[597:727] # [597:726]

    diferenca_left_right = min_right - min_left
    print('\nmin_right: ', min_right)
    print('min_left: ', min_left)

    print('\nright bottom: ', min(right_bottom_vector))
    print('right top: ', min(right_top_vector))
    print('front: ', min(front_vector))
    print('left top: ', min(left_top_vector))
    print('left bottom: ', min(left_bottom_vector))

    print('\n|diferenca_left_right|: ', abs(diferenca_left_right))

    if min(front_vector) < 0.5 and abs(diferenca_left_right) < 0.1:
        ''' Se estiver bem de frente com a parede com lado em equilíbrio'''

        print('\n obstaculo na frente, lados livres')
        angulo = pi/3  # vira para qualquer lado

        vel.linear.x = 0.0
        vel.angular.z = angulo
        print('angulo: ', angulo)

    elif min(front_vector) < 0.5 and abs(diferenca_left_right) >= 0.1:
        ''' Se estiver bem de frente com a parede, mas um dos lados mais livre que o outro '''
        if min_left > min_right:
            angulo = pi/2
        else:
            angulo = (-1) * pi/2

        print('\n obstaculo na frente, um lado mais livre')
        vel.linear.x = 0.0
        vel.angular.z = angulo

        print('angulo: ', angulo)

    elif min(right_bottom_vector) < 0.5 and min(right_top_vector) == 1:
        ''' Tem que tirar a direita'''
        print('tem que virar a direita')
        angulo = -pi/2
        print('angulo: ', angulo)
        vel.angular.z = angulo
        vel.linear.x = 0.2
    
    elif min(left_bottom_vector) < 0.5 and min(left_top_vector) == 1:
        ''' Tem que tirar a esquerda'''
        print('tem que virar a esquerda')
        angulo = pi/2
        print('angulo: ', angulo)
        vel.angular.z = angulo
        vel.linear.x = 0.2
    

    elif front_min_vetor >= 0.85:
        print('frente vazia')

        if diferenca_left_right == 0:
            ''' Não tem nada na volta. Segue em frente '''
            angulo = 0.0
        else:
            ''' Frente está vazia. Um dos lados tem parede '''
            if min_right < min_left:
                angulo = (-1)*pi/2
            else:
                angulo = pi/2

        vel.angular.z = angulo
        vel.linear.x = 0.8

    else:
        ''' Não tem obstáculos. Segue reto '''
        vel.linear.x = 0.5
        vel.angular.z = 0.0

    PUBLISHER.publish(vel)

def main():
    global PUBLISHER

    rospy.init_node(f'control_robot')

    laser_scan_topic = '/p3dx/laser/scan'
    cmd_vel_topic = '/p3dx/cmd_vel'

    PUBLISHER = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    rospy.Subscriber(laser_scan_topic, LaserScan, laser_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()