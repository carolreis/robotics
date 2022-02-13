#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi, sin, cos, radians, atan
import numpy as np


def laser_callback(msg):

    vectors = msg.ranges
    angle_increment = msg.angle_increment # 0.0057
    msg = Twist()

    print('\n')
    vectors = [x if x < 2 else 2 for x in vectors]

    ''' Obtém o meio da circunferência '''
    middle = int(len(vectors)/2) # 363

    # Y dos vetores
    right = [
        vectors[152] * sin((152-middle) * 0.0057),
        vectors[90] * sin((90-middle) * 0.0057)
    ]
    # X dos vetores
    right_cos = [
        vectors[152] * cos((152-middle) * 0.0057),
        vectors[90] * cos((90-middle) * 0.0057)
    ]

    # X dos vetores
    front = [
        vectors[394] * cos((394-middle) * 0.0057),
        vectors[363] * cos((363-middle) * 0.0057),
        vectors[333] * cos((333-middle) * 0.0057)
    ]

    # Y dos vetores
    left = [
        vectors[576] * sin((576-middle) * 0.0057),
        vectors[636] * sin((636-middle) * 0.0057)
    ]
    # X dos vetores
    left_cos = [
        vectors[576] * cos((576-middle) * 0.0057),
        vectors[636] * cos((636-middle) * 0.0057)
    ]

    angulo = 0.0
    msg.linear.x = 0.5
    msg.angular.z = 0.0

    # FUGIR DA PAREDE
    # Se tem obstáculo na frente
    if min(front) < 0.5:

        # Para o robô
        msg.linear.x = 0.0

        # Soma as laterais
        soma = right[0] + right[1] + left[0] + left[1]

        # Vazio dos lados
        if abs(soma) < 0.5:
            angulo = pi/2

    # PERSEGUIR PAREDE
    # Se não tem obstáculo na frente e tem parede na direita
    if abs((max(right))) < 1:

        x_right_top = right_cos[0]
        y_right_top = right[0] 

        x_right_bottom = right_cos[1]
        y_right_bottom = right[1] 

        # Identificar se tem um ângulo aberto
        diferenca_y = abs(y_right_top - y_right_bottom)

        if min(front) > 1:
            # Não tem nada na frente

            if diferenca_y > 0.5:
                # Não tem obstáculo, tem um angulo aberto, vira

                novo_x = x_right_top - x_right_bottom 
                novo_y = y_right_top - y_right_bottom
                angulo = atan(novo_y/novo_x)

            else:

                # Não tem obstáculo, segue a parede        
                media = (right[0] + right[1]) / 2

                cy = (media + 0.5)

                cx = 0.8
                angulo = atan(cy/cx)

        else:
            # Tem obstáculo na frente, angulo fechado, vira

            msg.angular.z = 0.0
            novo_x = 1 * (x_right_top + x_right_bottom)
            novo_y = (-1) * (y_right_top + y_right_bottom)
            angulo = atan(novo_y/novo_x)

    # Se não tem obstáculo na frente e tem parede na esquerda
    elif abs(min(left)) < 1:

        x_left_top = left_cos[0]
        y_left_top = left[0] 

        x_left_bottom = left_cos[1]
        y_left_bottom = left[1] 

        # Identificar se tem um ângulo aberto
        diferenca_y = abs(y_left_top - y_left_bottom)

        if min(front) > 1:

            if diferenca_y > 0.5:
                novo_x = x_left_top - x_left_bottom 
                novo_y = y_left_top - y_left_bottom
                angulo = atan(novo_y/novo_x)
            else:
                media = (left[0] + left[1]) / 2
                cy = (media - 0.5)
                cx = 0.8
                angulo = atan(cy/cx)

        else:
            # Tem obstáculo na frente, angulo fechado, vira
            msg.angular.z = 0.0
            novo_x = 1 * (x_left_top + x_left_bottom)
            novo_y = (-1) * (y_left_top + y_left_bottom)
            angulo = atan(novo_y/novo_x)

    else:
        msg.linear.x = 0.5

    print('angulo: ', angulo)

    msg.angular.z = angulo


    PUBLISHER.publish(msg)

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