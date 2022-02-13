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

    # raio 2
    vectors = [x if x < 2 else 2 for x in vectors]

    right_bottom_vector = vectors[0:137] # [0:136]
    right_top_vector = vectors[137:275] # [0:274]
    front_vector = vectors[275:459] # [275:458]
    left_top_vector = vectors[459:597] # [459:596]
    left_bottom_vector = vectors[597:727] # [597:726]
  
    intervalos = [(0,137),(137,275),(275,459),(459,597),(597,727)]

    somas_x = []
    somas_y = []
    degs = []

    medias = []

    middle = 363

    for intervalo in intervalos:
        somax = 0
        somay = 0

        # calcula a média dos vetores
        medias.append(sum(vectors[intervalo[0]:intervalo[1]])/len(range(intervalo[0],intervalo[1])))

        # Calcula a soma vetorial no intervalo corrente
        for i in (range(intervalo[0], intervalo[1])):
            somax += vectors[i] * cos((i-middle) * 0.0057)
            somay += vectors[i] * sin((i-middle) * 0.0057)

        somas_x.append(somax)
        somas_y.append(somay)
        degs.append(atan(somay/somax))

    # Calcula a soma vetorial de todos os setores
    somax=0
    somay=0
    for i in range(0,len(vectors)):
        somax += vectors[i] * cos((i-middle) * 0.0057)
        somay += vectors[i] * sin((i-middle) * 0.0057)
    deg = atan(somay/somax)

    print('\nmédias:')
    print(medias)

    global_vec = [somax, somay, deg]
    print('\nglobal vec:')
    print(global_vec)

    # print('\nsetores:')
    # print(somas_x)
    # print(somas_y)
    # print(degs)

    right_bottom = (somas_x[0], somas_y[0], degs[0])
    right_top = (somas_x[1], somas_y[1], degs[1])
    front = (somas_x[2], somas_y[2], degs[2])
    left_top = (somas_x[3], somas_y[3], degs[3])
    left_bottom = (somas_x[4], somas_y[4], degs[4])

    print('\n')
    print('direita baixo: ', right_bottom)
    print('direita cima: ', right_top)
    print('frente: ', front)
    print('esquerda cima: ',left_top)
    print('esquerda baixo: ',left_bottom)

    print('\nsoma dos angulos:')
    angulo_soma = sum(degs)
    print(angulo_soma)

    # Usar a diferença entre os y para saber se está no meio de um corredor ou não
    max_direita = max(right_bottom[1],right_top[1])
    max_esquerda = max(left_bottom[1],left_top[1])
    print('\nmax direita abs: ', abs(max_direita))
    print('\nmax esquerda abs: ', abs(max_esquerda))
    diferenca = abs(max_direita) - abs(max_esquerda)
    print('dif dir - esq: ', diferenca)

    angulo = 0
      
    # Se for positivo, se mantém à esquerda
    if diferenca > 0 and diferenca  < 150:
        angulo = pi/2
    elif diferenca < 0 and diferenca  > -150:
        angulo = -pi/2
        print('?')
    else:
        angulo = 0 

    vel.linear.x = 0.2
    vel.angular.z = angulo

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
