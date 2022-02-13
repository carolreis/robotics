#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi, sin, cos, radians, atan


def laser_callback(msg):

    vectors = msg.ranges
    angle_increment = msg.angle_increment # 0.0057
    msg = Twist()
    somax = 0
    somay = 0

    ''' Transforma tudo em 1 - para evitar um vetor resultante enorme
        Só a preciso a direção, então, dá para usar vetores unitários.
        Ele só vai "enxergar" obstáculos até raio 1
    '''
    vectors = [x if x < 1 else 1 for x in vectors]


    ''' Obtém o meio da circunferência '''
    middle = int(len(vectors)/2) # 363


    '''
        Soma vetorial para obter o vetor resultante indicando a nova direção do robô 
        Usa o range(192, 534), e não todos os pontos d0 laser (192, 727)... porque... ??
    '''
    for i in range(192,534):
        somax += vectors[i] * cos ((i-middle) * 0.0057)
        somay += vectors[i] * sin ((i-middle) * 0.0057)


    '''
        Se |Ey| ~ 0 =>
            Não tem obstáculos laterais.
            Ou está bem no meio de um corredor, mesmo que o corredor seja "estreito" os vetores vão se anular e -> 0
    '''

    msg.linear.x = 0.5

    min_vetor = min(vectors[192:534])
    ''' 
        Se o menor vetor < 0.4, significa que há um obstáculo na frente | range (192, 534)
        ey -> 0
    '''
    if min_vetor < 0.3:

        somax_e = 0
        somax_d = 0
        somay_e = 0
        somay_d = 0

        '''
            ESQUERDA | range (0,192)
            Faz a soma vetorial com os vetores da esquerda
        '''
        for i in range(0,192):
            somax_e += vectors[i] * cos ((i-363) * 0.0057)
            somay_e += vectors[i] * sin ((i-363) * 0.0057)

        if abs(somax_e) > 0.01:
            ''' Obtém a direção indicada pelos vetores da esquerda '''
            angulo_e = atan(somay_e/somax_e)
            if somax_e < 0:
                angulo_e += pi
        else:
            ''' Evita divisão por 0 '''
            angulo_e = (-1) * pi/2

        '''
            DIREITA | range (534,727)
            Faz a soma vetorial com os vetores da direita
        '''
        for i in range(534,727):
            somax_d += vectors[i] * cos ((i-363) * 0.0057)
            somay_d += vectors[i] * sin ((i-363) * 0.0057)

        if abs(somax_d) > 0.01:
            ''' Obtém a direção indicada pelos vetores da direita '''
            angulo_d = atan(somay_d/somax_d)
            if somax_d < 0:
                angulo_d += pi
        else:
            ''' Evitar divisão por 0 '''
            angulo_d = pi/2

        ''' Verifica espaço livre '''
        diferenca = somay_e - somay_d # Se as laterais forem simétricas

        if abs(diferenca) < 150: # Fica oscilando 
            angulo = 2
        else:
            if diferenca > 0:
                ''' Se houver mais espaço livre à esquerda, vira à esquerda '''
                angulo = angulo_e
            else:
                ''' Se houver mais espaço livre à direita, vira à direita '''
                angulo = angulo_d

        ''' Para o robô para rotacionar '''
        msg.linear.x = 0.0

    elif abs(somax) > 0.01:
        ''' Range do ângulo: (preencher)'''
        angulo = atan(somay/somax)
        if somax < 0:
            angulo += pi

    else:
        ''' Se |Ex| ~ 0  =>  pi/2 - para evitar divisão por zero '''
        angulo = pi/2

    ''' Define o angulo rad / sec '''
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