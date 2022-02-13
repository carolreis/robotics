#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import pygame
from math import atan2, sin, cos, atan, pi, acos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import gc

ORIGINAL_GRID_SIZE = 20
NEW_GRID_SIZE = 81
BLOCKSIZE = 10
REGIONS = NEW_GRID_SIZE / ORIGINAL_GRID_SIZE # 2
WIDTH = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
HEIGHT = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
XY = None
ORIGINAL_XY = None
QUANT_BLOCOS = (WIDTH/BLOCKSIZE)
MAX_RANGE = 3
SONARES = {
    'sonar0': {},
    'sonar1': {},
    'sonar2': {},
    'sonar3': {},
    'sonar4': {},
    'sonar5': {},
    'sonar6': {},
    'sonar7': {}
}
POSITIONS = []
ORIGIN = (WIDTH/2, HEIGHT/2)
angulo_visao = 0.267000/2
angulos = [pi/2, (pi/2/1.8), (pi/2)/3, (pi/2)/9, -(pi/2)/9,  -(pi/2)/3, -(pi/2/1.8), -pi/2] # Em relação ao robô

THETA = None
GOAL = Point()
GOAL.x = 2
GOAL.y = 0

def _newx(x):
    ''' Coordenada gazebo x -> grid x, multiplica pelo tamanho do bloco '''
    _x = ( x + int(ORIGINAL_GRID_SIZE / 2) ) * REGIONS
    _x = _x * BLOCKSIZE
    return int(_x)
    
def _newy(y):
    ''' Coordenada gazebo y -> grid y, multiplica pelo tamanho do bloco '''
    _y = ( y - int(ORIGINAL_GRID_SIZE / 2) ) * (-1) * REGIONS
    _y = _y * BLOCKSIZE
    return int(_y)

def _transform_coord(x,y):
    ''' Inverte os eixos '''
    XY = (x, y)
    XY = (y, HEIGHT - x)
    return XY

def _polar_xy(raio, angulo):
    ''' Gazebo (r,angulo) -> (x,y) grid ''' 
    x = _newx(raio*cos(angulo))
    y = _newy(raio*sin(angulo))
    return _transform_coord(x,y)

def _xy_polar(x,y,r=None):
    try:
        if not r:
            r = sqrt(x**2 + y**2)
        angle = atan(y/x)
    except:
        angle = pi/2
    return (r,angle)

def _new_coord(x,y):
    ''' Transforma (x,y) gazebo -> (x,y) grid '''
    _x = int(_newx(x))
    _y = int(_newy(y))
    return _transform_coord(_x, _y)

def transformed_angle_xy(r, angle):
    transformed = _polar_xy(r, angle)
    x = transformed[0] + (XY[0] - ORIGIN[0])
    y = transformed[1] + (XY[1] - ORIGIN[0])

    if x > WIDTH - 1:
        x = WIDTH - 1
    elif x < 0:
        x = 0

    if y > HEIGHT - 1:
        y = HEIGHT - 1
    elif y < 0:
        y = 0

    return (x,y)

def _get_xy_sonar(range, angle):
    ''' Obtém (x,y) do sonar de acordo com o ângulo do robô '''
    xy = transformed_angle_xy(range, angle)
    xy_top = transformed_angle_xy(range, angle + angulo_visao)
    xy_bottom = transformed_angle_xy(range, angle - angulo_visao)

    return (xy, xy_top, xy_bottom)

def _get_angle_sonar(yaw):
    ''' Ajsuta o ângulo do sonar com o ângulo do robô '''
    angs = []
    for a in angulos:
        angs.append(yaw+a)
    return angs

def _set_sonar_data(sonar_index, yaw, rng):
    global SONARES
    sonar_key = 'sonar' + str(sonar_index)
    ang = angulos[sonar_index] + yaw
    (xy, xy_top, xy_bottom) = _get_xy_sonar(rng, ang)
    SONARES[sonar_key].update(
        {
            'angle': ang,
            'range': rng,
            'xy_eixo': xy,
            'xy_top': xy_top,
            'xy_bottom': xy_bottom,
        }
    )

def _path(surface):
    if ORIGINAL_XY:

        msg = Twist()
    
        robot_xy = ORIGINAL_XY

        print('\nGOAL: ', GOAL)
        print('robot_xy: ', robot_xy)
        inc_x = GOAL.x - robot_xy[0]
        inc_y = GOAL.y - robot_xy[1]
        print('Resulting vector: ', (inc_x, inc_y))

        angle_to_go = atan2(inc_y, inc_x)
        print('ANGLE TO GO: ', angle_to_go)

        print('THETA: ', THETA)
        print('angle_to_go - THETA: ', abs(angle_to_go - THETA))

        if abs(inc_x) < 0.1 and abs(inc_y) < 0.1:
            msg.linear.z = 0.0
            msg.linear.x = 0.0
        else:
            msg.linear.x = 0.0
            diff = angle_to_go - THETA

            print("Diferença: ", diff)
            if abs(diff) > 0.1:

                if diff > 0:
                    msg.angular.z = 0.1
                else:
                    msg.angular.z = -0.1
            else:
                msg.linear.x = 0.1
                msg.angular.z = 0.0


        PUBLISHER.publish(msg)

def odom_callback(msg):
    ''' Callback da odometria '''
    global XY
    global THETA
    global ORIGINAL_XY

    # Posição do robô gazebo -> grid
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ORIGINAL_XY = (x,y)
    XY = _new_coord(x,y)

    # Orientação robô gazebo
    THETA_q = msg.pose.pose.orientation
    THETA_list = [THETA_q.x, THETA_q.y, THETA_q.z, THETA_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (THETA_list)

    THETA = yaw


def sonar_callback(msg, data):
    ''' Callback do sonar '''
    if THETA:
        _set_sonar_data(data['index'], THETA, msg.range)

def _get_index_cell(xy):
    index_x = int(xy[0] / BLOCKSIZE)
    index_y = int(xy[1] / BLOCKSIZE)
    return (index_x, index_y)

def _get_xy_cell_index(index):
    ''' Retorna o (x,y) da célula, pelo seu índice '''
    x = index[0] * BLOCKSIZE
    y = index[1] * BLOCKSIZE
    return (x,y)

def _get_xy_matrix(xy, matriz_rect):
    ''' Obtém o (x,y) da célula na matriz através do (x,y) do objeto '''
    indexes = _get_index_cell(xy)
    x_normalized = indexes[0]
    y_normalized = indexes[1]
    _x = matriz_rect[x_normalized][y_normalized][0]
    _y = matriz_rect[x_normalized][y_normalized][1]
    return (_x,_y)

MATRIZ = []

def _cria_matriz():
    global MATRIZ

    map = []

    for x in range(0,WIDTH,BLOCKSIZE):
        linhas_map = []
        for y in range(0,HEIGHT,BLOCKSIZE):
            linhas_map.append((x,y))
        map.append(linhas_map)

    MATRIZ = map

    del map
    del linhas_map

''' TODO:
    Separar pintura do sonar e a atualização das células
'''
def draw_sonar(surface):

    _x_robo = XY[0]
    _y_robo = XY[1]
    cell_indexes_robo = _get_index_cell(XY)

    ''' EXIBE EIXOS SONARES '''
    for sonar in SONARES:
        sonar = SONARES[sonar]

        if len(sonar.items()) > 0:
            # Pinta eixo acústico do sonar
            pygame.draw.line(surface, (0,0,255), (_x_robo, _y_robo), (sonar['xy_eixo'][0], sonar['xy_eixo'][1]))

            # Pinta os limites do sonar
            pygame.draw.line(surface, (0,0,255), (_x_robo, _y_robo), (sonar['xy_top'][0], sonar['xy_top'][1]))
            pygame.draw.line(surface, (0,0,255), (_x_robo, _y_robo), (sonar['xy_bottom'][0], sonar['xy_bottom'][1]))

            cells = []

            ''' Varre todas as células dentro do cone e faz o cálculo das probabilidades '''

            sonar['cells'] = cells

            del cells
            gc.collect()


def draw_robot(surface):
    global POSITIONS

    robo_size = int(BLOCKSIZE/3)

    paintx = _get_xy_matrix(XY, MATRIZ)[0]
    painty = _get_xy_matrix(XY, MATRIZ)[1]

    POSITIONS.append((paintx, painty))

    for position in POSITIONS:
        pygame.draw.rect(surface, (255,0,0), (position[0], position[1],BLOCKSIZE,BLOCKSIZE), 0)

    _x = XY[0]
    _y = XY[1]

    pygame.draw.circle(surface, (0,0,255), (_x, _y), robo_size)

def show_grid(surface):
    ''' Renderiza a tela '''
    global MATRIZ

    if not XY:
        return

    # "Apaga" toda vez
    surface.fill((10,10,10))

    # Renderiza matriz
    for l in MATRIZ:
        for c in l:
            pygame.draw.rect(surface, (0,20,0), (c[0], c[1] ,BLOCKSIZE,BLOCKSIZE), 1)

def main():
    global PUBLISHER

    rospy.init_node(f'mapping_robot')

    cmd_vel_topic = '/p3dx3/cmd_vel'
    PUBLISHER = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    odom_topic = '/p3dx3/odom'
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    for i in range(0,8):
        sonar_topic = '/p3dx3/sonar' + str(i)
        rospy.Subscriber(sonar_topic, Range, sonar_callback, ({'index':i}))

    rate = rospy.Rate(5)

    _cria_matriz()

    pygame.init()
    surface = pygame.display.set_mode((WIDTH,HEIGHT))

    while not rospy.is_shutdown():

        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: pygame.quit()

        show_grid(surface)
        draw_robot(surface) 
        draw_sonar(surface)
        _path(surface)

        pygame.display.update()
        rate.sleep()

    # pygame.quit()

if __name__ == '__main__':
    main()
