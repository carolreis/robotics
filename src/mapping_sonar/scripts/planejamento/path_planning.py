#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import pygame
from math import sin, cos, atan, pi, acos, sqrt, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import gc
import numpy as np
import json
import new_astar


ORIGINAL_GRID_SIZE = 20
NEW_GRID_SIZE = 81
BLOCKSIZE = 10
REGIONS = NEW_GRID_SIZE / ORIGINAL_GRID_SIZE
WIDTH = int(BLOCKSIZE * NEW_GRID_SIZE)
HEIGHT = int(BLOCKSIZE * NEW_GRID_SIZE)
XY = None
ORIGINAL_XY = None
QUANT_BLOCOS = (WIDTH/BLOCKSIZE)
MAX_RANGE = 3
ORIGIN = (int(WIDTH/2), int(HEIGHT/2))
MAX_SCORE = 15
MIN_SCORE = 0
GRAPH = []

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
angulo_visao = 0.267000/2
angulos = [pi/2, (pi/2/1.8), (pi/2)/3, (pi/2)/9, -(pi/2)/9,  -(pi/2)/3, -(pi/2/1.8), -pi/2] # Em relação ao robô

THETA = None

''' X '''
def _inv_cell_gazebo_grid_x(c):
    cx = ( int(NEW_GRID_SIZE / 2) - c) * (ORIGINAL_GRID_SIZE / NEW_GRID_SIZE)
    return cx

def _cell_gazebo_grid_x(x):
    _x = ( x + int(ORIGINAL_GRID_SIZE / 2) ) * REGIONS
    return _x

def _newx(x):
    ''' Coordenada gazebo x -> grid x, multiplica pelo tamanho do bloco '''
    _x = _cell_gazebo_grid_x(x)
    _x = _x * BLOCKSIZE
    return int(_x)

def _inv_newx(x):
    print('inv newx: ', x)
    _x = x / BLOCKSIZE
    print('inv newx: ', _x)
    _x = _inv_cell_gazebo_grid_x(_x)
    print('inv newx: ', _x)
    return _x

''' Y '''
def _inv_cell_gazebo_grid_y(c):
    cy = ( int(NEW_GRID_SIZE / 2) - c) * (ORIGINAL_GRID_SIZE / NEW_GRID_SIZE)
    return cy

def _cell_gazebo_grid_y(y):
    _y = ( y - int(ORIGINAL_GRID_SIZE / 2) ) * (-1) * REGIONS
    return _y

def _newy(y):
    ''' Coordenada gazebo y -> grid y, multiplica pelo tamanho do bloco '''
    _y = _cell_gazebo_grid_y(y)
    _y = _y * BLOCKSIZE
    return int(_y)

def _inv_newy(y):
    print('inv new _y: ', y)
    _y = y / BLOCKSIZE
    print('inv new _y: ', _y)
    _y = _inv_cell_gazebo_grid_y(_y)
    print('inv new _y: ', _y)
    return _y

''' AXIS TRANSFORMATION '''
def _inv_transform_coord(x,y):
    ''' Inverte os eixos '''
    # XY = (x, -y)
    XY = (x, y)
    return XY

def _transform_coord(x,y):
    ''' Inverte os eixos '''
    XY = (x, y)
    XY = (y, HEIGHT - x)
    return XY

''' COORD TRANSFORMATIONS '''
def _polar_xy(raio, angulo):
    ''' Gazebo (r,angulo) -> (x,y) grid ''' 
    x = _newx(raio*cos(angulo))
    y = _newy(raio*sin(angulo))
    return _transform_coord(x,y)

def _inv_new_coord(x,y):
    _x = _inv_newx(x)
    _y = _inv_newy(y)
    print('inv_new_coord: ', (_x, _y))
    return _inv_transform_coord(_x, _y)

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

def _get_index_cell_grid(xy):
    ''' Recebe um (x,y) do grid e retorna a célula do grid '''
    index_x = int(xy[0] / BLOCKSIZE)
    index_y = int(xy[1] / BLOCKSIZE)
    return (index_x, index_y)

def _get_xy_cell_index(index):
    ''' Retorna o (x,y) da célula, pelo seu índice '''
    x = index[0] * BLOCKSIZE
    y = index[1] * BLOCKSIZE
    return (x,y)

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

def _get_xy_sonar(range, angle):
    ''' Obtém (x,y) do sonar de acordo com o ângulo do robô '''
    xy = transformed_angle_xy(range, angle)
    xy_top = transformed_angle_xy(range, angle + angulo_visao)
    xy_bottom = transformed_angle_xy(range, angle - angulo_visao)

    return (xy, xy_top, xy_bottom)

def _get_xy_matrix(xy, matriz_rect):
    ''' Obtém o (x,y) da célula na matriz através do (x,y) do objeto '''
    indexes = _get_index_cell_grid(xy)
    x_normalized = indexes[0]
    y_normalized = indexes[1]
    _x = matriz_rect[x_normalized][y_normalized][0]
    _y = matriz_rect[x_normalized][y_normalized][1]
    return (_x,_y)


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

def sonar_callback(msg, data):
    ''' Callback do sonar '''
    if THETA:
        _set_sonar_data(data['index'], THETA, msg.range)

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

def _path(surface=None):
    if XY:

        # print('\n====================================================== ')
        GOAL = Point()

        global route

        if len(route) > 0:
            r = route[0]
            ''' CÉLULA '''
            print("Célula objetivo grid (rota): ", r)

            cogx = _inv_cell_gazebo_grid_x(r[1])
            cogy = _inv_cell_gazebo_grid_y(r[0])
            celula_objetivo_gazebo = _inv_transform_coord(cogx, cogy)
            print("Célula objetivo gazebo: ", celula_objetivo_gazebo)

            ''' ---- Pinta célula '''
            xy = _get_xy_cell_index(r)
            if surface:
                pygame.draw.rect(surface, (0,255,0), (xy[0], xy[1] ,BLOCKSIZE,BLOCKSIZE), 1)

            ''' ROBO '''
            robo_celula = _get_index_cell_grid(XY)
            # print("\nRobô XY: ", XY)
            robo_gazebo_xy = (round(ORIGINAL_XY[0],4), round(ORIGINAL_XY[1],4))
            print("\nRobô gazebo ORIGINAL XY: ", ORIGINAL_XY)
            print('Robo gazebo XY', robo_gazebo_xy)
            print('Robo celula grid: ', robo_celula)

            ''' ORIGEM '''
            origin_index = _get_index_cell_grid(ORIGIN)
            print("\nCélula origem no grid: ", origin_index)

            origin_gazebo = (_inv_cell_gazebo_grid_x(origin_index[0]), _inv_cell_gazebo_grid_y(origin_index[1]))
            print("Origin gazebo: ", origin_gazebo)

            ''' OBJETIVO '''
            GOAL.x = celula_objetivo_gazebo[0]
            GOAL.y = celula_objetivo_gazebo[1]


            result_vetor_x = GOAL.x - robo_gazebo_xy[0]
            result_vetor_y = GOAL.y - robo_gazebo_xy[1]
            print("\nResulting vector: ", (result_vetor_x, result_vetor_y))

            try:
                angle_to_go = atan2(result_vetor_y,result_vetor_x)
            except:
                angle_to_go = pi/2

            anguloRobo = THETA
        
            diff = angle_to_go - anguloRobo

            # TODO: TESTE
            angle_to_go += pi
            anguloRobo += pi

            print('\nAngle to go: ', angle_to_go)
            print('Ângulo do robô: ', anguloRobo)

            print("Diferença: ", diff)

            ''' MOVIMENTO '''
            msg = Twist()

            print('\n')
            if robo_celula == r:
                msg.linear.z = 0.0
                msg.linear.x = 0.0
                route.pop(0)
            else:

                msg.linear.x = 0.0

                if abs(diff) > 0.1:

                    if diff > 0:
                        msg.angular.z = 0.2
                    else:
                        msg.angular.z = -0.2
                
                else:
                    print('andar')
                    msg.linear.x = 0.3
                    msg.angular.z = 0.0

            PUBLISHER.publish(msg)

            # robo_gazebo_xy = _inv_new_coord(XY[0], XY[1])
            # celula_objetivo_gazebo = (_inv_cell_gazebo_grid_x(r[0]), _inv_cell_gazebo_grid_y(r[1]))
            # GOAL.x = celula_objetivo_gazebo[0] - origin_gazebo[0]
            # GOAL.y = celula_objetivo_gazebo[1] - origin_gazebo[1]    
            # if abs(result_vetor_x) <= 0.05 and abs(result_vetor_y) <= 0.05:

MATRIZ = []

def _cria_matriz():
    global MATRIZ

    map = []

    for x in range(0,WIDTH,BLOCKSIZE):
        linhas_map = []

        for y in range(0,HEIGHT,BLOCKSIZE):
            linhas_map.append((x,y))
        map.append(linhas_map)

    MATRIZ = map.copy()

    del map


def draw_occupancy(surface):
    for indice_linha in range(0, len(OCCUPANCY)):
        for indice_coluna in range(0, len(OCCUPANCY[indice_linha])):
            xy = _get_xy_cell_index((indice_linha,indice_coluna))
            try:
                pygame.draw.rect(
                    surface,
                    (
                        abs((255 / MAX_SCORE) * OCCUPANCY[indice_linha][indice_coluna]),
                        abs((255 / MAX_SCORE) * OCCUPANCY[indice_linha][indice_coluna]),
                        abs((255 / MAX_SCORE) * OCCUPANCY[indice_linha][indice_coluna])
                    ),
                    (xy[0], xy[1],BLOCKSIZE, BLOCKSIZE),
                    1
                )
            except:
                pass

def show_grid(surface):
    ''' Renderiza a tela '''
    global MATRIZ

    # "Apaga" toda vez
    surface.fill((0,0,0))

    # Renderiza matriz
    for l in MATRIZ:
        for c in l:
            pygame.draw.rect(surface, (0,20,0), (c[0], c[1] ,BLOCKSIZE,BLOCKSIZE), 1)

mapa_f = open('MAPA.json')
mapa = json.load(mapa_f)

OCCUPANCY = mapa

# goal_gazebo = (_inv_cell_gazebo_grid_x(goal[0]), _inv_cell_gazebo_grid_x(goal[0]))
# print('GOAL GAZEBO: ', goal_gazebo)
# route = [(40,40), (40,41), (40,42), (40,43), (41,43), (41,45), (41,46)]

def main():
    global PUBLISHER

    rospy.init_node(f'show_map')


    cmd_vel_topic = '/p3dx3/cmd_vel'
    PUBLISHER = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    odom_topic = '/p3dx3/odom'
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    for i in range(0,8):
        sonar_topic = '/p3dx3/sonar' + str(i)
        rospy.Subscriber(sonar_topic, Range, sonar_callback, ({'index':i}))


    rate = rospy.Rate(10)

    _cria_matriz()

    pygame.init()
    surface = pygame.display.set_mode((WIDTH,HEIGHT))

    begin = False

    global route

    while not rospy.is_shutdown():

        if not begin and XY:
            #  TODO: Pegar do robô
            #  Converter coordenadas goal gazebo -> goal grid
            # start = _get_index_cell_grid(XY)

            # start = (39,40)
            # goal = (12,47)

            # # Teste 1  - dir baixo
            # start = (45,42)
            # goal = (48,42)

            # # Teste 2 - esq baixo
            # start = (35,42)
            # goal = (48,42)

            # Teste 3 - com lista
            start = (41,41)
            # goal = (20,12)
            goal = (10,22)

            print('GOAL GRID: ', goal)
            print('START: ', start)
            # p = astar.PathPlanning(mapa, start, goal)
            # route, new_map = p.get_route()  # route grid
            print('MAPA GAZEBO: ', mapa)
            p = new_astar.AStar(start, goal, mapa)
            route = p.path_planning()  # route grid

            # route_gazebo = []
            # for r in route:
            #     new_r = (_inv_cell_gazebo_grid_x(r[0]), _inv_cell_gazebo_grid_y(r[1]))
            #     route_gazebo.append(new_r)

            print('ROUTE GRID: ', route)
            # print('ROUTE GAZEBO: ', route_gazebo)

            begin = True

        for event in pygame.event.get(): 
            if event.type == pygame.QUIT:
                pygame.quit()

        show_grid(surface)
        draw_robot(surface)
        draw_occupancy(surface)
        _path(surface)

        for r in route:
            xy = _get_xy_cell_index(r)
            pygame.draw.rect(surface, (0,255,0), (xy[0], xy[1] ,BLOCKSIZE,BLOCKSIZE), 1)

        pygame.display.update()
        rate.sleep()

    pygame.quit()

if __name__ == '__main__':
    main()
