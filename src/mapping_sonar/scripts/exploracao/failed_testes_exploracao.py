#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import pygame
from math import exp, sin, cos, atan, pi, acos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import gc
import potential
import operator
import numpy as np
from math import atan2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


ORIGINAL_GRID_SIZE = 20
NEW_GRID_SIZE = 81
BLOCKSIZE = 10
REGIONS = NEW_GRID_SIZE / ORIGINAL_GRID_SIZE # 2
WIDTH = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
HEIGHT = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
XY = None
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

MAX_SCORE = 15
MIN_SCORE = 0

ORIENTATION = None
KERNEL = [
    [0.5, 0.5, 0.5],
    [0.5, 1, 0.5],
    [0.5, 0.5, 0.5]
]

MATRIZ = []
OCCUPANCY = []
VISITED_MATRIX = []

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

def odom_callback(msg):
    ''' Callback da odometria '''
    global XY
    global ORIENTATION

    # Posição do robô gazebo -> grid
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    XY = _new_coord(x,y)

    # Orientação robô gazebo
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    ORIENTATION = yaw

def sonar_callback(msg, data):
    ''' Callback do sonar '''
    if ORIENTATION:
        _set_sonar_data(data['index'], ORIENTATION, msg.range)

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

def _cria_matriz():
    global MATRIZ
    global OCCUPANCY
    global VISITED_MATRIX

    map = []
    occupancy_matrix = []

    for x in range(0,WIDTH,BLOCKSIZE):
        linhas_map = []
        linhas_occupancy = []
        for y in range(0,HEIGHT,BLOCKSIZE):
            linhas_map.append((x,y))
            linhas_occupancy.append(0)
        map.append(linhas_map)
        occupancy_matrix.append(linhas_occupancy)

    MATRIZ = map
    OCCUPANCY = occupancy_matrix
    VISITED_MATRIX = np.zeros_like(OCCUPANCY)

    del map
    del linhas_map
    del linhas_occupancy
    del occupancy_matrix

def calc_pot(cel_robo, sonares, OCCUPANCY, recent_visited):

    f = 12  # 3 células gazebo
    x_robo = cel_robo[0]
    y_robo = cel_robo[1]
    grid = np.array(OCCUPANCY)
    janela = grid[x_robo-f:x_robo+f, y_robo-f:y_robo+f]
    print('window: ', janela)

    # try:

        # #  TODO: Fazer isso recursivo
        # print("\n")
        # def menor_maior(cells):
        #     menor_x = min(cells, key=operator.itemgetter(0))[0] # -> ()
        #     menor_y = min(cells, key=operator.itemgetter(1))[1]
        #     maior_x = max(cells, key=operator.itemgetter(0))[0]
        #     maior_y = max(cells, key=operator.itemgetter(1))[1]
        #     return (menor_x, menor_y, maior_x, maior_y)

        # todos_menores = []
        # todos_maiores = []
        # for s_key in sonares:
        #     s = sonares[s_key]
        #     cells = s['cells']
        #     if cel_robo not in cells:
        #         cells.append(cel_robo)
        #     print("... Células do sonar %s: %s " % (s_key, cells))
        #     try:
        #         menor_x, menor_y, maior_x, maior_y = menor_maior(cells)
        #         menores = (menor_x, menor_y)
        #         print('menores: ', menores)
        #         maiores = (maior_x, maior_y)
        #         print('maiores: ', maiores)
        #         if menores not in todos_menores:
        #             todos_menores.append(menores)
        #         if maiores not in todos_maiores:
        #             todos_maiores.append(maiores)

        #     except:
        #         pass

        # print("... Todos os maiores: ", todos_maiores)
        # print("... Todos os menores: ", todos_menores)

        # c = todos_menores
        # for maiores in todos_maiores:
        #     if maiores not in c:
        #         c.append(maiores)
        
        # print("Todos: ", c)
        # # for d in c:
        #     # print("index %s xy %s " % (d, _get_xy_cell_index(d)))

        # return [],[],[],[]
        # menor_x, menor_y, maior_x, maior_y = menor_maior(c)
        # print('final: ', (menor_x, menor_y, maior_x, maior_y))

        # dimension = max(abs(maior_x-menor_x), abs(maior_y-menor_y))
        # print('dimension: ', dimension)
        # np_occupancy = np.array(OCCUPANCY)
        # print('index: ', (menor_x, ' ', menor_x+dimension+1, ' ', menor_y , ' ' ,menor_y+dimension+1))
        # # sub_matrix = np_occupancy[menor_x:menor_x+dimension+1, menor_y:menor_y+dimension+1]
        # sub_matrix = np_occupancy[menor_x:menor_x+dimension+1, menor_y:menor_y+dimension+1]
        # print('sub matrix: ', sub_matrix)
        # print('sub matrix shape: ', sub_matrix.shape)

        # full_obstacle_matrix = np.zeros_like(OCCUPANCY)
        # full_obstacle_matrix[menor_x:menor_x+dimension+1, menor_y:menor_y+dimension+1] = sub_matrix

        # try:
        #     # # calcula pot
        #     hpf = potential.HarmonicPotentialField(sub_matrix)
        #     dx, dy, pot = hpf.calc()
        # except:
        #     dx = np.zeros(sub_matrix[0])
        #     dy = np.zeros(sub_matrix[0])
        #     pot = np.zeros(sub_matrix[0])

        # print('pot: ', pot)
        # print('pot.shape: ', pot.shape)
        # print('dx: ', dx)
        # print('dx shape: ', dx.shape)
        # print('dy: ', dy)
        # print('dy shape: ', dy.shape)

        # robo_cel_relative = (int(dx.shape[0]/2), int(dy.shape[0]/2))
        # print('robo_cel_relative: ', robo_cel_relative)

        # # TODO: Retirar
        # linspc = np.linspace(0, 1, len(dx))
        # xx = np.outer(linspc, np.ones(len(dx))).T
        # yy = np.outer(linspc, np.ones(len(dx)))

        # plt.rc('text')
        # plt.rc('font', family='sans-serif')
        # plt.rc('xtick',labelsize=10)
        # plt.rc('ytick',labelsize=10)
        # plt.figure(figsize=(40,40))
        # ax = plt.axes(projection='3d')
        # ax.set_xlabel('x', fontsize = 10)
        # ax.set_ylabel('y',fontsize = 10)
        # ax.set_zlabel('f(x,y)',fontsize = 10)
        # ax.plot_surface(xx, yy, pot,cmap='nipy_spectral', edgecolor='none')
        # plt.show()

        # return dx, dy, robo_cel_relative, pot
    #     return [], [], [], []

    # except BaseException as e:
    #     print('e: ', e)
    return [], [], [], []
        

''' TODO:
    Separar pintura do sonar e a atualização das células
'''
def draw_sonar(surface):

    _x_robo = XY[0]
    _y_robo = XY[1]
    cell_indexes_robo = _get_index_cell(XY)

    recent_visited = []
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

            # Raio de [0,3], de 0.01 em 0.01
            for r in [r/20 for r in range(0,60)]:

                a = sonar['angle']

                # Pega (x,y) da célula
                xy = transformed_angle_xy(r, a)

                # Pega índice da célula
                xy_indexes = _get_index_cell(xy)

                # Se a célula não foi armazenada anteriormente
                if (xy_indexes) not in cells and sonar['range'] < MAX_RANGE:

                    # Adiciona célula na lista
                    cells.append(xy_indexes)

                    R = sonar['range']  # Range do gazeno

                    if r < (R + 0.2):

                        if r < (R - 0.2):
                            if OCCUPANCY[xy_indexes[0]][xy_indexes[1]] != MIN_SCORE:
                                OCCUPANCY[xy_indexes[0]][xy_indexes[1]] -= 1
                        else:
                            if OCCUPANCY[xy_indexes[0]][xy_indexes[1]] < MAX_SCORE:
                                OCCUPANCY[xy_indexes[0]][xy_indexes[1]] += 3
                    
                        VISITED_MATRIX[xy_indexes[0]][xy_indexes[1]] = 1
                        recent_visited.append(xy_indexes)
 
            del cells
            gc.collect()

    # print('recent_visited: ', recent_visited)
    # print('VISITED MATRIX: ', VISITED_MATRIX)
    # print('VISITED MATRIX shape: ', VISITED_MATRIX.shape)
    print('VISITED MATRIX == 1: ', np.where(VISITED_MATRIX == 1))

    dx, dy, robo_cel_relative, pot = calc_pot(cell_indexes_robo, SONARES, OCCUPANCY, recent_visited)
    # return dx, dy, robo_cel_relative, pot
    return [],[],[],[]


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

def draw_visited(surface):

    for indice_linha in range(0, len(VISITED_MATRIX)):
        for indice_coluna in range(0, len(VISITED_MATRIX[indice_linha])):
            xy = _get_xy_cell_index((indice_linha,indice_coluna))
            try:
                pygame.draw.rect(
                    surface,
                    (
                        abs(255 * VISITED_MATRIX[indice_linha][indice_coluna]),
                        abs(255 * VISITED_MATRIX[indice_linha][indice_coluna]),
                        abs(255 * VISITED_MATRIX[indice_linha][indice_coluna])
                    ),
                    (xy[0], xy[1],BLOCKSIZE, BLOCKSIZE),
                    1
                )
            except:
                pass

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

def path(dx, dy, robo_cel_relative, pot):
    try:
        print("angulo robo: ", ORIENTATION)
        rdx = dx[robo_cel_relative[0]][robo_cel_relative[1]]
        rdy = dy[robo_cel_relative[0]][robo_cel_relative[1]]
        angle_to_go = atan2(rdy, rdx)
        # angle_to_go = atan2(rdx, -rdy)
        print('angulo para virar: ', angle_to_go)
    except:
        angle_to_go = ORIENTATION

    diff = angle_to_go - ORIENTATION
    print('diff: ', diff)
    print('abs(diff): ', abs(diff))

    # # # TODO: TESTE
    # angle_to_go += pi
    # anguloRobo += pi

    # print('\nAngle to go: ', angle_to_go)
    # print('Ângulo do robô: ', anguloRobo)

    # print("Diferença: ", diff)

    # ''' MOVIMENTO '''
    msg = Twist()

    # print('\n')

    # msg.linear.x = 0.0

    if abs(diff) > 0.1:
        print("diff > 0.1: ", abs(diff))
        msg.angular.z = 0.1
        msg.linear.x = 0.0
    else:
        print("diff < 0.1: ", abs(diff))
        msg.angular.z = 0
        msg.linear.x = 0.1
    
    print('msg: ', msg)

    PUBLISHER.publish(msg)

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

    rate = rospy.Rate(1)

    _cria_matriz()

    pygame.init()
    surface = pygame.display.set_mode((WIDTH,HEIGHT))

    while not rospy.is_shutdown():

        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: pygame.quit()
        show_grid(surface)
        draw_robot(surface)
        draw_occupancy(surface)
        draw_visited(surface)
        dx, dy, robo_cel_relative, pot = draw_sonar(surface)
        # path(dx, dy, robo_cel_relative, pot)

        pygame.display.update()
        rate.sleep()

    # pygame.quit()

if __name__ == '__main__':
    main()
