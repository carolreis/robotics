#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import pygame
from math import sin, cos, atan, pi, acos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import gc
import numpy as np
import json
# import astar

mapa_f = open('mapa_para_potencial.json')
mapa = json.load(mapa_f)
OCCUPANCY = mapa

ORIGINAL_GRID_SIZE = 20
# NEW_GRID_SIZE = 81
NEW_GRID_SIZE = 41
BLOCKSIZE = 20
REGIONS = NEW_GRID_SIZE / ORIGINAL_GRID_SIZE # 2
WIDTH = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
HEIGHT = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
XY = None
QUANT_BLOCOS = (WIDTH/BLOCKSIZE)
MAX_RANGE = 3
ORIGIN = (WIDTH/2, HEIGHT/2)
MAX_SCORE = 15
MIN_SCORE = 0
GRAPH = []

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

def _get_index_cell(xy):
    index_x = int(xy[0] / BLOCKSIZE)
    index_y = int(xy[1] / BLOCKSIZE)
    return (index_x, index_y)

def _get_xy_cell_index(index):
    ''' Retorna o (x,y) da célula, pelo seu índice '''
    x = index[0] * BLOCKSIZE
    y = index[1] * BLOCKSIZE
    return (x,y)

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
    surface.fill((0,20,0))

    # Renderiza matriz
    for l in MATRIZ:
        for c in l:
            pygame.draw.rect(surface, (125,125,125), (c[0], c[1] ,BLOCKSIZE,BLOCKSIZE), 0)


# start = (10,10)
# goal = (15,42)

# start = (39,40)
# goal = (13,47)

# start = (43,40)
# goal = (50,47)

# start = (13,36)
# goal = (23,40)

# start = (43,40)
# goal = (60,47)

# start = (43,40)
# goal = (65,40)

# p = astar.PathPlanning(mapa, start, goal)
# route, newmap = p.get_route()
# print("ROUTE: ", route)
# OCCUPANCY = newmap

def main():
    global PUBLISHER

    rospy.init_node(f'show_map')

    rate = rospy.Rate(10)

    _cria_matriz()

    pygame.init()
    surface = pygame.display.set_mode((WIDTH,HEIGHT))

    while not rospy.is_shutdown():

        for event in pygame.event.get(): 
            if event.type == pygame.QUIT:
                pygame.quit()

        show_grid(surface)
        draw_occupancy(surface)

        # for r in route:
        #     xy = _get_xy_cell_index(r)
        #     pygame.draw.rect(surface, (0,255,0), (xy[0], xy[1] ,BLOCKSIZE,BLOCKSIZE), 1)

        pygame.display.update()
        rate.sleep()

    pygame.quit()

if __name__ == '__main__':
    main()
