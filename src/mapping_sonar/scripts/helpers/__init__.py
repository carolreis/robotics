#! /usr/bin/env python3
from math import sin, cos, atan, pi, acos, sqrt, atan2

''' ========== DEFINITIONS ========== '''
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

''' ========== FUNCTIONS ========== '''

def _newx(x):
    ''' Coordenada gazebo x -> grid x, multiplica pelo tamanho do bloco '''
    _x = ( x + int(ORIGINAL_GRID_SIZE / 2) ) * REGIONS
    _x = _x * BLOCKSIZE
    return int(_x)

def _inv_newx(x):
    _x = x / BLOCKSIZE
    _x = ( _x - int(ORIGINAL_GRID_SIZE / 2)) * (ORIGINAL_GRID_SIZE / NEW_GRID_SIZE)
    return _x

def _newy(y):
    ''' Coordenada gazebo y -> grid y, multiplica pelo tamanho do bloco '''
    _y = ( y - int(ORIGINAL_GRID_SIZE / 2) ) * (-1) * REGIONS
    _y = _y * BLOCKSIZE
    return int(_y)

def _inv_newy(y):
    _y = y / BLOCKSIZE
    _y = ( _y - int(ORIGINAL_GRID_SIZE / 2)) * (ORIGINAL_GRID_SIZE / NEW_GRID_SIZE)
    return _y

def _transform_coord(x,y):
    ''' Inverte os eixos '''
    XY = (x, y)
    XY = (y, HEIGHT - x)
    return XY

def _inv_transform_cood(x,y):
    pass

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
