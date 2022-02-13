#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pygame


ORIGINAL_GRID_SIZE = 20
NEW_GRID_SIZE = 41
BLOCKSIZE = 20
REGIONS = NEW_GRID_SIZE / ORIGINAL_GRID_SIZE # 2
WIDTH = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
HEIGHT = int(BLOCKSIZE * NEW_GRID_SIZE) # 100
RUN = True
XY = None
QUANT_BLOCOS = (WIDTH/BLOCKSIZE) # 4

print('WIDTH: ', WIDTH)
print('HEIGHT: ', HEIGHT)

def _newx(x):
    print('x gazebo: ', x)
    _x = ( x + int(ORIGINAL_GRID_SIZE / 2) ) * REGIONS
    print('_x: ', _x)

    _x = _x * BLOCKSIZE
    print('_x * blocksize: ', _x)
    return int(_x)
    
def _newy(y):
    print('y gazebo: ', y)
    _y = ( y - int(ORIGINAL_GRID_SIZE / 2) ) * (-1) * REGIONS
    print('_y: ', _y)
    _y = _y * BLOCKSIZE
    print('_y * blocksize: ', _y)
    return int(_y)

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    global XY
    _x = int(_newx(x))
    _y = int(_newy(y))

    XY = (_x, _y)
    XY = (_y, HEIGHT - _x)


def quit_grid():
    pygame.quit()


def show_grid():

    pygame.init()
    
    surface = pygame.display.set_mode((WIDTH,HEIGHT))

    global RUN
    POSITIONS = []

    while RUN:

        matriz_rect = []

        if not XY:
            continue

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                RUN = False

        surface.fill((0,0,0))

        # CRIA MATRIZ
        for x in range(0,WIDTH,BLOCKSIZE):

            linhas = []

            for y in range(0,HEIGHT,BLOCKSIZE):

                pygame.draw.rect(surface, (255,0,0), (x, y,BLOCKSIZE,BLOCKSIZE), 1)

                linhas.append((x,y))
            matriz_rect.append(linhas)

        try:

            robo_size = int(BLOCKSIZE/3)

            # global POSITIONS
            xnormal = int(XY[0] / BLOCKSIZE)
            ynormal = int(XY[1] / BLOCKSIZE)

            print('XY: ', XY)
            print('xnormal: ', xnormal)
            print('ynormal: ', ynormal)

            paintx = matriz_rect[xnormal][ynormal][0]
            painty = matriz_rect[xnormal][ynormal][1]
            POSITIONS.append((paintx, painty))

            for position in POSITIONS:
                pygame.draw.rect(surface, (255,0,0), (position[0], position[1],BLOCKSIZE,BLOCKSIZE), 0)

            _x = XY[0]
            _y = XY[1]
            pygame.draw.circle(surface, (0,0,255), (_x, _y), robo_size)

            pygame.display.update()
        except:
            pass

        # RUN = False

    pygame.quit()


def main():
    global PUBLISHER

    rospy.init_node(f'mapping_robot')

    odom_topic = '/p3dx/odom'
    cmd_vel_topic = '/p3dx/cmd_vel'

    PUBLISHER = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    show_grid()

    rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    #     rate.sleep()


if __name__ == '__main__':
    main()
