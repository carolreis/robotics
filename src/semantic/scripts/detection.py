#! /usr/bin/env python3

from math import sin, cos, atan, pi, acos, sqrt, atan2

# ROS imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid

import numpy as np
from scipy import stats

## CONSTANTS
D = None

# Robot
CURRENT_POSE = None #  (X, Y, THETA)
PREV_POSE = None  # (X, Y, THETA)
STATE = 'STILL' #  'MOVING'

# YOLO
DARKNET = {
    'xmin': None,
    'xmax': None,
    'ymin': None,
    'ymax': None,
    'id': None,
    'class': None
}
BOUNDING_BOX_CENTROID = None

# Camera
PI_180 = 0.01745329251994329577
CAMERA_DEG_ANGLE = 69.4
CAMERA_RAD_ANGLE = CAMERA_DEG_ANGLE * PI_180
CAMERA_COLOR_WIDTH = 640
CAMERA_COLOR_HEIGHT = 480
CAMERA_RAD_ANGLE_PER_PIXEL = CAMERA_COLOR_WIDTH / CAMERA_RAD_ANGLE
CAMERA_DEPTH_WIDTH = 1280
CAMERA_DEPTH_HEIGHT = 720
DEPTH_VALUE = None
DEPTH_X = None
DEPTH_Y = None
PIXEL_COLOR_X = None
PIXEL_COLOR_Y = None

# Objects
CLASSES = [
    'person',
    'bicycle',
    'car',
    'motorbike',
    'aeroplane',
    'bus',
    'train',
    'truck',
    'boat',
    'traffic light',
    'fire hydrant',
    'stop sign',
    'parking meter',
    'bench',
    'bird',
    'cat',
    'dog',
    'horse',
    'sheep',
    'cow',
    'elephant',
    'bear',
    'zebra',
    'giraffe',
    'backpack',
    'umbrella',
    'handbag',
    'tie',
    'suitcase',
    'frisbee',
    'skis',
    'snowboard',
    'sports ball',
    'kite',
    'baseball bat',
    'baseball glove',
    'skateboard',
    'surfboard',
    'tennis racket',
    'bottle',
    'wine glass',
    'cup',
    'fork',
    'knife',
    'spoon',
    'bowl',
    'banana',
    'apple',
    'sandwich',
    'orange',
    'broccoli',
    'carrot',
    'hot dog',
    'pizza',
    'donut',
    'cake',
    'chair',
    'sofa',
    'pottedplant',
    'bed',
    'diningtable',
    'toilet',
    'tvmonitor',
    'laptop',
    'mouse',
    'remote',
    'keyboard',
    'cell phone',
    'microwave',
    'oven',
    'toaster',
    'sink',
    'refrigerator',
    'book',
    'clock',
    'vase',
    'scissors',
    'teddy bear',
    'hair drier',
    'toothbrush',
]

# Readings storage
'''
    Object structure:
    {
        'x': 0,
        'y': 0,
        'class': '',

        #  TO GET IT LATER
        'width': 0,
        'height': 0
    }
'''
READINGS = []

OBJECT_STRUCTURE = {
    'x': 0,
    'y': 0,
    'Class': ''
}

# >>>>> Map
MAP_WIDTH = None
MAP_HEIGHT = None
MAP_ORIGIN = None
MAP_DATA = None
MAP_RESOLUTION = None
MAP_HEADER = None
MAP_INFO = None

'''
  // It transforms the coordinate system from the Odom to the Map
  std::tuple<int, int> transformCoordinateOdomToMap(float x, float y) {
    int j = y / map_resolution_ - map_origin_y_ / map_resolution_;
    int i = x / map_resolution_ - map_origin_x_ / map_resolution_;
    return std::make_tuple(i, j);
  }

  // It transforms the coordinate system from the Map to the Odom
  std::tuple<float, float> transformCoordinateMapToOdom(int x, int y) {
    float i = (x + map_origin_x_ / map_resolution_) * map_resolution_;
    float j = (y + map_origin_y_ / map_resolution_) * map_resolution_;
    return std::make_tuple(i, j);
  }
'''

# It transforms the coordinate system from the Odom to the Map
def transform_coord_odom_top_map(x, y):
    j = y / MAP_RESOLUTION - MAP_ORIGIN.y / MAP_RESOLUTION
    i = x / MAP_RESOLUTION - MAP_ORIGIN.x / MAP_RESOLUTION
    return (int(i), int(j))

# It transforms the coordinate system from the Map to the Odom
def transform_coord_map_to_odom(x, y):
    i = (x + MAP_ORIGIN.x / MAP_RESOLUTION) * MAP_RESOLUTION
    j = (y + MAP_ORIGIN.y / MAP_RESOLUTION) * MAP_RESOLUTION
    return (i,j)

def matrix_indices_to_vector_index(i, j):
    return i + j * MAP_WIDTH

def create_global_map_message(a):
    x = a[0]
    y = a[1]
    radius = 5

    grid = OccupancyGrid()
    grid.header = MAP_HEADER
    grid.info = MAP_INFO
    grid.data = [None] * (MAP_HEIGHT * MAP_WIDTH)

    #  Copy map
    for h in range(0, MAP_HEIGHT):
        m = h * MAP_WIDTH
        for w in range(0, MAP_WIDTH):
            i = w + m

            if h == y and w == x:
                print("H: %s | W: %s" % (h, w))
                grid.data[i] = 80
            else:
                grid.data[i] = MAP_DATA[i]

    # Radius
    for i in range(y - radius, y + radius):
        for j in range(x - radius, x + radius):
            if x >= 0 and x < MAP_WIDTH and y >= 0 and y < MAP_HEIGHT:
                idx = matrix_indices_to_vector_index(j, i)
                grid.data[idx] = 80

    PUBLISHER.publish(grid)

def mock_found_objects():
    x = [10, 12, 30, 40]
    y = [5, 7, 15, 30]
    classes = ['bench', 'sofa', 'desk', 'chair']

    for i in range(0, len(x)):
        obj_structure = OBJECT_STRUCTURE.copy()
        obj_structure['x'] = x[i]
        obj_structure['y'] = y[i]
        obj_structure['Class'] = classes[i]
        READINGS.append(obj_structure)

def euclidian_distance(x,y):
    return sqrt(x**2 + y**2)

# O mock não pode ser feito no loop. Dã.
# mock_found_objects()

#  Objects distance
def object_match(current_obj, object_class):

    #  TODO: Colocar essa condição em outro lugar depois
    '''
        Não é bem que só pode conferir se está se movendo...
        É que se adicionou uma vez já, e ficou parado,
        não precisa continuar verificando
    '''
    # if STATE == 'MOVING':
    if True:

        #  TODO: Descobrir um threshold aceitável
        thresold = 3  # In meters

        current_coordinate = (current_obj[0], current_obj[1])
        current_d = euclidian_distance(current_coordinate[0], current_coordinate[1])

        seen = False

        #  TODO: Silly approach
        for i in range(0, len(READINGS)):

            obj = READINGS[i]

            print("\n... LIST Object ID: ", i)
            print("... CURRENT OBJECT: ", current_obj)
            print("... LIST OBJECT: ", obj)

            prev_d = euclidian_distance(obj['x'], obj['y'])
            print('... LIST Object distance from origin: ', prev_d)

            print('... CURRENT Object distance from origin: ', current_d)

            diff = abs(current_d - prev_d)
            print("... DIFFERENCE: ", diff)

            if (diff < thresold):
                seen = True
                print("seen!!! ")
                break

        print('seen value:? ', seen)

        if not seen:
            print('Not seen, add it')
            obj_structure = OBJECT_STRUCTURE.copy()
            obj_structure['x'] = current_coordinate[0]
            obj_structure['y'] = current_coordinate[1]
            obj_structure['Class'] = object_class
            READINGS.append(obj_structure)

        print("Final readings: ", READINGS)

def check_robot_state():
    global STATE

    if PREV_POSE:

        x_diff = abs(CURRENT_POSE[0] - PREV_POSE[0])
        y_diff = abs(CURRENT_POSE[1] - PREV_POSE[1])

        theta_diff = abs(CURRENT_POSE[2] - PREV_POSE[2])
        d = euclidian_distance(x_diff, y_diff)

        euclidian_threshold = 0.0005 # Meters
        angular_threshold = 0.0005 # Rad

        if d > euclidian_threshold or theta_diff > angular_threshold:
            STATE = 'MOVING'
        else:
            STATE = 'STILL'

    print("STATE: ", STATE)

def align_depth_to_color(x, min_dimension, max_dimension):
    """ You have a small image centered on a bigger one.
        You have a pixel on the small image, let's stay, pixel 10.
        You want the corresponding pixel on the bigger image, which could be, 
        for instance, pixel 20.
        That's what it does. :)
    """

    """              ~700/800
    0 .......... 640 .......... 1280
         0 ..... 320 ..... 640
                     . (330)

           ~500
    0 .......... 640 .......... 1280
    0 ..... 320 ..... 640
               . (330)

    """
    prop = min_dimension / max_dimension
    xu = (x/prop)
    # shift = (max_dimension - min_dimension)
    # xu = (x/prop) - shift
    return int(xu)


def get_object_angle_pov_robot(x_angle, THETA):
    '''
    Centro como 0º
    '''
    return x_angle - THETA

def get_object_angle(robo_angle, pixel_x):
    """ 
        Get the angle of a pixel from the image,
        based on the opening angle from camera,
        and based on the angle from the robot - which is the center of the image
    """
    # half_camera_angle = CAMERA_RAD_ANGLE / 2
    # angle = robo_angle + half_camera_angle - (CAMERA_RAD_ANGLE_PER_PIXEL * pixel_x)

    metade = CAMERA_COLOR_WIDTH / 2

    p = pixel_x - metade

    angle = (p / CAMERA_RAD_ANGLE_PER_PIXEL) * (-1)

    b = robo_angle + angle

    if b > pi:
        b = b - (2*pi)
    elif b < -pi:
        b = b + (2*pi)

    return b

def get_world_xy_from_angle(object_angle, d, x_robot, y_robot):
    """ It gets the (x,y) based on angle
        And it sums up the robot coordinates
        In order to return the coordinates based on world coordinates
    """
    try:
        object_x = cos(object_angle) * d
        object_y = sin(object_angle) * d
        object_centroid = (object_x, object_y)
        return object_centroid
    except BaseException as e:
        print("Exception: %s " % e)

def get_bounding_box_centroid(xmax, xmin, ymax, ymin):
    x_centroid = ((xmax - xmin) / 2) + xmin
    y_centroid = ((ymax - ymin) / 2) + ymin
    return x_centroid, y_centroid

def map_callback(msg):
    global MAP_WIDTH
    global MAP_HEIGHT
    global MAP_ORIGIN
    global MAP_DATA
    global MAP_RESOLUTION
    global MAP_INFO
    global MAP_HEADER

    MAP_INFO = msg.info
    MAP_RESOLUTION = MAP_INFO.resolution
    MAP_WIDTH = MAP_INFO.width
    MAP_HEIGHT = MAP_INFO.height
    MAP_ORIGIN = MAP_INFO.origin.position
    MAP_DATA = msg.data
    MAP_HEADER = msg.header

    print('...................................')
    print("MAP WIDTH: ", MAP_WIDTH)
    print("MAP HEIGHT: ", MAP_HEIGHT)
    print("MAP RESOLUTION: ", MAP_RESOLUTION)
    print("ORIGIN:\n", MAP_ORIGIN)
    print('...................................')

def darknet_callback(msg):
    global DARKNET
    global BOUNDING_BOX_CENTROID

    DARKNET.update({
        'xmin': msg.bounding_boxes[0].xmin,
        'xmax': msg.bounding_boxes[0].xmax,
        'ymin': msg.bounding_boxes[0].ymin,
        'ymax': msg.bounding_boxes[0].ymax,
        'id': msg.bounding_boxes[0].id,
        'class': msg.bounding_boxes[0].Class
    })

    BOUNDING_BOX_CENTROID = get_bounding_box_centroid(
                                            DARKNET['xmax'],
                                            DARKNET['xmin'],
                                            DARKNET['ymax'],
                                            DARKNET['ymin']
                                        )
bridge = CvBridge()

def depth_callback(msg):

    global DEPTH_VALUE
    global PIXEL_COLOR_X
    global PIXEL_COLOR_Y
    global DEPTH_X
    global DEPTH_Y

    if BOUNDING_BOX_CENTROID:
        x,y = BOUNDING_BOX_CENTROID
        DEPTH_X = align_depth_to_color(x, CAMERA_COLOR_WIDTH, CAMERA_DEPTH_WIDTH)
        DEPTH_Y = align_depth_to_color(y, CAMERA_COLOR_HEIGHT, CAMERA_DEPTH_HEIGHT)

        cv_image = bridge.imgmsg_to_cv2(msg, msg.encoding)  # It gets depth information in a 'readable way'

        # # TEST MODE
        # np_img = np.array(cv_image)
        # DEPTH_VALUE = stats.mode(np_img)

        # TEST MEDIAN
        DEPTH_VALUE = np.mean(cv_image) / 1000

        # # Pega o DEPTH do pixel específico
        # if DEPTH_X and DEPTH_Y:
        #     ''' Depth is in meters
        #         Opencv returns image as: (y,x) and not (x,y)
        #     '''
        #     depth = cv_image[DEPTH_Y][DEPTH_X] / 1000
        #     DEPTH_VALUE = depth

def pointcloud_callback(msg):
    # print("msg: ", msg.point_step)
    pass

def odom_callback(msg):

    global CURRENT_POSE
    global PREV_POSE

    if CURRENT_POSE:
        # print("...definind prev_pose")
        PREV_POSE = CURRENT_POSE

    # Posição do robô
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y

    # Pose do robô
    THETA_q = msg.pose.pose.orientation
    THETA_list = [THETA_q.x, THETA_q.y, THETA_q.z, THETA_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (THETA_list)
    # print("\nyaw: %s " % yaw)

    THETA = round(yaw, 4)

    CURRENT_POSE = (X, Y, THETA)

def main():
    global PUBLISHER

    rospy.init_node('detection')

    darknet_topic = '/darknet_ros/bounding_boxes'
    rospy.Subscriber(darknet_topic, BoundingBoxes, darknet_callback)

    odom_topic = '/p3dx3_TESTE/odom'
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    depth_topic = '/camera/depth/image_raw'
    rospy.Subscriber(depth_topic, Image, depth_callback)

    pointclould_topic = '/camera/depth/color/points'
    rospy.Subscriber(pointclould_topic, PointCloud2, pointcloud_callback)

    map_topic = '/map'
    rospy.Subscriber(map_topic, OccupancyGrid, map_callback)

    map_publish_topic = '/map_carol'
    PUBLISHER = rospy.Publisher(map_publish_topic, OccupancyGrid)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        if BOUNDING_BOX_CENTROID and DEPTH_VALUE and CURRENT_POSE and DARKNET:

            # Object bounding box centroid
            x_bounding_box, y_bounding_box = BOUNDING_BOX_CENTROID

            # # Object angle in world, using robot angle  
            x_angle = get_object_angle(CURRENT_POSE[2], x_bounding_box)

            # # Object angle based on robot point of view, tere the center is 0º
            angle_from_robot_vision = get_object_angle_pov_robot(x_angle, CURRENT_POSE[2])

            # # Getting x-axis and y-axis coordinate on map, based on angle and distance (radius)
            object_map_xy = get_world_xy_from_angle(x_angle, DEPTH_VALUE, CURRENT_POSE[0], CURRENT_POSE[1])

            object_xy = (
                    object_map_xy[0] + CURRENT_POSE[0],
                    object_map_xy[1] + CURRENT_POSE[1]
            )

            if MAP_DATA:
                a = transform_coord_odom_top_map(object_xy[0], object_xy[1])
                create_global_map_message(a)

            print("\n\nDARKNET: ", DARKNET)
            print("BOUNDING_BOX_CENTROID: ", BOUNDING_BOX_CENTROID)
            print("ROBOT: ", CURRENT_POSE)
            print("DEPTH: ", DEPTH_VALUE)
            print("ANGLE: ", x_angle)
            print("OBJECT XY: ", object_map_xy)
            print("OBJECT XY + ROBOT: ", object_xy)

            # #  TODO: Sincronizar as leituras e.e
            # check_robot_state()
            # object_match(object_map_xy, DARKNET['class'])

            # # print("angle_from_robot_vision: ", angle_from_robot_vision)
            # # print("OBJECT MAP XY: ", object_map_xy)

        rate.sleep()

if __name__ == '__main__':
    main()
