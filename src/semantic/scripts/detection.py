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

# import numpy as np
# import json


D = None

# Robot
THETA = None
X = None
y = None

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
CAMERA_RAD_ANGLE_PER_PIXEL = CAMERA_RAD_ANGLE / CAMERA_COLOR_WIDTH
CAMERA_DEPTH_WIDTH = 1280
CAMERA_DEPTH_HEIGHT = 720
DEPTH_VALUE = None
DEPTH_X = None
DEPTH_Y = None
PIXEL_COLOR_X = None
PIXEL_COLOR_Y = None

def align_depth_to_color(x, min_dimension, max_dimension):
    """ You have a small image centered on a bigger one.
        You have a pixel on the small image, let's stay, pixel 10.
        You want the corresponding pixel on the bigger image, which could be, 
        for instance, pixel 20.
        That's what it does. :)
    """
    prop = min_dimension / max_dimension
    shift = max_dimension - min_dimension
    xu = (x/prop) + shift
    return int(xu)

def get_object_angle(robo_angle, pixel_x):
    """ 
        Get the angle of a pixel from the image,
        based on the opening angle from camera,
        and based on the angle from the robot - which is the center of the image
    """
    half_camera_angle = CAMERA_RAD_ANGLE / 2
    angle = robo_angle - half_camera_angle + (CAMERA_RAD_ANGLE_PER_PIXEL * pixel_x)
    return angle

def get_world_xy_from_angle(object_angle, d, x_robot, y_robot):
    """ It gets the (x,y) based on angle
        And it sums up the robot coordinates
        In order to return the coordinates based on world coordinates
    """
    try:
        object_x = cos(object_angle) * d
        object_y = sin(object_angle) * d
        object_centroid = (object_x + x_robot, object_y + y_robot)
        # object_centroid = ( -1 * (object_y + y_robot), (object_x + x_robot))
        return object_centroid
    except BaseException as e:
        print("Exception: %s " % e)

def get_bounding_box_centroid(xmax, xmin, ymax, ymin):
    x_centroid = (xmax - xmin) / 2
    y_centroid = (ymax - ymin) / 2
    return x_centroid, y_centroid

def get_object_world_xy():
    xmax = DARKET['xmax']
    xmin = DARKET['xmin']
    ymax = DARKET['ymax']
    ymin = DARKET['ymin']
    _id = DARKET['id']
    _class = DARKET['class']

    return 0

    # print('DARKNET: %s ' % DARKNET)
    # x_pixel, y_pixel = get_bounding_box_centroid(xmax, xmin, ymax, ymin)
    # depth_x_coord = align_depth_to_color(x_pixel, CAMERA_COLOR_WIDTH, CAMERA_DEPTH_WIDTH)
    # depth_x_value = get_depth(depth_x_coord)
    x_object_angle = get_object_angle(robot_angle_x, x_pixel)
    object_centroid_xy = get_world_xy_from_angle(x_object_angle, depth_x_value, robot_angle_x, robot_angle_y)
    return object_centroid_xy

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

    # print("DARKNET: ", DARKNET)

    BOUNDING_BOX_CENTROID = get_bounding_box_centroid(
                                            DARKNET['xmax'],
                                            DARKNET['xmin'],
                                            DARKNET['ymax'],
                                            DARKNET['ymin']
                                        )
    # print('BOUNDING BOX CENTROID: ', BOUNDING_BOX_CENTROID)
    pass

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

        # print("DEPTH X: ", DEPTH_X)
        # print("DEPTH Y: ", DEPTH_Y)

        cv_image = bridge.imgmsg_to_cv2(msg, msg.encoding)  # It gets depth information in a 'readable way'

        if DEPTH_X and DEPTH_Y:
            ''' Depth is in meters
                Opencv returns image as: (y,x) and not (x,y)
            '''
            depth = cv_image[DEPTH_Y][DEPTH_X] / 1000
            DEPTH_VALUE = depth
            # print("DEPTH VALUE: ", depth)

def pointcloud_callback(msg):
    # print("msg: ", msg.point_step)
    pass

def odom_callback(msg):

    global THETA, X, Y

    # Posição do robô
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y

    # Pose do robô
    THETA_q = msg.pose.pose.orientation
    THETA_list = [THETA_q.x, THETA_q.y, THETA_q.z, THETA_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (THETA_list)
    # print("\nyaw: %s " % yaw)

    THETA = round(yaw, 4)

def main():

    rospy.init_node('detection')

    darknet_topic = '/darknet_ros/bounding_boxes'
    rospy.Subscriber(darknet_topic, BoundingBoxes, darknet_callback)

    odom_topic = '/p3dx3_TESTE/odom'
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    depth_topic = '/camera/depth/image_raw'
    rospy.Subscriber(depth_topic, Image, depth_callback)

    pointclould_topic = '/camera/depth/color/points'
    rospy.Subscriber(pointclould_topic, PointCloud2, pointcloud_callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        if BOUNDING_BOX_CENTROID and DEPTH_VALUE:
            # Getting angle
            x_bounding_box, y_bounding_box = BOUNDING_BOX_CENTROID
            x_angle = get_object_angle(THETA, x_bounding_box)
            print("\nX-AXIS ANGLE OF OBJECT: ", x_angle)

            # Getting x-axis and y-axis coordinate on map, based on angle and distance (radius)
            object_map_xy = get_world_xy_from_angle(x_angle, DEPTH_VALUE, X, Y)
            print("OBJECT MAP XY: ", object_map_xy)
            print("ROBOT: %s, %s, %s " % (X,Y, THETA))

        rate.sleep()

if __name__ == '__main__':
    main()
