#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, atan, pi, acos, sqrt, atan2
from cv_bridge import CvBridge, CvBridgeError

# from math import sin, cos, atan, pi, acos, sqrt, atan2
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import numpy as np
# import json


THETA = None
D = None
X = None
y = None

PI_180 = 0.01745329251994329577
CAMERA_DEG_ANGLE = 69.4
CAMERA_RAD_ANGLE = CAMERA_DEG_ANGLE * PI_180
CAMERA_COLOR_WIDTH = 640
CAMERA_COLOR_HEIGHT = 480
CAMERA_RAD_ANGLE_PER_PIXEL = CAMERA_RAD_ANGLE / CAMERA_COLOR_WIDTH
CAMERA_DEPTH_WIDTH = 1280
CAMERA_DEPTH_HEIGHT = 720

def get_angle(robo_angle, pixel_x):
    """ 
        Get the angle of a pixel from the image,
        based on the opening angle from camera,
        and based on the angle from the robot - which is the center of the image
    """
    half_camera_angle = CAMERA_RAD_ANGLE / 2
    angle = robo_angle - half_camera_angle + (CAMERA_RAD_ANGLE_PER_PIXEL * pixel_x)
    return angle

def camera_coord_color_to_depth():
    # CAMERA_COLOR_WIDTH
    # CAMERA_DEPTH_WIDTH
    return 400

def get_depth(x_pixel):
    # TODO: retirar
    return 2

def get_object_centroid(theta, d, x, y):
    try:
        b = cos(theta) * d
        a = sin(theta) * d
        centroid = (a+x, b+y)
    except BaseException as e:
        print("Exception: %s " % e)

def darknet_callback(msg):
    # print('\nmsg x_min: %s ' % msg.bounding_boxes[0].xmin)
    # print('msg y_min: %s ' % msg.bounding_boxes[0].ymin)
    # print('msg x_max: %s ' % msg.bounding_boxes[0].xmax)
    # print('msg y_max: %s ' % msg.bounding_boxes[0].ymax)
    # print('msg id: %s ' % msg.bounding_boxes[0].id)
    # print('msg class: %s ' % msg.bounding_boxes[0].Class)
    pass

bridge = CvBridge()

def depth_callback(msg):

    cv_image = bridge.imgmsg_to_cv2(msg, msg.encoding)
    pix = (msg.width/2, msg.height/2)
    # In meters
    _x = int(pix[0])
    _y = int(pix[1])
    depth = cv_image[_x][_y] / 1000
    print('depth: ', depth)
    # print('MESSAGE DEPTH: ', type(msg.data))
    # print('MESSAGE DEPTH: ',  "".join(map(chr, msg.data)))
    pass

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

    THETA = yaw

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
        rate.sleep()

if __name__ == '__main__':
    main()
