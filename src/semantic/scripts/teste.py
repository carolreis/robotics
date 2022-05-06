PI_180 = 0.01745329251994329577
CAMERA_DEG_ANGLE = 69.4
CAMERA_RAD_ANGLE = CAMERA_DEG_ANGLE * PI_180
CAMERA_COLOR_WIDTH = 640
CAMERA_COLOR_HEIGHT = 480
CAMERA_RAD_ANGLE_PER_PIXEL = CAMERA_RAD_ANGLE / CAMERA_COLOR_WIDTH
CAMERA_DEPTH_WIDTH = 1280
CAMERA_DEPTH_HEIGHT = 720

from math import sin, cos, atan, pi, acos, sqrt, atan2


class PixelWorldCoord:

    def get_angle(robo_angle, pixel_x):
        """ 
            Get the angle of a pixel from the image,
            based on the opening angle from camera,
            and based on the angle from the robot - which is the center of the image
        """
        half_camera_angle = CAMERA_RAD_ANGLE / 2
        angle = robo_angle - half_camera_angle + (CAMERA_RAD_ANGLE_PER_PIXEL * pixel_x)
        return angle

    def get_world_xy_from_angle(initial_x, initial_y, angle, d):
        print('d: %s ' % d)
        print('initial x: %s' % initial_x)
        print('initial y: %s' % initial_y)
        x = initial_x + (cos(angle) * d)
        y = initial_y + (sin(angle) * d)

        return (x,y)

    def camera_coord_color_to_depth():
        # CAMERA_COLOR_WIDTH
        # CAMERA_DEPTH_WIDTH
        return 400

    def get_depth(x_pixel):
        # TODO: retirar
        return 2

pwc = PixelWorldCoord()

robo_angle = pi/2
x_pixel = CAMERA_COLOR_WIDTH / 2 # TESTE com meio da imagem
angle = pwc.get_angle(robo_angle, x_pixel)

print("camera angle: %s " % CAMERA_RAD_ANGLE)
print('robo angle: %s ' % robo_angle)
print('angle: %s ' % angle)

robo_x = 2
robo_y = 3
depth = pwc.get_depth(x_pixel)

world_xy = pwc.get_world_xy_from_angle(robo_x, robo_y, angle, depth)
print('world xy: ', world_xy)
