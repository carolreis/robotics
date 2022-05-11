#! /usr/bin/env python3
from math import cos, sin, pi

object_angle = 1.3810279181956848
d = 3.2752903602430554

# object_angle = 1.625272209780129
# d = 2.646096216362847

x = cos(object_angle) * d
y = sin(object_angle) * d

print(x, ' ', y)
# object_centroid = (object_x + x_robot, object_y + y_robot)