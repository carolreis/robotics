#! /usr/bin/env python3
def transform_coord_odom_top_map(x, y):
    map_resolution = 0.05000000074505806
    map_origin_y = -100
    map_origin_x = -100

    j = i = y / map_resolution - map_origin_y / map_resolution
    i = x / map_resolution - map_origin_x / map_resolution
    return (i, j)

a = transform_coord_odom_top_map(0,0)

print("a: ", a)