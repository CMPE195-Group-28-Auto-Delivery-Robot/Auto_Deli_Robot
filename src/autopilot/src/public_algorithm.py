#!/usr/bin/env python

import math


# 返回点到点的距离
# finish
def distance_point_to_point(x1, y1, point):
    distance = math.sqrt((point[0] - x1) ** 2 + (point[1] - y1) ** 2)
    return distance


# 返回点到点的角度
# finish
def angle_point_to_point(x1, y1, point):
    angle = math.atan2(point[1] - y1, point[0] - x1)
    return angle


# 返回点到线段的距离和角度
# finish
def distance_angle_point_to_line(x, y, line_left, line_right):
    x1, y1 = line_left
    x2, y2 = line_right
    distance_between_x = x2 - x1
    distance_between_y = y2 - y1
    intercept_percentage = ((x - x1) * distance_between_x + (y - y1) * distance_between_y) / (distance_between_x ** 2 + distance_between_y ** 2)
    intercept_percentage = 0 if intercept_percentage < 0 else (1 if intercept_percentage > 1 else intercept_percentage)
    closest_point_x = x1 + intercept_percentage * distance_between_x
    closest_point_y = y1 + intercept_percentage * distance_between_y
    distance = math.sqrt((x - closest_point_x) ** 2 + (y - closest_point_y) ** 2)
    angle = math.atan2(closest_point_y - y, closest_point_x - x)
    return distance, angle