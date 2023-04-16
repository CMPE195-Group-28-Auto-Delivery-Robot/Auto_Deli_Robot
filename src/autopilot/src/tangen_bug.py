#!/usr/bin/env python

import math

from public_algorithm import distance_angle_point_to_line

two_pi, half_pi = 2 * math.pi, math.pi / 3


# 判断两组线段是否相交
# finish
def check_intersection_line(line_1, line_2):
    line_1_start, line_1_end = line_1
    line_2_start, line_2_end = line_2
    line_1_dx, line_1_dy = line_1_end[0] - line_1_start[0], line_1_end[1] - line_1_start[1]
    line_2_dx, line_2_dy = line_2_end[0] - line_2_start[0], line_2_end[1] - line_2_start[1]
    cross_product = line_1_dx * line_2_dy - line_1_dy * line_2_dx
    if cross_product == 0:
        return False
    intersection_1 = ((line_2_start[0] - line_1_start[0]) * line_2_dy - (line_2_start[1] - line_1_start[1]) * line_2_dx) / cross_product
    intersection_2 = ((line_2_start[0] - line_1_start[0]) * line_1_dy - (line_2_start[1] - line_1_start[1]) * line_1_dx) / cross_product
    return (0 <= intersection_1 <= 1) and (0 <= intersection_2 <= 1)


# 计算限制区域的最大距离
# finish
def restricted_area_size(restricted_areas_coordinate):
    x1, y1 = restricted_areas_coordinate[0]
    x2, y2 = restricted_areas_coordinate[1]
    x3, y3 = restricted_areas_coordinate[2]
    x4, y4 = restricted_areas_coordinate[3]
    return max(math.sqrt((x1 - x3) ** 2 + (y1 - y3) ** 2), math.sqrt((x2 - x4) ** 2 + (y2 - y4) ** 2))


# 判断线段和区域是否相交
# finish
def check_intersection_area(line, area):
    for i in range(4):
        if check_intersection_line(line, [area[0][i], area[0][(i + 1) % 4]]):
            to_center_distance = math.sqrt((area[1][0] - line[0][0]) ** 2 + (area[1][1] - line[0][1]) ** 2)
            if to_center_distance < (restricted_area_size(area[0])/2):
                return True
    return False


# 添加死区
# finish
def add_dead_zone(dead_zones, angle):
    dead_zone_start = angle - half_pi
    dead_zone_end = angle + half_pi
    if dead_zone_start <= -math.pi:
        dead_zone_start = dead_zone_start + two_pi
    if dead_zone_end > math.pi:
        dead_zone_end = dead_zone_end - two_pi
    dead_zones.append([dead_zone_start, dead_zone_end])
    return dead_zones


# 合并死区
# finish
def get_dead_zone(curren_coordinate, target_point, to_goal_angel, obstacles, check_depth=20, check_angle=half_pi):
    dead_zones = []
    for temp_obstacle in obstacles:
        distance, angle = distance_angle_point_to_line(curren_coordinate[0], curren_coordinate[1], temp_obstacle[0][0], temp_obstacle[0][1])
        if distance > check_depth:
            continue
        if check_intersection_line([curren_coordinate, target_point], temp_obstacle[0]):
            add_dead_zone(dead_zones, angle)
        else:
            check_near_start = to_goal_angel - check_angle
            check_near_end = to_goal_angel + check_angle
            if check_near_start < angle < check_near_end:
                add_dead_zone(dead_zones, angle)
    return dead_zones


# 获得安全区范围
# finish
def get_safe_zone(dead_zones):
    safe_zone_start, safe_zone_end = 0, 0
    for temp_zone in dead_zones:
        if safe_zone_start == 0 and safe_zone_end == 0:
            safe_zone_start = temp_zone[1]
            safe_zone_end = temp_zone[0] + two_pi
        elif temp_zone[0] > safe_zone_end:
            safe_zone_start = max(safe_zone_start, temp_zone[1])
            safe_zone_end = temp_zone[0]
        elif temp_zone[1] < safe_zone_start:
            safe_zone_start = temp_zone[1]
            safe_zone_end = min(safe_zone_end, temp_zone[0] + two_pi)
        else:
            safe_zone_start = max(safe_zone_start, temp_zone[1])
            safe_zone_end = min(safe_zone_end, temp_zone[0] + two_pi)
    if safe_zone_start == safe_zone_end == 0:
        return None
    return [safe_zone_start % two_pi, safe_zone_end % two_pi]


# 寻找最近点
def closest_angle(angle_1, angle_2, target_angle):
    distance1 = abs(target_angle - angle_1) % two_pi
    distance2 = abs(target_angle - angle_2) % two_pi
    if distance1 > math.pi:
        distance1 = two_pi - distance1
    if distance2 > math.pi:
        distance2 = two_pi - distance2
    if distance1 < distance2:
        return angle_1
    else:
        return angle_2


# 保持贴墙
# finish
def bug_direction(safe_zone, to_goal_angel, prev_slope):
    if prev_slope:
        return closest_angle(safe_zone[0], safe_zone[1], prev_slope)
    return closest_angle(safe_zone[0], safe_zone[1], to_goal_angel)


# 调用接口
# finish
def tangent_bug(curren_coordinate, target_point, obstacles, restricted_areas, slope):
    to_goal_angel = math.atan2(target_point[1] - curren_coordinate[1], target_point[0] - curren_coordinate[0])
    dead_zones = get_dead_zone(curren_coordinate, target_point, to_goal_angel, obstacles)
    if dead_zones:
        safe_zone = get_safe_zone(dead_zones)
        slope = bug_direction(safe_zone, to_goal_angel, slope)
        if slope == 0:
            slope = two_pi
        return slope
    elif restricted_areas:
        if check_intersection_area([curren_coordinate, target_point], restricted_areas[0]):
            return to_goal_angel
    return None

