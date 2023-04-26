#!/usr/bin/env python

import math
import numpy as np

from public_algorithm import distance_angle_point_to_line, distance_point_to_point, angle_point_to_point
from loc_weighted_regression import loc_weighted_regression

import matlab_test

# initially attractive and repulsive forces 
attractive_force = 5
repulsive_force = 2 * attractive_force

size = 8
step = 3


# Add endpoint with strong attraction in range and weak attraction overall
# finish
def add_target_point(goal_coordinate, x, y, target_area=size):
    to_goal_distance = distance_point_to_point(x, y, goal_coordinate)
    to_goal_angle = angle_point_to_point(x, y, goal_coordinate)
    if to_goal_distance > target_area:
        x_vector = 4 * attractive_force * target_area * math.cos(to_goal_angle)
        y_vector = 4 * attractive_force * target_area * math.sin(to_goal_angle)
    elif to_goal_distance > 1:
        x_vector = 8 * attractive_force * target_area * math.cos(to_goal_angle)
        y_vector = 8 * attractive_force * target_area * math.sin(to_goal_angle)
    else:
        x_vector = 0
        y_vector = 0
    return x_vector, y_vector, to_goal_angle


# Add starting point with strong rejection in range
# finish
def add_start_point(start_coordinate, x, y, x_vector, y_vector, start_area=size/2):
    to_start_distance = distance_point_to_point(x, y, start_coordinate)
    if to_start_distance > start_area:
        return x_vector, y_vector
    elif to_start_distance > 1:
        to_start_angle = angle_point_to_point(x, y, start_coordinate)
        x_vector -= 4 * repulsive_force * start_area * math.cos(to_start_angle)
        y_vector -= 4 * repulsive_force * start_area * math.sin(to_start_angle)
    return x_vector, y_vector


# Add obstacle line segment with strong repulsion in the range and attraction to the target at the periphery
# finish
def add_obstacle(obstacle_coordinate, obstacle_center_coordinate, to_goal_angle, x, y, x_vector, y_vector, weight=2, surround=size, curve=2*size):
    to_obstacle_distance, to_obstacle_angle = distance_angle_point_to_line(x, y, obstacle_coordinate[0], obstacle_coordinate[1])
    if to_obstacle_distance > 20:
        return x_vector, y_vector
    to_center_distance = distance_point_to_point(x, y, obstacle_center_coordinate)
    total_distance = (10 * to_obstacle_distance + to_center_distance) / 12
    if total_distance > curve + surround:
        return x_vector, y_vector
    elif total_distance > surround:
        to_center_angle = angle_point_to_point(x, y, obstacle_center_coordinate)
        total_angle = math.atan2((math.sin(to_goal_angle) - math.sin(to_center_angle)), (math.cos(to_goal_angle) - math.cos(to_center_angle)))
        x_vector += 12 * weight * attractive_force * (surround + curve - total_distance) * math.cos(total_angle)
        y_vector += 12 * weight * attractive_force * (surround + curve - total_distance) * math.sin(total_angle)
    elif total_distance > 1:
        to_center_angle = angle_point_to_point(x, y, obstacle_center_coordinate)
        total_angle = math.atan2((math.sin(to_obstacle_angle) + math.sin(to_center_angle)), (math.cos(to_obstacle_angle) + math.cos(to_center_angle)))
        x_vector -= 8 * weight * repulsive_force * (surround + curve - total_distance) * math.cos(total_angle)
        y_vector -= 8 * weight * repulsive_force * (surround + curve - total_distance) * math.sin(total_angle)
    return x_vector, y_vector


# Add a low weight zone with weak repulsion in the range and attraction toward the target in the periphery
# finish
def add_restricted_area(restricted_areas_coordinate, restricted_areas_center_coordinate, to_goal_angle, x, y, x_vector, y_vector, surround=size):
    intersections = 0
    for i in range(4):
        x1, y1 = restricted_areas_coordinate[i]
        x2, y2 = restricted_areas_coordinate[(i + 1) % 4]
        if min(y1, y2) <= y < max(y1, y2) and ((y - y1) * (x2 - x1) / (y2 - y1) + x1) >= x:
            intersections += 1
    if intersections % 2 == 1:
        x_vector -= 40 * repulsive_force * math.cos(to_goal_angle + math.pi/4)
        y_vector -= 40 * repulsive_force * math.sin(to_goal_angle - math.pi/4)
    else:
        min_distance = []
        for i in range(4):
            distance, angle = distance_angle_point_to_line(x, y, restricted_areas_coordinate[i], restricted_areas_coordinate[(i + 1) % 4])
            if distance <= surround:
                min_distance.append(distance)
        if min_distance:
            distance = min(min_distance)
            to_center_angle = angle_point_to_point(x, y, restricted_areas_center_coordinate)
            total_angle = math.atan2(math.sin(to_goal_angle) - 4 * math.cos(to_center_angle), math.cos(to_goal_angle) - 4 * math.sin(to_center_angle))
            x_vector += 6 * (surround - distance) * repulsive_force * math.cos(total_angle)
            y_vector += 6 * (surround - distance) * repulsive_force * math.sin(total_angle)
    return x_vector, y_vector


# Add Slope, for tilt gradient
# finish
def add_slope(slope_angle, x_vector, y_vector, force=5):
    x_vector += 100 * attractive_force * force * math.cos(slope_angle)
    y_vector += 100 * attractive_force * force * math.sin(slope_angle)
    return x_vector, y_vector


# Return to next move coordinates
# finish
def next_step(curren_coordinate, target_point, x_vector_arr, y_vector_arr, prev_path, step_length=step, step_depth=step*4):
    path = prev_path
    path.append(curren_coordinate)
    temp_coordinate = curren_coordinate
    repeat_flag = 0
    for step in range(step_depth):
        angle = math.atan2(x_vector_arr[int(path[-1][1]), int(path[-1][0])], y_vector_arr[int(path[-1][1]), int(path[-1][0])])
        temp_coordinate = [temp_coordinate[0] + 2 * math.sin(angle), temp_coordinate[1] + 2 * math.cos(angle)]
        path.append([round(temp_coordinate[0]), round(temp_coordinate[1])])
        if path[-1] in path[-4:-1]:
            repeat_flag += 1
    if path[-1] == target_point or path[-2] == target_point or path[-3] == target_point:
        repeat_flag = 0
    return path[-step_depth:], loc_weighted_regression(path, (step_depth - step_length)), repeat_flag


# Extrapolate the overall vector map
# finish
def get_vector_map(x_map, y_map, resolution, start_point, target_point, obstacles, restricted_areas, slope):
    x_vector_arr = np.zeros_like(x_map)
    y_vector_arr = np.zeros_like(y_map)
    for i in range(resolution):
        for j in range(resolution):
            x_vector_arr[i][j], y_vector_arr[i][j], to_goal_angle = add_target_point(target_point, x_map[i][j], y_map[i][j])
            x_vector_arr[i][j], y_vector_arr[i][j] = add_start_point(start_point, x_map[i][j], y_map[i][j], x_vector_arr[i][j], y_vector_arr[i][j])
            for temp_obstacle in obstacles:
                x_vector_arr[i][j], y_vector_arr[i][j] = add_obstacle((temp_obstacle[0], temp_obstacle[1]), temp_obstacle[2], to_goal_angle, x_map[i][j], y_map[i][j], x_vector_arr[i][j], y_vector_arr[i][j])
            for temp_area in restricted_areas:
                x_vector_arr[i][j], y_vector_arr[i][j] = add_restricted_area(temp_area[0], temp_area[1], to_goal_angle, x_map[i][j], y_map[i][j], x_vector_arr[i][j], y_vector_arr[i][j])
            if slope:
                x_vector_arr[i][j], y_vector_arr[i][j] = add_slope(slope, x_vector_arr[i][j], y_vector_arr[i][j])
    return x_vector_arr, y_vector_arr


# Function interface
# finish
def gradient_descent(x_arr, y_arr, resolution, start_point, target_point, obstacles, restricted_areas, prev_path, slope):
    x_map, y_map = np.meshgrid(x_arr, y_arr)
    x_vector_arr, y_vector_arr = get_vector_map(x_map, y_map, resolution, start_point, target_point, obstacles, restricted_areas, slope)
    save_path, next_coordinate, repeat_flag = next_step(start_point, target_point, x_vector_arr, y_vector_arr, prev_path)
    #matlab_test.all_map(x_map, y_map, x_vector_arr, y_vector_arr, start_point, target_point, next_coordinate, obstacles)
    return save_path, next_coordinate, repeat_flag, slope