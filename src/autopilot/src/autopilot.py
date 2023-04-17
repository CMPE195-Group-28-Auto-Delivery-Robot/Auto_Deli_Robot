#!/usr/bin/env python

import random
import numpy as np

import gradient_descent

resolution = 100
zoom_rate = 10000000

def coordinate_fix(coordinate):
    coordinate[0] *= zoom_rate
    coordinate[1] *= zoom_rate
    coordinate[0] += resolution / 2
    coordinate[1] += resolution / 2
    return coordinate


def coordinates_fix(coordinates):
    for i in range(len(coordinates)):
        coordinates[i] = coordinate_fix(coordinates[i])
    return coordinates


def undo_coordinate_fix(coordinate):
    coordinate[0] /= zoom_rate
    coordinate[1] /= zoom_rate
    coordinate[0] -= resolution / 2
    coordinate[1] -= resolution / 2
    return coordinate


class autopilot:

    def __init__(self):
        self.x_arr = np.arange(0, resolution, 1)
        self.y_arr = np.arange(0, resolution, 1)
        self.slope = None
        self.path = []
        self.save_path = []
        self.start_point = [50, 50]
        self.repeat_time = 0

    def start(self):
        self.slope = None
        self.repeat_time = 0

    def get_next(self, obstacles, restricted_areas, step_length, target_point, merge=5):
        target_point = coordinate_fix(target_point)
        obstacles = coordinates_fix(obstacles)
        restricted_areas = coordinates_fix(restricted_areas)
        self.save_path, next_coordinate, repeat_flag, self.slope = gradient_descent.gradient_descent(self.x_arr, self.y_arr, resolution, self.start_point, target_point, obstacles, restricted_areas, self.save_path, self.slope, step_length)
        if next_coordinate == target_point:
            return None, True
        elif abs(next_coordinate[0] - target_point[0]) <= merge and abs(next_coordinate[1] - target_point[1]) <= merge:
            return None, True
        #if slope is not None:
        else:
            if repeat_flag > 2:
                if self.repeat_time > 2:
                    return None, False
                next_coordinate[0] += random.randint(-1, 1)
                next_coordinate[1] += random.randint(-1, 1)
                print("error: jump")
                self.repeat_time += 1
            #else:
                #slope = ObstacleAvoidance.tangent_bug(curren_coordinate, target_point, obstacles, restricted_areas, slope)
                #print("tangent_bug:slope")
        #elif repeat_flag > 1:
            #slope = ObstacleAvoidance.tangent_bug(curren_coordinate, target_point, obstacles, restricted_areas, slope)
            #print("tangent_bug:slope")
        return undo_coordinate_fix(next_coordinate), False