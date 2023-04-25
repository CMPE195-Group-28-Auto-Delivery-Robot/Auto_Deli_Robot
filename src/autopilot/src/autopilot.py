#!/usr/bin/env python

import random
import numpy as np

import gradient_descent
import tangen_bug

resolution = 100
zoom = 20
merge = 4

def obs_coordinate_fix(fix, coordinate):
    coordinate = list(coordinate)
    coordinate[0] *= zoom
    coordinate[1] *= zoom
    coordinate[0] += resolution / 2
    coordinate[1] += resolution / 2
    return tuple(coordinate)


def obs_coordinates_fix(fix, coordinates):
    for i in range(len(coordinates)):
        coordinate_list = list(coordinates[i])
        coordinate_list[0] = obs_coordinate_fix(fix, coordinate_list[0])
        coordinate_list[1] = obs_coordinate_fix(fix, coordinate_list[1])
        coordinate_list[2] = obs_coordinate_fix(fix, coordinate_list[2])
        coordinates[i] = tuple(coordinate_list)
    return coordinates


def target_coordinate_fix(fix, coordinate):
    coordinate = list(coordinate)
    coordinate[0] -= fix[0]
    coordinate[1] -= fix[1]
    coordinate[0] *= zoom
    coordinate[1] *= zoom
    coordinate[0] += resolution / 2
    coordinate[1] += resolution / 2
    return tuple(coordinate)


def undo_coordinate_fix(fix, coordinate):
    coordinate[0] -= resolution / 2
    coordinate[1] -= resolution / 2
    coordinate[0] = float(coordinate[0]/zoom)
    coordinate[1] = float(coordinate[1]/zoom)
    coordinate[0] += fix[0]
    coordinate[1] += fix[1]
    return coordinate


class autopilot:

    def __init__(self):
        self.x_arr = np.arange(0, resolution, 1)
        self.y_arr = np.arange(0, resolution, 1)
        self.slope = None
        self.path = []
        self.save_path = []
        self.start_point = [resolution / 2, resolution / 2]
        self.repeat_time = 0

    def start(self):
        self.slope = None
        self.repeat_time = 0

    def get_next(self, obstacles, restricted_areas, curren_point, target_point):
        print("curren_point: ")
        print(curren_point)
        print("target_point: ")
        print(target_point)
        target_point = target_coordinate_fix(curren_point, target_point)
        obstacles = obs_coordinates_fix(curren_point, obstacles)
        restricted_areas = obs_coordinates_fix(curren_point, restricted_areas)
        self.save_path, next_point, repeat_flag, self.slope = gradient_descent.gradient_descent(self.x_arr, self.y_arr, resolution, self.start_point, target_point, obstacles, restricted_areas, self.save_path, self.slope)
        if abs(next_point[0] - target_point[0]) <= merge and abs(next_point[1] - target_point[1]) <= merge:
            return None, True
        if self.slope is not None:
            if repeat_flag > 2:
                if self.repeat_time > 2:
                    return None, False
                next_point[0] += random.randint(-1, 1)
                next_point[1] += random.randint(-1, 1)
                print("error: jump")
                self.repeat_time += 1
            else:
                self.slope = tangen_bug.tangent_bug(self.start_point, target_point, obstacles, restricted_areas, self.slope)
                print("tangent_bug: slope")
                print(self.slope)
        elif repeat_flag > 1:
            self.slope = tangen_bug.tangent_bug(self.start_point, target_point, obstacles, restricted_areas, self.slope)
            print("tangent_bug: slope")
            print(self.slope)
        return undo_coordinate_fix(curren_point, next_point), False