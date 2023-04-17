#!/usr/bin/env python

import numpy as np


# Local weighted linear regression
# finish
def loc_weighted_regression(input_path, step_length, tau=2):
    length = len(input_path)
    arr = np.mat(np.array([i for i in range(1, length + 1)]))
    x_matrix = np.mat(np.array([i[0] for i in input_path]))
    y_matrix = np.mat(np.array([i[1] for i in input_path]))
    slope = np.shape(arr)[1]
    weights = np.mat(np.eye(slope))
    for j in range(slope):
        diff_matrix = (length - step_length) - arr[0, j]
        weights[j, j] = np.exp(diff_matrix ** 2 / (-2.0 * tau ** 2))
    gram_matrix = arr * (weights * arr.T)
    regression_coefficient_x = gram_matrix.I * (arr * (weights * x_matrix.T))
    regression_coefficient_y = gram_matrix.I * (arr * (weights * y_matrix.T))
    return [int((length - step_length) * regression_coefficient_x), int((length - step_length) * regression_coefficient_y)]
