import numpy as np
from scipy import linalg

# 0.1 to 0.9
tau_var = 0.8
# 1 or 2
degree_var = 1

# Local weighted linear regression
# finish
def regression_algorithm(x, y, index, tau=tau_var, degree=1):
    size = len(x)
    nearest_val = int(np.floor(tau * size))
    distances = np.abs(x - x[index])
    weights = np.zeros(size)
    nearest_idx = distances <= np.sort(distances)[nearest_val]
    weights[nearest_idx] = (1 - (distances[nearest_idx] / np.sort(distances)[nearest_val]) ** 2) ** 2
    x_local = x[distances <= np.sort(distances)[nearest_val]]
    y_local = y[distances <= np.sort(distances)[nearest_val]]
    design_matrix = np.vander(x_local, degree + 1)
    weight_matrix = np.diag(weights[distances <= np.sort(distances)[nearest_val]])
    regression_coefficient = linalg.solve(np.dot(np.dot(design_matrix.T, weight_matrix), design_matrix), np.dot(np.dot(design_matrix.T, weight_matrix), y_local))
    return np.dot(regression_coefficient, np.vander(np.array([x[index]]), degree + 1)[0])


# Regression interface
# finish
def loc_weighted_regression(input_path, step_length):
    length = len(input_path)
    arr = np.array([i for i in range(0, length)])
    x = np.array([path[0] for path in input_path])
    y = np.array([path[1] for path in input_path])
    x_fix = regression_algorithm(arr, x, (length - step_length))
    y_fix = regression_algorithm(arr, y, (length - step_length))
    return [x_fix, y_fix]