#!/usr/bin/env python

import matplotlib.pyplot as plt

def all_map(x_map, y_map, x_vector_arr, y_vector_arr, start_point, target_point, obstacles):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.streamplot(x_map, y_map, x_vector_arr, y_vector_arr)
    ax.quiver(x_map, y_map, x_vector_arr, y_vector_arr)
    ax.quiver(x_map, y_map, x_vector_arr, y_vector_arr)
    ax.add_patch(plt.Circle(target_point, 1, color='b'))
    ax.annotate("End", xy=target_point, fontsize=10, ha="center")
    ax.add_patch(plt.Circle(start_point, 1, color='r'))
    ax.annotate("Start", xy=start_point, fontsize=10, ha="center")
    for temp_obstacle in obstacles:
        x1, y1 = temp_obstacle[0][0]
        x2, y2 = temp_obstacle[0][1]
        ax.annotate("Obstacle", xy=temp_obstacle[1], fontsize=10, ha="center")
        plt.plot([x1, x2], [y1, y2], linewidth='5')
    ax.set_title('Test')
    plt.show()