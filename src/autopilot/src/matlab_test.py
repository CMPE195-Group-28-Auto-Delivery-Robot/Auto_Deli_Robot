#!/usr/bin/env python

import matplotlib.pyplot as plt

resolution = 100
size = 5

def all_map(x_map, y_map, x_vector_arr, y_vector_arr, start_point, target_point, next_point, obstacles):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.streamplot(x_map, y_map, x_vector_arr, y_vector_arr)
    ax.quiver(x_map, y_map, x_vector_arr, y_vector_arr)
    ax.quiver(x_map, y_map, x_vector_arr, y_vector_arr)
    if(abs(target_point[0] - start_point[0]) <= resolution/2 and abs(target_point[1] - start_point[1]) <= resolution/2):
        ax.add_patch(plt.Circle(target_point, 1, color='b'))
        ax.annotate("Target", xy=target_point, fontsize=10, ha="center")
    ax.add_patch(plt.Circle(start_point, 1, color='r'))
    ax.annotate("Robot", xy=start_point, fontsize=10, ha="center")
    ax.add_patch(plt.Circle(next_point, 1, color='y'))
    ax.annotate("Next", xy=next_point, fontsize=10, ha="center")
    for temp_obstacle in obstacles:
        x1, y1 = temp_obstacle[0]
        x2, y2 = temp_obstacle[1]
        ax.annotate("Obstacle", xy=temp_obstacle[2], fontsize=10, ha="center")
        plt.plot([x1, x2], [y1, y2], linewidth='5')
    x1, y1 = resolution/2+size, resolution/2+size
    x2, y2 = resolution/2+size, resolution/2-size
    x3, y3 = resolution/2-size, resolution/2-size
    x4, y4 = resolution/2-size, resolution/2+size
    plt.plot([x1, x2, x3, x4, x1], [y1, y2, y3, y4, y1], linewidth='2.5')
    ax.set_title('Test')
    plt.show()