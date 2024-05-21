import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os
import random
import math

from collections import deque


# size_list must contain integer with even numberk
def map_gen(x_size, y_size, z_size, obstacle_num, size_list):
    height = [i for i in range(3, z_size - 1)]
    size = [(i,i) for i in size_list]

    # map contains tuple (name, translation, height, size)
    map = []

    for i in range(obstacle_num):
        map.append(('box_{}'.format(i), (random.randint(0, x_size), random.randint(0, y_size)), 
                    random.choice(height), random.choice(size)))

    return map


# BaseBox { translation size name }
def proto_option_gen(name, translation, height, size):
    proto_name_1 = 'BaseBox {'
    proto_name_2 = '}'
    translation_str = ' translation {} {} {}'.format(translation[0], translation[1], height/2)
    size_str = ' size {} {} {}'.format(size[0], size[1], height)
    name_str = ' name "{}"'.format(name)
    ret_str = proto_name_1 + translation_str + size_str + name_str + proto_name_2

    return ret_str


def matrix_gen(map_list, x_size, y_size):
    map_matrix = np.zeros((x_size, y_size))
    occupancy_matrix = np.zeros((x_size, y_size))
    
    for obstacle in map_list:
        obs_translation = obstacle[1]
        obs_size = obstacle[3]
        obs_height = obstacle[2]
        obs_start = (obs_translation[0] - (obs_size[0] // 2), obs_translation[1] - (obs_size[1] // 2))

        # generating map_matrix
        for x in range(obs_start[0], obs_start[0]+obs_size[0]+1):
            if x >= x_size or x < 0:
                continue
            for y in range(obs_start[1], obs_start[1]+obs_size[1]+1):
                if y >= y_size or y < 0:
                    continue
                map_matrix[x][y] = obs_height

        # generating occupancy_matrix
        for x in range(obs_start[0]-1, obs_start[0]+obs_size[0]+2):
            if x >= x_size or x < 0:
                continue
            for y in range(obs_start[1]-1, obs_start[1]+obs_size[1]+2):
                if y >= y_size or y < 0:
                    continue
                occupancy_matrix[x][y] = obs_height + 1

    return map_matrix, occupancy_matrix


def grid_gen(o_matrix, x_size, y_size, z_size):
    tan_gamma = 1.75
    x_delta = 1.0

    grid_z_size = int(z_size // (x_delta*tan_gamma)) + 1

    grid = np.zeros((grid_z_size, x_size, y_size))
    for z in range(int(grid_z_size)):
        for x in range(x_size):
            for y in range(y_size):
                if o_matrix[x][y] > z*tan_gamma:
                    grid[z][x][y] = 1

    return grid


def start_dest_generate(o_matrix, x_size, y_size, z_size):
    # fix the start_z with 1 m
    start_x, start_y, start_z, end_x, end_y, end_z = 0, 0, 1, 0, 0, 0

    # make list with grid points
    grid_points = []
    grid_points_bottom = []
    for x in range(0, x_size-1):
        for y in range(0, y_size-1):
            grid_points_bottom.append((x, y, 1))
            for z in range(1, z_size-1):
                grid_points.append((x, y, z))

    random.shuffle(grid_points)
    random.shuffle(grid_points_bottom)

    grid_points = deque(grid_points)
    grid_points_bottom = deque(grid_points_bottom)

    while grid_points_bottom:
        choice_point = grid_points_bottom.popleft()

        start_x, start_y, _ = choice_point

        if o_matrix[start_x][start_y] > 0:
            continue
        else:
            break

    while grid_points:
        choice_point = grid_points.popleft()
        end_x, end_y, end_z = choice_point

        if start_x == end_x and start_y == end_y and start_z == end_z:
            continue
        elif o_matrix[end_x][end_y] > 0:
            continue
        elif round(math.sqrt((end_x-start_x)**2 + (end_y-start_y)**2 + (end_z-start_z)**2)) < 35:
            continue
        else:
            break

    if grid_points:
        #print("> successed to find start, destination point")
        return (start_x, start_y, start_z), (end_x, end_y, end_z)
    else:
        # print("> failed to find start, destination point")
        return (-1,-1,-1), (-1,-1,-1)

def grid_translation(point, is_reverse=True):
    tan_gamma = 1.75
    x_delta = 1.0
    x,y,z = point
    
    if not is_reverse:
        return x_delta*x, x_delta*y, tan_gamma*z
    else:
        return (int(x // x_delta), int(y // x_delta), int(z // tan_gamma))


def matrix_plot(matrix, m_type):
    plt.clf()
    current_date = datetime.now()
    plt.imshow(matrix, cmap='viridis')
    plt.colorbar()
    plt.title('generated map')

    folder_name = "data_generation_" + current_date.strftime('%Y%m%d')
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    filename = "plot_" + current_date.strftime('%Y%m%d') + "_map"
    if m_type == "occupancy":
        plt.title('generated occupancy matrix')
        filename = "plot_" + current_date.strftime('%Y%m%d') + "_occupancy"
    filename += ".png"

    filename = os.path.join(folder_name, filename)

    plt.savefig(filename)


def grid_plot(grid : np.array, x_size : int, y_size : int):
    plt.clf()
    current_date = datetime.now()
    filename = "plot_" + current_date.strftime('%Y%m%d') + "_grid.png"
    fig, axs = plt.subplots(grid.shape[0], 1, figsize=(x_size, y_size))

    folder_name = "data_generation_" + current_date.strftime('%Y%m%d')
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    filename = os.path.join(folder_name, filename)

    for i in range(grid.shape[0]):
        axs[i].imshow(grid[i], cmap='viridis')
        axs[i].set_title(f'Layer {i}')
        axs[i].axis('off')

    plt.savefig(filename)