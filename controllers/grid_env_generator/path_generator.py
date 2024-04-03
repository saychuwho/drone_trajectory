""" 
pseudo code from this paper is referenced 

paper : Theta*: Any-Angle Path Planning on Grids

- a_star() : algorithm 1
- post_smoothing() : algorithm 2
- theta_star() & update_vertex_theta_star() : algorithm 3

Matlab code from here(https://github.com/danielesartori/3D-grid-path-planning) is referenced

- line_sight_partial_3D() : line_sight_partial_3D.m
    - Converted matlab code into python code
    - I wrote comments same as matlab code, so it won't be hard to match my code with original code
    - Bound of matrix is modified because matlab matrix index starts from 1, and python numpy.array index starts from 0
    - `sight` value don't have 0.5, so it is changed with boolean

"""

from map_generator import *
import math
import heapq
import numpy as np
from datetime import datetime
import os

from energy_model import *

# destination vector
# combination of dz index + dx_dy_index should be used
dx = [-1,-1,-1,0,0,0,1,1,1]
dy = [-1,0,1,-1,0,1,-1,0,1]
dz = [-1, 0, 1]


# define constant of cost function
K_E = 1     # Energy constant 
K_H = 100   # H-value constant
K_G = 100   # G-value constant


def a_star(grid, grid_size, start, dest, is_controller=True, energy_path=False):
    if not is_controller: print("# generating path with a_star")

    is_print = False if is_controller else True

    global dx
    global dy
    global dz

    x_size, y_size, z_size = grid_size

    tan_gamma = 1.75
    x_delta = 1.0
    z_size = int((z_size // (x_delta*tan_gamma)))+1
    valid_grid_size = (x_size, y_size, z_size)
    
    start = grid_translation(start)
    dest = grid_translation(dest)

    open_heap = []
    closed = set()
    g_val = np.full((z_size, x_size, y_size), np.inf)
    parent = {}

    e_val = np.full((z_size, x_size, y_size), np.inf)

    parent[start] = start

    # h_val : heuristic cost : euclidean distance
    h_val = np.full((z_size, x_size, y_size), np.inf)

    # h_val & e_val update
    for z in range(z_size):
        for x in range(x_size):
            for y in range(y_size):
                if grid[z][x][y] == 0:
                    h_val[z][x][y] = __get_euclidean_distance((x,y,z), dest)


    row, col, layer = start
    g_val[layer][row][col] = 0
    if energy_path:
        heapq.heappush(open_heap, (K_G*g_val[layer][row][col]
                    + K_H*h_val[layer][row][col] + K_E*e_val[layer][row][col], start))
    else:
        heapq.heappush(open_heap, (K_G*g_val[layer][row][col]
                    + K_H*h_val[layer][row][col], start))


    path_found = False
    while open_heap:
        v_val, v = heapq.heappop(open_heap)
        # if not is_controller: print(f"## pop {v} with {v_val}")
        row, col, layer = v

        if v == dest:
            path_found = True
            break
        closed.add(v)

        for i in range(len(dz)):
            for j in range(len(dx)):
                adjx = row + dx[j]
                adjy = col + dy[j]
                adjz = layer + dz[i]
                s_prime = (adjx, adjy, adjz)

                if __is_valid(grid, valid_grid_size, closed, s_prime, is_print):
                    __update_vertex_a_star(open_heap, v, s_prime, parent, g_val, h_val, energy_path, e_val)


    # consturcting path
    paths = []
    paths_smooth = []      
    if path_found == False:
        print("> no path found")
        return paths, paths_smooth, 0, 0
    
    path_parent = dest
    paths.append(dest)
    while path_parent != start:
        path_parent = parent[path_parent]
        paths.append(path_parent)

    paths = list(reversed(paths))

    # smooth path
    paths_smooth = __post_smooth_path(grid, valid_grid_size, paths)

    paths_energy_sum = 0
    paths_smooth_energy_sum = 0
    # calculate path energy total
    for i in range(1, len(paths)):
        paths_energy_sum += __get_energy_consumption(paths[i-1], paths[i])
    for i in range(1, len(paths_smooth)):
        paths_smooth_energy_sum += __get_energy_consumption(paths_smooth[i-1], paths_smooth[i])

    return paths, paths_smooth, paths_energy_sum, paths_smooth_energy_sum


def theta_star(grid, grid_size, start, dest, is_controller=True, energy_path=False):
    if not is_controller: print("# generating path with theta_star")

    is_print = False if is_controller else True

    global dx
    global dy
    global dz

    x_size, y_size, z_size = grid_size

    tan_gamma = 1.75
    x_delta = 1.0
    z_size = int((z_size // (x_delta*tan_gamma)))+1
    valid_grid_size = (x_size, y_size, z_size)
    
    start = grid_translation(start)
    dest = grid_translation(dest)

    open_heap = []
    closed = set()
    g_val = np.full((z_size, x_size, y_size), np.inf)
    parent = {}

    e_val = np.full((z_size, x_size, y_size), np.inf)

    parent[start] = start

    # h_val : heuristic cost : euclidean distance
    # h_val update
    h_val = np.full((z_size, x_size, y_size), np.inf)

    for z in range(z_size):
        for x in range(x_size):
            for y in range(y_size):
                if grid[z][x][y] == 0:
                    h_val[z][x][y] = __get_euclidean_distance((x,y,z), dest)
                    e_val[z][x][y] = __get_energy_consumption((x, y, z), dest)


    row, col, layer = start
    g_val[layer][row][col] = 0
    if energy_path:
        heapq.heappush(open_heap, (K_G*g_val[layer][row][col]
                    + K_H*h_val[layer][row][col] + K_E*e_val[layer][row][col], start))
    else:
        heapq.heappush(open_heap, (K_G*g_val[layer][row][col]
                    + K_H*h_val[layer][row][col], start))
    
    path_found = False
    while open_heap:
        v_val, v = heapq.heappop(open_heap)
        # if not is_controller: print(f"## pop {v} with {v_val}")
        row, col, layer = v

        if v == dest:
            path_found = True
            break
        closed.add(v)

        for i in range(len(dz)):
            for j in range(len(dx)):
                adjx = row + dx[j]
                adjy = col + dy[j]
                adjz = layer + dz[i]
                s_prime = (adjx, adjy, adjz)

                if __is_valid(grid, valid_grid_size, closed, s_prime, is_print):
                    __update_vertex_theta_star(open_heap, v, s_prime, parent, g_val, h_val, grid, valid_grid_size, energy_path, e_val)


    # consturcting path
    paths = []
    paths_smooth = []      
    if path_found == False:
        print("> no path found")
        return paths, paths_smooth, 0, 0
    
    path_parent = dest
    paths.append(dest)
    while path_parent != start:
        path_parent = parent[path_parent]
        paths.append(path_parent)

    paths = list(reversed(paths))

    # smooth path
    paths_smooth = __post_smooth_path(grid, valid_grid_size, paths)

    paths_energy_sum = 0
    paths_smooth_energy_sum = 0
    # calculate path energy total
    for i in range(1, len(paths)):
        paths_energy_sum += __get_energy_consumption(paths[i-1], paths[i])
    for i in range(1, len(paths_smooth)):
        paths_smooth_energy_sum += __get_energy_consumption(paths_smooth[i-1], paths_smooth[i])

    return paths, paths_smooth, paths_energy_sum, paths_smooth_energy_sum


""" functions used in a_star & theta_star """

def __get_euclidean_distance(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    return round(math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2))


def __get_energy_consumption(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2

    horizontal_distance = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    vertical_distance = z1 - z2

    V_air_assumption = 1
    V_vert_assumption = 1.75 if vertical_distance > 0 else -1.75
    alpha_assumption = 0.1

    tmp_t_1 = horizontal_distance / V_air_assumption
    tmp_t_2 = vertical_distance / V_vert_assumption

    t = tmp_t_1 if tmp_t_1 > tmp_t_2 else tmp_t_2

    T = thrust(alpha_assumption, V_air_assumption, 0)
    P = P_induced(T, V_vert_assumption) + P_profile(T, V_air_assumption, alpha_assumption) + P_parasite(V_air_assumption)

    return P*t
    

def __is_valid(grid, grid_size, closed, s, is_print=True):
    s_row, s_col, s_layer = s
    x_size, y_size, z_size = grid_size

    # out of bound
    if not (0 <= s_row < x_size and 0 <= s_col < y_size and 0 <= s_layer < z_size):
        return False
    
    # invalid node
    if grid[s_layer][s_row][s_col] != 0:
        return False
    
    # if is_print: print(f"> {s} closed : {s in closed}")

    if s in closed:
        return False
    
    return True


def __update_vertex_a_star(open_heap: list, v, s_prime, parent: list, g_val, h_val, energy_path=False, e_val=None):
    row, col, layer = v
    s_prime_row, s_prime_col, s_prime_layer = s_prime
    
    before_g_val_prime = g_val[s_prime_layer][s_prime_row][s_prime_col]
    h_val_prime = h_val[s_prime_layer][s_prime_row][s_prime_col]
    g_val_v = g_val[layer][row][col]

    e_val_prime = e_val[s_prime_layer][s_prime_row][s_prime_col]

    # __get_euclidean_distance(v, s_prime)

    if (g_val_v + __get_euclidean_distance(v, s_prime) < before_g_val_prime):
        g_val[s_prime_layer][s_prime_row][s_prime_col] = g_val_v + __get_euclidean_distance(v, s_prime)

        parent[s_prime] = v
        
        if (before_g_val_prime + h_val_prime, s_prime) in open_heap:
            open_heap.remove((K_G*before_g_val_prime + K_H*h_val_prime, s_prime))
            heapq.heapify(open_heap)
        
        if not energy_path:
            heapq.heappush(open_heap, (K_G*g_val[s_prime_layer][s_prime_row][s_prime_col]+
                                    K_H*h_val_prime, s_prime))

        else:
            heapq.heappush(open_heap, (K_G*g_val[s_prime_layer][s_prime_row][s_prime_col]+
                                        K_H*h_val_prime + 
                                        K_E*e_val_prime
                                        , s_prime))


def __update_vertex_theta_star(open_heap: list, v, s_prime, parent: list, g_val, h_val, grid, grid_size, energy_path=False, e_val=None):
    # path 2 : replace v with parent[v]
    if __lines_sight_partial_3D(grid, grid_size, parent[v], s_prime):
        __update_vertex_a_star(open_heap, parent[v], s_prime, parent, g_val, h_val, energy_path, e_val)
    # path 1. same as update_vertex_a_star()
    else:
        __update_vertex_a_star(open_heap, v, s_prime, parent, g_val, h_val, energy_path, e_val)


def __lines_sight_partial_3D(grid, grid_size, s_parent, s_prime) -> bool:
    # Size of environment matrix
    x_size, y_size, z_size = grid_size
    
    # Rename
    x1_0, y1_0, z1_0 = s_parent
    x2, y2, z2 = s_prime

    # Distance
    dx = x2 - x1_0
    dy = y2 - y1_0
    dz = z2 - z1_0

    sx = 1
    sy = 1
    if dy<0:
        dy = -dy
        sy = -1
    if dx<0:
        dx = -dx
        sx = -1

    # Angle between height and horizontal trace
    gamma = math.atan2(dz, math.sqrt(dx**2 + dy**2))

    # Initialize
    x1 = x1_0
    y1 = y1_0
    sight = True

    f = 0
    if dy >= dx:
        while y1 != y2:
            f += dx
            if f >= dy and 0 <= y1 + (sy-1)//2 and y1 + (sy-1)//2 < y_size and 0 <= x1 + (sx-1)//2 and x1 + (sx-1)//2 < x_size:
                z = math.floor(z1_0 + math.tan(gamma)*math.sqrt((x1 + (sx-1)//2 - x1_0)**2 + (y1 + (sy-1)//2 - y1_0)**2))
                
                # difference : numpy.array's start index is 0
                if z < 0:
                    z = 0
                elif z >= z_size:
                    z = z_size - 1

                if grid[z][x1+(sx-1)//2][y1+(sy-1)//2] > 0:
                    sight = False
                    return sight
                
                x1 += sx
                f -= dy

            if 0 <= y1 + (sy-1)//2 and y1 + (sy-1)//2 < y_size and 0 <= x1 + (sx-1)//2 and x1 + (sx-1)//2 < x_size:
                z = math.floor(z1_0 + math.tan(gamma)*math.sqrt((x1 + (sx-1)//2 - x1_0)**2 + (y1 + (sy-1)//2 - y1_0)**2))

                if z < 0:
                    z = 0
                elif z >= z_size:
                    z = z_size - 1
                
                if f != 0 and grid[z][x1+(sx-1)//2][y1+(sy-1)//2] > 0:
                    if grid[z][x1+(sx-1)//2][y1+(sy-1)//2] > 0:
                        sight = False
                        return sight
                
                x1 += sx
                f -= dy
            
            if 0 <= y1 + (sy-1)//2 and y1 + (sy-1)//2 < y_size and 0 <= x1 and x1 < x_size:
                z_1=math.floor(z1_0+math.tan(gamma)*math.sqrt((x1-x1_0)**2+(y1+(sy-1)//2-y1_0)**2))
                if z_1 < 0:
                    z_1 = 0
                elif z_1 >= z_size:
                    z_1 = z_size - 1
                
                z_2=math.floor(z1_0+math.tan(gamma)*math.sqrt((x1-1-x1_0)**2+(y1+(sy-1)//2-y1_0)**2))
                if z_2 < 0:
                    z_2 = 0
                elif z_2 >= z_size:
                    z_2 = z_size - 1
                
                if dx==0 and grid[z_1][x1][y1+(sy-1)//2] > 0 and grid[z_2][x1-1][y1+(sy-1)//2]>0:
                     if grid[z_1][x1][y1+(sy-1)//2]==1 and grid[z_2][x1-1][y1+(sy-1)//2]==1:
                        sight = False
                        return sight
            
            y1 += sy
    
    else:
        while x1 != x2:
            f += dy

            if f >= dx and 0 <= y1+(sy-1)//2 and y1+(sy-1)//2 < y_size and 0 <= x1+(sx-1)//2 and x1+(sx-1)//2 < x_size:
                z = math.floor(z1_0+math.tan(gamma)*math.sqrt((x1+(sx-1)//2-x1_0)**2+(y1+(sy-1)//2-y1_0)**2))
                if z < 0:
                    z = 0
                elif z >= z_size:
                    z = z_size - 1

                if grid[z][x1+(sx-1)//2][y1+(sy-1)//2] > 0:
                    if grid[z][x1+(sx-1)//2][y1+(sy-1)//2] == 1:
                        sight = False
                        return sight
                
                y1 += sy
                f -= dx

            if 0 < y1+(sy-1)//2 and y1+(sy-1)//2 < y_size and 0 < x1+(sx-1)//2 and x1+(sx-1)//2 < x_size:
                z=math.floor(z1_0+math.tan(gamma)*math.sqrt((x1+(sx-1)//2-x1_0)**2+(y1+(sy-1)//2-y1_0)**2))
                if z < 0:
                    z = 0
                elif z >= z_size:
                    z = z_size - 1

                if f!=0 and grid[z][x1+(sx-1)//2][y1+(sy-1)//2]>0:
                    if grid[z][x1+(sx-1)//2][y1+(sy-1)//2] == 1:
                        sight = False
                        return sight
                    
            if 1 <= y1 and y1 < y_size and 0 <= x1+(sx-1)//2 and x1+(sx-1)//2 < x_size:
                z_1=math.floor(z1_0+math.tan(gamma)*math.sqrt((x1+(sx-1)//2-x1_0)**2+(y1-y1_0)**2))
                if z_1 < 0:
                    z_1 = 0
                elif z_1 >= z_size:
                    z_1 = z_size - 1
                
                z_2=math.floor(z1_0+math.tan(gamma)*math.sqrt((x1+(sx-1)//2-x1_0)**2+(y1-1-y1_0)**2))
                if z_2 < 0:
                    z_2 = 0
                elif z_2 >= z_size:
                    z_2 = z_size - 1
                
                if dy==0 and grid[z_1][x1+(sx-1)//2][y1]>0 and grid[z_2][x1+(sx-1)//2][y1-1]>0:
                    if grid[z_1][x1+(sx-1)//2][y1]==1 and grid[z_2][x1+(sx-1)//2][y1-1]==1:
                        sight = False
                        return sight
                
            x1 += sx

    return sight


def __post_smooth_path(grid, grid_size, paths: list) -> list:
    smooth_path = []
    path_last_idx = len(paths) - 1
    t_k = paths[0]
    for i in range(path_last_idx - 1):
        s_i = paths[i]
        s_i_1 = paths[i+1]
        if not __lines_sight_partial_3D(grid, grid_size, t_k, s_i_1):
            smooth_path.append(t_k)
            t_k = s_i

    t_k = paths[path_last_idx]
    smooth_path.append(t_k)
    return smooth_path


""" functions write & store path """

def write_path(paths, path_type="default"):
    if not paths:
        print("> no paths to write")
        return

    current_date = datetime.now()
    filename = "path_" + current_date.strftime('%Y%m%d')
    filename += ("_" + path_type)
    filename += ".txt"

    folder_name = "data_" + current_date.strftime('%Y%m%d')
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    filename = os.path.join(folder_name, filename)

    with open(filename, 'w') as f:
        for item in paths:
            f.write(str(grid_translation(item, False))+'\n')


def plot_path(matrix, paths, path_type="default"):
    if not paths:
        print("> no paths to plot")
        return
    
    plt.clf()

    # plot path x, y
    path_x = []
    path_y = []
    for x, y, _ in paths:
        path_x.append(x)
        path_y.append(y)
    
    plt.plot(path_x, path_y, linewidth=7, color='red')

    # plot map matrix
    current_date = datetime.now()
    plt.imshow(matrix, cmap='viridis')
    plt.title('generated path')
    filename = "plot_path_" + current_date.strftime('%Y%m%d')
    filename += ("_" + path_type)

    folder_name = "data_" + current_date.strftime('%Y%m%d')
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    filename = os.path.join(folder_name, filename)


    plt.savefig(filename)