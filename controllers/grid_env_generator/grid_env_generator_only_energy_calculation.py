from map_generator import *
from path_generator import *

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os

from matplotlib import cm
from matplotlib.ticker import LinearLocator

# map generation
xsize = 40
ysize = 40
zsize = 10
grid_size = (xsize, ysize, zsize)
obstacle_num = 30

# size_list must contain even number
size_list = [2,4,6]

def energy_calculation(k_const=None):
    print("##### energy consumption calculation #####")

    a_star_delta = []
    a_star_smooth_delta = []
    theta_star_delta = []
    theta_star_smooth_delta = []

    for i in range(100):
        print(f"\n$$$ iter : {i}")
        
        #print("... generating random map ...")
        gen_map = map_gen(xsize, ysize, zsize, obstacle_num, size_list)

        #print("... generating map matrix, occupancy matrix ...")
        gen_map_matrix, gen_occupancy_matrix = matrix_gen(gen_map, xsize, ysize)

        #print("... generating grid ...")
        grid = grid_gen(gen_occupancy_matrix, xsize, ysize, zsize)

        #print("... generating start, dest ...")
        start, dest = start_dest_generate(gen_occupancy_matrix, xsize, ysize, zsize)
        #print(f"> start {start} \n> dest {dest}")

        if start[0] == -1 or dest[0] == -1:
            continue

        #print("... calculating path ...")
        paths, paths_smooth, p_e_sum, p_s_e_sum = a_star(grid, grid_size, start, dest, False, False, k_const)
        paths_theta, paths_smooth_theta, p_e_sum_theta, p_s_e_sum_theta = theta_star(grid, grid_size, start, dest, False, False, k_const)

        #print("... calculating path with energy constraint ...")
        paths_e, paths_smooth_e, p_e_e_sum, p_s_e_e_sum = a_star(grid, grid_size, start, dest, False, True, k_const)
        paths_e_theta, paths_smooth_e_theta, p_e_e_sum_theta, p_s_e_e_sum_theta = theta_star(grid, grid_size, start, dest, False, True, k_const)

        # if energy calculation is 0, we should not include this data
        if (p_e_sum == 0 or p_s_e_sum == 0 or p_e_sum_theta == 0 or p_s_e_sum_theta == 0 or 
            p_e_e_sum == 0 or p_s_e_e_sum == 0 or p_e_e_sum_theta == 0 or p_s_e_e_sum_theta == 0):
            continue
        else:
            a_star_delta.append(p_e_sum - p_e_e_sum)
            a_star_smooth_delta.append(p_s_e_sum - p_s_e_e_sum)
            theta_star_delta.append(p_e_sum_theta - p_e_e_sum_theta)
            theta_star_smooth_delta.append(p_s_e_sum_theta - p_s_e_e_sum_theta)


    avg_a_star = round(sum(a_star_delta)/len(a_star_delta),2)
    avg_a_star_smooth = round(sum(a_star_smooth_delta)/len(a_star_smooth_delta),2)
    avg_theta_star = round(sum(theta_star_delta)/len(theta_star_delta),2)
    avg_theta_star_smooth = round(sum(theta_star_smooth_delta)/len(theta_star_smooth_delta),2)

    a_star_delta.sort()
    a_star_smooth_delta.sort()
    theta_star_delta.sort()
    theta_star_smooth_delta.sort()

    med_a_star = round(a_star_delta[len(a_star_delta)//2],2)
    med_a_star_smooth = round(a_star_smooth_delta[len(a_star_smooth_delta)//2],2)
    med_theta_star = round(theta_star_delta[len(theta_star_delta)//2],2)
    med_theta_star_smooth = round(theta_star_smooth_delta[len(theta_star_smooth_delta)//2],2)

    print("\n\naverage:")
    print(f"\ta_star : {avg_a_star}")
    print(f"\ta_star_smooth : {avg_a_star_smooth}")
    print(f"\ttheta_star : {avg_theta_star}")
    print(f"\ttheta_star_smooth : {avg_theta_star_smooth}")

    print("\nmedian:")
    print(f"\ta_star : {med_a_star}")
    print(f"\ta_star_smooth : {med_a_star_smooth}")
    print(f"\ttheta_star : {med_theta_star}")
    print(f"\ttheta_star_smooth : {med_theta_star_smooth}")

    return (avg_a_star, avg_a_star_smooth, avg_theta_star, avg_theta_star_smooth, med_a_star, med_a_star_smooth, med_theta_star, med_theta_star_smooth)

def save_csv(data, name=""):
    current_date = datetime.now()
    filename2 = current_date.strftime('%Y%m%d') + f"_{name}.csv"
    folder_name = "data_3_" + current_date.strftime('%Y%m%d')
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    filename2 = os.path.join(folder_name, filename2)

    np.savetxt(filename2, data, delimiter=',')


def plot_data(data, name=""):
    plt.clf()
    current_date = datetime.now()
    filename = current_date.strftime('%Y%m%d') + f"_{name}.png"
    folder_name = "data_3_" + current_date.strftime('%Y%m%d')
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    filename = os.path.join(folder_name, filename)

    """ code from ChatGPT """
    # Create meshgrid for x, y coordinates
    x = np.arange(data.shape[0])
    y = np.arange(data.shape[1])
    xx, yy = np.meshgrid(x, y)

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    # Plot points
    surf = ax.plot_surface(xx, yy, data, cmap=cm.coolwarm, linewidth=0, antialiased=False)

    fig.colorbar(surf, shrink=0.5, aspect=5)

    # Set labels and title
    ax.set_xlabel('K_g')
    ax.set_ylabel('K_h')
    ax.set_zlabel('E diff')
    ax.set_title(name)

    plt.savefig(filename)
    


""" make datas and plotting here """

avg_a_star = np.zeros((10,10))
avg_a_star_smooth = np.zeros((10,10))
avg_theta_star = np.zeros((10,10))
avg_theta_star_smooth = np.zeros((10,10))

med_a_star = np.zeros((10,10))
med_a_star_smooth = np.zeros((10,10))
med_theta_star = np.zeros((10,10))
med_theta_star_smooth = np.zeros((10,10))

"""
for k_e in range(1, 101, 10):
    for k_g in range(10, 1001, 10):
        for k_h in range(10, 1001, 10):
            k_constant = (k_g, k_h, k_e)
            
            result = energy_calculation(k_constant)
            avg_a_star[k_g][k_h] = result[0]
            avg_a_star_smooth[k_g][k_h] = result[1]
            avg_theta_star[k_g][k_h] = result[2]
            avg_theta_star_smooth[k_g][k_h] = result[3]

            med_a_star[k_g][k_h] = result[4]
            med_a_star_smooth[k_g][k_h] = result[5]
            med_theta_star[k_g][k_h] = result[6]
            med_theta_star_smooth[k_g][k_h] = result[7]
"""


# test code
for k_g in range(100, 1001, 100):
    for k_h in range(100, 1001, 100):
        k_constant = (k_g, k_h, 4)

        # os.system('cls')
        os.system('clear')
        print(f"## k_g : {k_g} k_h : {k_h} k_e : {4}")
        
        result = energy_calculation(k_constant)
        avg_a_star[k_g//100-1][k_h//100-1] = result[0]
        avg_a_star_smooth[k_g//100-1][k_h//100-1] = result[1]
        avg_theta_star[k_g//100-1][k_h//100-1] = result[2]
        avg_theta_star_smooth[k_g//100-1][k_h//100-1] = result[3]

        med_a_star[k_g//100-1][k_h//100-1] = result[4]
        med_a_star_smooth[k_g//100-1][k_h//100-1] = result[5]
        med_theta_star[k_g//100-1][k_h//100-1] = result[6]
        med_theta_star_smooth[k_g//100-1][k_h//100-1] = result[7]

save_csv(avg_a_star, "avg_a_star_4")
save_csv(avg_a_star_smooth, "avg_a_star_smooth_4")
save_csv(avg_theta_star, "avg_theta_star_4")
save_csv(avg_theta_star_smooth, "avg_theta_star_smooth_4")
save_csv(med_a_star, "med_a_star_4")
save_csv(med_a_star_smooth, "med_a_star_smooth_4")
save_csv(med_theta_star, "med_theta_star_4")
save_csv(med_theta_star_smooth, "med_theta_star_smooth_4")

plot_data(avg_a_star, "avg_a_star_4")
plot_data(avg_a_star_smooth, "avg_a_star_smooth_4")
plot_data(avg_theta_star, "avg_theta_star_4")
plot_data(avg_theta_star_smooth, "avg_theta_star_smooth_4")
plot_data(med_a_star, "med_a_star_4")
plot_data(med_a_star_smooth, "med_a_star_smooth_4")
plot_data(med_theta_star, "med_theta_star_4")
plot_data(med_theta_star_smooth, "med_theta_star_smooth_4")




# result = []
# for k_e in range(1, 11):
    
#     k_constant = (1000, 1000, k_e)
#     os.system('clear')
#     print(f"## k_g : {1000} k_h : {1000} k_e : {k_e}")
#     calculation = energy_calculation(k_constant)
#     result.append(calculation)

# result = np.array(result)

# current_date = datetime.now()
# filename2 = current_date.strftime('%Y%m%d') + "_1000_1000_1_10.csv"
# folder_name = "data_" + current_date.strftime('%Y%m%d')
# if not os.path.exists(folder_name):
#     os.makedirs(folder_name)
# filename2 = os.path.join(folder_name, filename2)
# np.savetxt(filename2, result, delimiter=',')
