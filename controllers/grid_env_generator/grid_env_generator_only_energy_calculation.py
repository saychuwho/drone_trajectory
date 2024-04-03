from map_generator import *
from path_generator import *


# map generation
xsize = 40
ysize = 40
zsize = 10
grid_size = (xsize, ysize, zsize)
obstacle_num = 30

# size_list must contain even number
size_list = [2,4,6]

print("##### energy consumption calculation #####")

a_star_delta = []
a_star_smooth_delta = []
theta_star_delta = []
theta_star_smooth_delta = []

for i in range(100):
    print(f"\n$$$ iter : {i}")
    
    print("... generating random map ...")
    gen_map = map_gen(xsize, ysize, zsize, obstacle_num, size_list)

    print("... generating map matrix, occupancy matrix ...")
    gen_map_matrix, gen_occupancy_matrix = matrix_gen(gen_map, xsize, ysize)

    print("... generating grid ...")
    grid = grid_gen(gen_occupancy_matrix, xsize, ysize, zsize)

    print("... generating start, dest ...")
    start, dest = start_dest_generate(gen_occupancy_matrix, xsize, ysize, zsize)
    print(f"> start {start} \n> dest {dest}")

    if start[0] == -1 or dest[0] == -1:
        continue

    print("... calculating path ...")
    paths, paths_smooth, p_e_sum, p_s_e_sum = a_star(grid, grid_size, start, dest, False)
    paths_theta, paths_smooth_theta, p_e_sum_theta, p_s_e_sum_theta = theta_star(grid, grid_size, start, dest, False)

    print("... calculating path with energy constraint ...")
    paths_e, paths_smooth_e, p_e_e_sum, p_s_e_e_sum = a_star(grid, grid_size, start, dest, False, True)
    paths_e_theta, paths_smooth_e_theta, p_e_e_sum_theta, p_s_e_e_sum_theta = theta_star(grid, grid_size, start, dest, False, True)

    # if energy calculation is 0, we should not include this data
    if (p_e_sum == 0 or p_s_e_sum == 0 or p_e_sum_theta == 0 or p_s_e_sum_theta == 0 or 
        p_e_e_sum == 0 or p_s_e_e_sum == 0 or p_e_e_sum_theta == 0 or p_s_e_e_sum_theta == 0):
        continue
    else:
        a_star_delta.append(p_e_sum - p_e_e_sum)
        a_star_smooth_delta.append(p_s_e_sum - p_s_e_e_sum)
        theta_star_delta.append(p_e_sum_theta - p_e_e_sum_theta)
        theta_star_smooth_delta.append(p_s_e_sum_theta - p_s_e_e_sum_theta)

print("\n\naverage:")
print(f"\ta_star : {sum(a_star_delta)/len(a_star_delta)}")
print(f"\ta_star_smooth : {sum(a_star_smooth_delta)/len(a_star_smooth_delta)}")
print(f"\ttheta_star : {sum(theta_star_delta)/len(theta_star_delta)}")
print(f"\ttheta_star_smooth : {sum(theta_star_smooth_delta)/len(theta_star_smooth_delta)}")

a_star_delta.sort()
a_star_smooth_delta.sort()
theta_star_delta.sort()
theta_star_smooth_delta.sort()

print("\nmedian:")
print(f"\ta_star : {a_star_delta[len(a_star_delta)//2]}")
print(f"\ta_star_smooth : {a_star_smooth_delta[len(a_star_smooth_delta)//2]}")
print(f"\ttheta_star : {theta_star_delta[len(theta_star_delta)//2]}")
print(f"\ttheta_star_smooth : {theta_star_smooth_delta[len(theta_star_smooth_delta)//2]}")