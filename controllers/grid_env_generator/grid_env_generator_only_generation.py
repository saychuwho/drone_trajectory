from map_generator import *
from path_generator import *


# map generation
xsize = 40
ysize = 40
zsize = 10
grid_size = (xsize, ysize, zsize)

# change the density of environment
obstacle_num = 50

# size_list must contain even number
size_list = [2,4,6]

# k_const = [K_G, K_H, K_E]
k_const = [100,100,823]

# coordinate는 (x,y,z) / np.array에서는 (z,x,y)
# start = (0,0,0)
# dest = (30,30,5) # temporary destination

print("... generating random map ...")
gen_map = map_gen(xsize, ysize, zsize, obstacle_num, size_list)

print("... generating map matrix, occupancy matrix ...")
gen_map_matrix, gen_occupancy_matrix = matrix_gen(gen_map, xsize, ysize)

print("... generating grid ...")
grid = grid_gen(gen_occupancy_matrix, xsize, ysize, zsize)

print("... plotting map matrix ...")
matrix_plot(gen_map_matrix, f"map_{obstacle_num}")

print("... plotting occupancy matrix ...")
matrix_plot(gen_occupancy_matrix, f"occupancy_{obstacle_num}")

print("... plotting grid ...")
grid_plot(grid, xsize, ysize, str(obstacle_num))

print("... generating start, dest ...")
start, dest = start_dest_generate(gen_occupancy_matrix, xsize, ysize, zsize)
print(f"> start {start} \n> dest {dest}")

if start[0] == -1 or dest[0] == -1:
    exit()

print("... calculating path ...")
paths, paths_smooth, p_e_sum, p_s_e_sum = a_star(grid, grid_size, start, dest, False, False, k_const)
paths_theta, paths_smooth_theta, p_e_sum_theta, p_s_e_sum_theta = theta_star(grid, grid_size, start, dest, False, False, k_const)

print("... calculating path with energy constraint ...")
paths_e, paths_smooth_e, p_e_e_sum, p_s_e_e_sum = a_star(grid, grid_size, start, dest, False, True, k_const)
paths_e_theta, paths_smooth_e_theta, p_e_e_sum_theta, p_s_e_e_sum_theta = theta_star(grid, grid_size, start, dest, False, True, k_const)

print("... energy consumption estimate ...")
print(f"> a-star : {round(p_e_sum,2)} / a-star energy {round(p_e_e_sum,2)} / a-star smooth {round(p_s_e_sum,2)} / a-star energy smooth {round(p_s_e_e_sum,2)}")
print(f"> theta-star : {round(p_e_sum_theta,2)} / theta-star energy {round(p_e_e_sum_theta,2)} / theta-star smooth {round(p_s_e_sum_theta,2)} / theta-star energy smooth {round(p_s_e_e_sum_theta,2)}")

print("... writing path ...")
write_path(paths, f"a_star_{obstacle_num}")
write_path(paths_smooth, f"smooth_a_star_{obstacle_num}")
write_path(paths_theta, f"theta_star_{obstacle_num}")
write_path(paths_smooth_theta, f"smooth_theta_star_{obstacle_num}")

print("... writing path with energy constraint ...")
write_path(paths_e, f"a_star_energy_{obstacle_num}")
write_path(paths_smooth_e, f"a_star_energy_smooth_{obstacle_num}")
write_path(paths_e_theta, f"theta_star_energy_{obstacle_num}")
write_path(paths_smooth_e_theta, f"theta_star_energy_smooth_{obstacle_num}")

print("... plotting path on map matrix ...")
plot_path(gen_map_matrix, paths, f"1_a_star_{obstacle_num}")
plot_path(gen_map_matrix, paths_smooth, f"2_a_star_smooth_{obstacle_num}")
plot_path(gen_map_matrix, paths_theta, f"3_theta_star_{obstacle_num}")
plot_path(gen_map_matrix, paths_smooth_theta, f"4_theta_star_smooth_{obstacle_num}")

print("... writing path with energy constraints ...")
plot_path(gen_map_matrix, paths_e, f"1_a_star_energy_{obstacle_num}")
plot_path(gen_map_matrix, paths_smooth_e, f"2_a_star_smooth_energy_{obstacle_num}")
plot_path(gen_map_matrix, paths_e_theta, f"3_theta_star_energy_{obstacle_num}")
plot_path(gen_map_matrix, paths_smooth_e_theta, f"4_theta_star_smooth_energy_{obstacle_num}")
