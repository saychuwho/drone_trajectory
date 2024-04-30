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
matrix_plot(gen_map_matrix, "map")

print("... plotting occupancy matrix ...")
matrix_plot(gen_occupancy_matrix, "occupancy")

print("... plotting grid ...")
grid_plot(grid, xsize, ysize)

print("... generating start, dest ...")
start, dest = start_dest_generate(gen_occupancy_matrix, xsize, ysize, zsize)
print(f"> start {start} \n> dest {dest}")

if start[0] == -1 or dest[0] == -1:
    exit()

print("... calculating path ...")
paths, paths_smooth, p_e_sum, p_s_e_sum = a_star(grid, grid_size, start, dest, False)
paths_theta, paths_smooth_theta, p_e_sum_theta, p_s_e_sum_theta = theta_star(grid, grid_size, start, dest, False)

print("... calculating path with energy constraint ...")
paths_e, paths_smooth_e, p_e_e_sum, p_s_e_e_sum = a_star(grid, grid_size, start, dest, False, True)
paths_e_theta, paths_smooth_e_theta, p_e_e_sum_theta, p_s_e_e_sum_theta = theta_star(grid, grid_size, start, dest, False, True)

print("... energy consumption estimate ...")
print(f"> a-star : {round(p_e_sum,2)} / a-star energy {round(p_e_e_sum,2)} / a-star smooth {round(p_s_e_sum,2)} / a-star energy smooth {round(p_s_e_e_sum,2)}")
print(f"> theta-star : {round(p_e_sum_theta,2)} / theta-star energy {round(p_e_e_sum_theta,2)} / theta-star smooth {round(p_s_e_sum_theta,2)} / theta-star energy smooth {round(p_s_e_e_sum_theta,2)}")

print("... writing path ...")
write_path(paths, "a_star")
write_path(paths_smooth, "smooth_a_star")
write_path(paths_theta, "theta_star")
write_path(paths_smooth_theta, "smooth_theta_star")

print("... writing path with energy constraint ...")
write_path(paths_e, "a_star_energy")
write_path(paths_smooth_e, "a_star_energy_smooth")
write_path(paths_e_theta, "theta_star_energy")
write_path(paths_smooth_e_theta, "theta_star_energy_smooth")

print("... plotting path on map matrix ...")
plot_path(gen_map_matrix, paths, "1_a_star")
plot_path(gen_map_matrix, paths_smooth, "2_a_star_smooth")
plot_path(gen_map_matrix, paths_theta, "3_theta_star")
plot_path(gen_map_matrix, paths_smooth_theta, "4_theta_star_smooth")

print("... writing path with energy constraints ...")
plot_path(gen_map_matrix, paths_e, "1_a_star_energy")
plot_path(gen_map_matrix, paths_smooth_e, "2_a_star_smooth_energy")
plot_path(gen_map_matrix, paths_e_theta, "3_theta_star_energy")
plot_path(gen_map_matrix, paths_smooth_e_theta, "4_theta_star_smooth_energy")
