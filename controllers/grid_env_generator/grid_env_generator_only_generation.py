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

print("... generating start, dest ...")
start, dest = start_dest_generate(gen_occupancy_matrix, xsize, ysize, zsize)
print(f"> start {start} \n> dest {dest}")

print("... generating grid ...")
grid = grid_gen(gen_occupancy_matrix, xsize, ysize, zsize)

print("... plotting map matrix ...")
matrix_plot(gen_map_matrix, "map")

print("... plotting occupancy matrix ...")
matrix_plot(gen_occupancy_matrix, "occupancy")

print("... plotting grid ...")
grid_plot(grid, xsize, ysize)

print("... calculating path ...")
paths, paths_smooth = a_star(grid, grid_size, start, dest, False)
paths_theta, paths_smooth_theta = theta_star(grid, grid_size, start, dest, False)
# paths_old = a_star_old(grid, grid_size, start, dest, False)


print("... writing path ...")
write_path(paths, "a_star")
# write_path(paths_old, "old_a_star")
write_path(paths_smooth, "smooth_a_star")
write_path(paths_theta, "theta_star")
write_path(paths_smooth_theta, "smooth_theta_star")


print("... plotting path on map matrix ...")
plot_path(gen_map_matrix, paths, "a_star")
# plot_path(gen_map_matrix, paths_old, "old_a_star")
plot_path(gen_map_matrix, paths_smooth, "smooth_a_star")
plot_path(gen_map_matrix, paths_theta, "theta_star")
plot_path(gen_map_matrix, paths_smooth_theta, "smooth_theta_star")