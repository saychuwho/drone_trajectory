"""grid_env_generator controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor

from map_generator import *
from path_generator import *


# create the Robot instance.
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

root_node = supervisor.getRoot()
children_field = root_node.getField('children')

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

print("... generating obstacles ...")
for obstacle in gen_map:
    name = obstacle[0]
    translation = obstacle[1]
    height = obstacle[2]
    size = obstacle[3]
    children_field.importMFNodeFromString(-1, proto_option_gen(name, translation, height, size))

print("... generating map matrix, occupancy matrix ...")
gen_map_matrix, gen_occupancy_matrix = matrix_gen(gen_map, xsize, ysize)

print("... generating start, dest ...")
start, dest = start_dest_generate(gen_occupancy_matrix, xsize, ysize, zsize)

print("... generating grid ...")
grid = grid_gen(gen_occupancy_matrix, xsize, ysize, zsize)

print("... plotting map matrix ...")
matrix_plot(gen_map_matrix, "map")

print("... plotting occupancy matrix ...")
matrix_plot(gen_occupancy_matrix, "occupancy")

print("... plotting grid ...")
grid_plot(grid, xsize, ysize)

print("... calculating path ...")
paths, paths_smooth = a_star(grid, grid_size, start, dest)
paths_theta, paths_smooth_theta = theta_star(grid, grid_size, start, dest)
# paths_old = a_star_old(grid, grid_size, start, dest)


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