"""grid_env_generator controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor

from map_generator import *
from path_generator import *
from energy_model import *

from datetime import datetime

# create the Robot instance.
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

root_node = supervisor.getRoot()
children_field = root_node.getField('children')

""" Environment Generation """

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

print("... place drone at start point ...")
drone = supervisor.getFromDef("DRONE")
drone_translation_field = drone.getField('translation')
init_drone_position = [start[0], start[1], start[2]]  # `start` is tuple. so we need to change it into `List`
drone_translation_field.setSFVec3f(init_drone_position)

viewpoint = supervisor.getFromDef("VIEWPOINT")
viewpoint_position_field = viewpoint.getField('position')
init_viewpoint_position = [-3 + start[0], 0 + start[1], 1.5 + start[2]]
viewpoint_position_field.setSFVec3f(init_viewpoint_position)

print("... generating grid ...")
grid = grid_gen(gen_occupancy_matrix, xsize, ysize, zsize)

print("... plotting map matrix ...")
matrix_plot(gen_map_matrix, "map")

print("... plotting occupancy matrix ...")
matrix_plot(gen_occupancy_matrix, "occupancy")

print("... plotting grid ...")
grid_plot(grid, xsize, ysize)

print("... calculating path ...")
paths, paths_smooth, p_e_sum, p_s_e_sum = a_star(grid, grid_size, start, dest)
paths_theta, paths_smooth_theta, p_e_sum_theta, p_s_e_sum_theta = theta_star(grid, grid_size, start, dest)

print("... calculating path with energy constraint ...")
paths_e, paths_smooth_e, p_e_e_sum, p_s_e_e_sum = a_star(grid, grid_size, start, dest, True, True)
paths_e_theta, paths_smooth_e_theta, p_e_e_sum_theta, p_s_e_e_sum_theta = theta_star(grid, grid_size, start, dest, True, True)

print("... energy consumption estimate ...")
print(f"> a-star : {p_e_sum} / a-star smooth {p_s_e_sum} / a-star energy {p_e_e_sum} / a-star energy smooth {p_s_e_e_sum}")
print(f"> theta-star : {p_e_sum_theta} / theta-star smooth {p_s_e_sum_theta} / theta-star energy {p_e_e_sum_theta} / theta-star energy smooth {p_s_e_e_sum_theta}")

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
plot_path(gen_map_matrix, paths, "a_star")
plot_path(gen_map_matrix, paths_smooth, "smooth_a_star")
plot_path(gen_map_matrix, paths_theta, "theta_star")
plot_path(gen_map_matrix, paths_smooth_theta, "smooth_theta_star")

print("... writing path with energy constraints ...")
plot_path(gen_map_matrix, paths_e, "a_star_energy")
plot_path(gen_map_matrix, paths_smooth_e, "a_star_energy_smooth")
plot_path(gen_map_matrix, paths_e_theta, "theta_star_energy")
plot_path(gen_map_matrix, paths_smooth_e_theta, "theta_star_energy_smooth")


# Generate Ball Object in environment that represents path point
current_date = datetime.now()

a_star_path_filename = f"data_generation_{current_date.strftime('%Y%m%d')}\path_{current_date.strftime('%Y%m%d')}_a_star_energy_smooth.txt"
theta_star_path_filename = f"data_generation_{current_date.strftime('%Y%m%d')}\path_{current_date.strftime('%Y%m%d')}_theta_star_energy_smooth.txt"

f_a_star = open(a_star_path_filename, "r")
f_theta_star = open(theta_star_path_filename, "r")

ball_iter = 0
while True:
    line = f_a_star.readline()
    if not line:
        break

    translation = list(map(float, line.split()))
    color = [1,0,0]
    name = f"a_star_point_{ball_iter}"

    children_field.importMFNodeFromString(-1, point_option_gen(name, translation, color))

    ball_iter += 1

# ball_iter = 0
# while True:
#     line = f_theta_star.readline()
#     if not line:
#         break

#     translation = list(map(float, line.split()))
#     color = [0,0,1]
#     name = f"theta_star_point_{ball_iter}"

#     children_field.importMFNodeFromString(-1, point_option_gen(name, translation, color))

#     ball_iter += 1

f_a_star.close()
f_theta_star.close()



""" Webot environment setting """

time_step_iter = 0

# used in energy consumption model
before_time = supervisor.getTime()
before_drone_translation = drone_translation_field.getSFVec3f()
drone_rotation_field = drone.getField('rotation')
m_payload = 0 

adj_alpha = -0.008

while supervisor.step(timestep) != -1:
    time = supervisor.getTime()

    """ drone energy consuption calculation """

    drone_translation = drone_translation_field.getSFVec3f()
    drone_rotation = drone_rotation_field.getSFVec3f()
    drone_row_pitch_yaw = quatarnion_to_roll_pitch_yaw(drone_rotation)

    alpha = -(drone_row_pitch_yaw[1] - adj_alpha)
    time_delta = time - before_time
    V_vert = -1 * ((drone_translation[2] - before_drone_translation[2]) / time_delta)
    V_air = math.sqrt((drone_translation[0] - before_drone_translation[0])**2 + 
                       (drone_translation[1] - before_drone_translation[1])**2) / time_delta
    
    T = thrust(alpha, V_air, m_payload)
    P_i = P_induced(T, V_vert)
    P_p = P_profile(T, V_air, alpha)
    P_par = P_parasite(V_air)

    if time_step_iter % 20 == 0:
        print("translation:", drone_translation)
        print("roll, pitch, yaw:", drone_row_pitch_yaw)
        print(f"P_i {P_i} / P_p {P_p} / P_par {P_par}")


    time_step_iter += 1