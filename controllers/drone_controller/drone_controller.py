"""drone_controller_python controller."""

'''
/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */

 # modified mavic2pro_controller.c into python

'''

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit
from controller import Keyboard
from controller import LED
from controller import Motor

import math
from datetime import datetime
from energy_model import *
import os

def SIGN(x):
    return ((x) > 0) - ((x) < 0)

def CLAMP(value, low, high):
    return max(low, min(value, high))

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get and enable devices
camera: Camera = robot.getDevice('camera')
camera.enable(timestep)
front_left_led: LED = robot.getDevice('front left led')
front_right_led: LED = robot.getDevice('front right led')
imu: InertialUnit = robot.getDevice('inertial unit')
imu.enable(timestep)
gps: GPS = robot.getDevice('gps')
gps.enable(timestep)
compass: Compass = robot.getDevice('compass')
compass.enable(timestep)
gyro: Gyro = robot.getDevice('gyro')
gyro.enable(timestep)
keyboard: Keyboard = robot.getKeyboard()
keyboard.enable(timestep)
camera_roll_motor: Motor = robot.getDevice('camera roll')
camera_pitch_motor: Motor = robot.getDevice('camera pitch')

# Get propeller motors and set them to velocity mode
front_left_motor: Motor = robot.getDevice('front left propeller')
front_right_motor: Motor = robot.getDevice('front right propeller')
rear_left_motor: Motor = robot.getDevice('rear left propeller')
rear_right_motor: Motor = robot.getDevice('rear right propeller')

front_left_motor.setPosition(float('+inf'))
front_left_motor.setVelocity(1.0)
front_right_motor.setPosition(float('+inf'))
front_right_motor.setVelocity(1.0)
rear_left_motor.setPosition(float('+inf'))
rear_left_motor.setVelocity(1.0)
rear_right_motor.setPosition(float('+inf'))
rear_right_motor.setVelocity(1.0)


# Display the welcome message
print("Start the drone...")

# Wait one second
while robot.step(timestep) != -1:
    if(robot.getTime() > 1.0):
        break

# Display manual control message
print("You can control the drone with your computer keyboard:")
print("- 'up': move forward.")
print("- 'down': move backward.")
print("- 'right': turn right.")
print("- 'left': turn left.")
print("- 'page up': increase the target altitude.")
print("- 'page down': decrease the target altitude.")
print("- 'shift + right': strafe right.")
print("- 'shift + left': strafe left.")

# Constants, empirically found
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

# Variables
target_altitude = 0.5

# payload mass
m_payload = 0.1


before_position = gps.getValues()
total_angle = [0,0,0]
before_angluar_velocity = gyro.getValues()
before_time = robot.getTime()

current_date = datetime.now()
folder_name = "data_" + current_date.strftime('%Y%m%d')
if not os.path.exists(folder_name):
    os.makedirs(folder_name)


""" climb rate file """
# file_path_climb = "climb_rate_{}.csv".format(current_date.strftime('%Y%m%d'))
# file_path_climb = os.path.join(folder_name, file_path_climb)
# file_climb = open(file_path_climb, 'w')

""" energy consumption file """
file_path_energy = "energy_{}.csv".format(current_date.strftime('%Y%m%d'))
file_path_energy = os.path.join(folder_name, file_path_energy)
file_energy = open(file_path_energy, 'w')


time_step_iter = 0

# Main loop:
while robot.step(timestep) != -1:
    time = robot.getTime()

    '''drone actuator part'''

    # Retrieve robot position using the sensors
    roll = imu.getRollPitchYaw()[0]
    pitch = imu.getRollPitchYaw()[1]
    altitude = gps.getValues()[2]
    roll_velocity = gyro.getValues()[0]
    pitch_velocity = gyro.getValues()[1]

    # Blink the front LEDs alternatively with a 1 second rate
    led_state = int(time) % 2
    front_left_led.set(led_state)
    front_right_led.set(not led_state)

    # Stabilize the Camera by actuating the camera motors according to the gyro feedback
    camera_roll_motor.setPosition(-0.115 * roll_velocity)
    camera_pitch_motor.setPosition(-0.1 * pitch_velocity)

    # Transform the keyboard input to disturbances on the stabilization algorithm
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    key = keyboard.getKey()
    while (key > 0):
        if key == keyboard.UP:
            pitch_disturbance = -2.0
        elif key == keyboard.DOWN:
            pitch_disturbance = 2.0
        elif key == keyboard.RIGHT:
            yaw_disturbance = -1.3
        elif key == keyboard.LEFT:
            yaw_disturbance = 1.3
        elif key == keyboard.SHIFT + keyboard.RIGHT:
            roll_disturbance = -1.0
        elif key == keyboard.SHIFT + keyboard.LEFT:
            roll_disturbance = 1.0
        elif key == keyboard.PAGEUP:
            target_altitude += 0.05
            print("target altitude: {} [m]".format(target_altitude))
        elif key == keyboard.PAGEDOWN:
            target_altitude -= 0.05
            print("target altitude: {} [m]".format(target_altitude))
        key = keyboard.getKey()
    
    # Compute the roll, pitch, yaw and vertical inputs
    roll_input = (k_roll_p * CLAMP(roll, -1.0, 1.0)) + roll_velocity + roll_disturbance
    pitch_input = (k_pitch_p * CLAMP(pitch, -1.0, 1.0)) + pitch_velocity + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_diffenence_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * math.pow(clamped_diffenence_altitude, 3.0)

    # Actuate the motors taking into consideration all the computed inputs
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)


    '''drone energy consumption model part - old version'''

    # current_position = gps.getValues()
    # v_1 = math.sqrt(math.pow(gps.getSpeedVector()[0],2) + math.pow(gps.getSpeedVector()[1],2))
    # v_2 = gps.getSpeedVector()[2]
    # h_delta = current_position[2] - before_position[2]
    # x_delta = math.sqrt(math.pow(current_position[0]-before_position[0], 2) + 
    #                     math.pow(current_position[1]-before_position[1], 2))
    
    # W = energy_consumption(v_1, v_2, m_payload, h_delta, x_delta)

    # before_position = gps.getValues()
    # print("current energy consumption (J) : {} / h_delta : {}".format(W, h_delta))

    """ drone energy consumption model part """

    # get angle with integrate angular velocity
    delta_time = time - before_time
    angular_velocity = gyro.getValues()
    delta_angle = [(a_vel - prev_a_vel)*delta_time for a_vel, prev_a_vel in zip(angular_velocity, before_angluar_velocity)]
    total_angle = [angle + d_angle for angle, d_angle in zip(total_angle, delta_angle)]

    before_time = time
    before_angluar_velocity = angular_velocity

    V_air = math.sqrt(math.pow(gps.getSpeedVector()[0],2) + math.pow(gps.getSpeedVector()[1],2))
    V_vert = abs(gps.getSpeedVector()[2]) 
    alpha = total_angle[1]    # attack angle is y-axis
    T = thrust(alpha, V_air, m_payload)

    power_induced = P_induced(T, V_vert)
    power_profile = P_profile(T, V_air, alpha)
    power_parasite = P_parasite(V_air)
    total_power = power_induced + power_profile + power_parasite

    if time_step_iter % 20 == 0:
        print(f"total energy consumption : {total_power} (J)")
        print(f"> induced power : {power_induced} (J)")
        print(f"> profile power : {power_profile} (J)")
        print(f"> parasite power : {power_parasite} (J)")
        file_energy.write(f"{round(time, 2)},{round(total_power, 5)},{round(power_induced, 5)},{round(power_profile, 5)},{round(power_parasite, 5)}\n")
        # file_climb.write("{},{},{},{}\n".format(time, current_position[0], current_position[1], current_position[2]))


    '''drone climb rate part'''
    # if time_step_iter % 20 == 0:
    #     file_climb.write("{},{},{},{}\n".format(time, current_position[0], current_position[1], current_position[2]))


    time_step_iter += 1

# Enter here exit cleanup code.

# file_climb.close()
file_energy.close()