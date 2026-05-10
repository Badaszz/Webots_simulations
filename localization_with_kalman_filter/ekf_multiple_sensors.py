"""Line following with obstacle avoidance"""

import math
import numpy as np

from controller import Robot, Motor

from ekf import Kf_step
from line_following import compute_line_follow_action
from odometry import get_robot_speeds, get_wheels_speed


MAX_SPEED = 3.14

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
delta_t = timestep / 1000.0

# counters
counter = 0

# GPS --- Initialization ---
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

# Ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# Obstacle detection sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
state = 'forward'

# Initialize encoders
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

encoderValues = []
robot.step(timestep)
encoderValues = [encoder[0].getValue(), encoder[1].getValue()]

# EKF initialization
P = np.diag([1e-1, 1e-1, 1e-2])
Q = np.diag([0.01, 0.01, 0.001])
R = 0.001

x = 0.55328
y = 0.793216
phi = 0.480668

com_vals = compass.getValues()
phi = -math.atan2(com_vals[0], com_vals[1])

while robot.step(timestep) != -1:
    psValues = [sensor.getValue() for sensor in ps]
    gsValues = [sensor.getValue() for sensor in gs]

    line_right = gsValues[0] > 600
    line_left = gsValues[2] > 600
    line_forward = gsValues[1] > 600

    obstacle = (psValues[0] >= 80) or (psValues[1] >= 80) or (psValues[7] >= 80)
    on_line = (not line_left) or (not line_right) or (not line_forward)

    encoderValuesNew = [encoder[0].getValue(), encoder[1].getValue()]
    wheel_speeds = get_wheels_speed(encoderValuesNew, encoderValues, delta_t)
    encoderValues = encoderValuesNew

    u, w = get_robot_speeds(wheel_speeds[0], wheel_speeds[1])

    gps_values = gps.getValues()
    measured_x, measured_y = gps_values[0], gps_values[1]

    compass_values = compass.getValues()
    compass_phi = -math.atan2(compass_values[0], compass_values[1])

    x, y, phi, P = Kf_step(
        x,
        y,
        phi,
        P,
        u,
        w,
        compass_phi,
        delta_t,
        Q,
        R,
        measured_x,
        measured_y,
    )

    state, leftSpeed, rightSpeed, counter = compute_line_follow_action(
        state,
        line_left,
        line_right,
        line_forward,
        obstacle,
        on_line,
        counter,
        MAX_SPEED,
    )

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    print(
        f"sim_time : {robot.getTime():.4f}, x : {x:.4f}, y : {y:.4f}, phi: {phi:.4f}, state: {state}"
    )
