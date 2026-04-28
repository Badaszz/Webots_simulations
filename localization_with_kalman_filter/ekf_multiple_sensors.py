"""Line following with obstacle avoidance"""

from controller import Robot, Motor, DistanceSensor
import time
import math 
import numpy as np


# Radius of the wheels
R = 0.0205
# Distance between wheels
D = 0.052

# create the Robot instance.
robot = Robot()
MAX_SPEED = 3.14

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
time_t = 0
delta_t = timestep / 1000

## counter for step cycles
counter = 0
COUNTER_MAX = 6
COUNTER_2 = 31

line_counter = 0
LINE_CONFIRM = 4

# GPS --- Initialization ---
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)
## Ground Sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

## obstacle detection sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
state = 'forward'

leftSpeed = 0
rightSpeed = 0

# Initialize the encoder
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

encoderValues = []
robot.step(timestep)
encoderValues = [encoder[0].getValue(), encoder[1].getValue()]

### Function Definitions:
def get_wheels_speed(encoder2, encoder1, time_change):
    wheel_speeds = []
    for i in range(2):
        angular_change = encoder2[i]-encoder1[i] 
        wheel_speeds.append((angular_change*R)/time_change)
    return wheel_speeds
    
def get_robot_speeds(wheel_left, wheel_right):
    robot_speeds = []
    # Robot Linear velocity
    robot_speeds.append((wheel_right+wheel_left)/2)
    # Robot Angular Velocity
    robot_speeds.append((wheel_right-wheel_left)/D)
    return robot_speeds

def get_robot_pose(linear, angular, x_pos, y_pos, phi_orient, time_change):
    angle_term = (phi_orient + ((angular*time_change)/2))
    new_x = x_pos + ((linear * time_change)*(math.cos(angle_term))) 
    new_y = y_pos + ((linear * time_change)*(math.sin(angle_term)))
    new_phi = phi_orient + (angular*time_change)   
    
    return [new_x, new_y, new_phi]

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


## KF function
def Kf_step(x, y, phi, P, u, w, compass_phi, delta_t, Q, R, gps_x, gps_y):
    angle_term = phi + (w * delta_t / 2)
    
    # ---------------- PREDICT ----------------
    x_pred = x + u * delta_t * math.cos(angle_term)
    y_pred = y + u * delta_t * math.sin(angle_term)
    phi_pred = phi + w * delta_t
    
    J = np.array([
        [1, 0, -u * delta_t * math.sin(angle_term)],
        [0, 1,  u * delta_t * math.cos(angle_term)],
        [0, 0, 1]
    ])
    
    P_pred = J @ P @ J.T + Q
    
    # ---------------- COMPASS UPDATE ----------------
    H_compass = np.array([[0, 0, 1]])
    
    innovation = wrap_angle(compass_phi - phi_pred)
    
    S = H_compass @ P_pred @ H_compass.T + R
    K = P_pred @ H_compass.T / S
    
    state = np.array([x_pred, y_pred, phi_pred]) + K.flatten() * innovation
    x, y, phi = state
    
    P_new = (np.eye(3) - K @ H_compass) @ P_pred
    
    # ---------------- GPS UPDATE ----------------
    H_gps = np.array([
        [1, 0, 0],
        [0, 1, 0]
    ])
    
    z_gps = np.array([gps_x, gps_y])
    z_pred_gps = np.array([x, y])
    
    innovation_gps = z_gps - z_pred_gps
    
    S_gps = H_gps @ P_new @ H_gps.T + R
    K_gps = P_new @ H_gps.T @ np.linalg.inv(S_gps)
    
    state = np.array([x, y, phi]) + K_gps @ innovation_gps
    x, y, phi = state
    
    P_new = (np.eye(3) - K_gps @ H_gps) @ P_new
    
    return x, y, wrap_angle(phi), P_new
    
#GPS Values
gps_values = gps.getValues() # [x, y, z]
measured_x = gps_values[0]
measured_y = gps_values[1] 

## Initialize P
P = np.diag([1e-1, 1e-1, 1e-2])

## Q - process noise (uncertainty)
Q = np.diag([1e-4, 1e-4, 1e-5])


R_error = 0.001  # Lower = trust sensor more
Q = np.diag([0.01, 0.01, 0.001])  # Higher = allow for wheel slip/uncertainty

# initial positions
x = 0.55328
y = 0.793216
phi = 0.480668
com_vals = compass.getValues()
val = -math.atan2(com_vals[0], com_vals[1])
gyro_phi = val

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    psValues = []
    for i in range(8):
        # print(f"ps {i} = {ps[i].getValue()}")
        psValues.append(ps[i].getValue())
    
       
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Process sensor data 
    line_right = gsValues[0] > 600
    line_left = gsValues[2] > 600
    line_forward = gsValues[1] > 600
    
    obstacle = (psValues[0] >= 80) or (psValues[1] >= 80) or (psValues[7] >= 80)
    
    on_line = (not line_left) or (not line_right) or (not line_forward)
    
    if state == 'forward':
        ## Initial state is to move forward
        leftSpeed  = MAX_SPEED
        rightSpeed = MAX_SPEED
        
        # First check for obstacles upper left, front or upper right
        if obstacle :
            # if there is an obstacle, then turn to the right
            state = 'right_turn'
            counter = 0
        elif line_left and not line_right:
            # if line is to the left and not to the right
            state = 'left'
            counter = 0
        elif line_right and not line_left:
            state = 'right'
            counter = 0
     
    elif (state == 'right'):
        #turn right if line is to the right
        leftSpeed  = 0.8 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED
         
        if counter == COUNTER_MAX:
            state = 'forward'
     
    elif (state == 'left'):
        #turn left if line is to the left
        leftSpeed  = 0.4 * MAX_SPEED
        rightSpeed = 0.8 * MAX_SPEED
         
        if counter == COUNTER_MAX:
            state = 'forward' 
             
    elif (state == 'right_turn'):
        # right turn to avoid obstacles
        leftSpeed  = MAX_SPEED
        rightSpeed = 0 
        
        #turn right for 50 time steps
        if counter == 50:
            # when done turning right, then move forward for some time
            if obstacle:
                counter = 34
            else:
                state = 'forward2'
                counter = 0
    
    elif (state == 'forward2'):
        # move forward for some time, to get past the obstacle
        leftSpeed  = 0.4 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED
        
        ## Count 
        if counter == 15:
            # after sufficient forward movement, then turn left to return to line path
            state = 'left_turn'
            counter = 0
            
    elif (state == 'left_turn'):
        leftSpeed  = 0.2 * MAX_SPEED
        rightSpeed = 0.8 * MAX_SPEED
        
        if on_line and (counter > 16):
            # if back on line path, continue normal movement
            state = 'right_turn2'
            counter = 0
        elif obstacle:
            # if obstacle still present then 
            state = 'right_turn'
            counter = 0
        elif counter == COUNTER_2:
            state = 'forward3'
            counter = 0
    
    elif (state == 'forward3'):
        # short forward movement to get back to line path
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
        
        if on_line:
            state = 'right_turn2'
            counter = 0
        
        ## Counter set for 13 steps -- 0.4 second
        if counter == 13:
            state = 'left_turn'
            counter = 0
            
    elif (state == 'right_turn2'):
        # sharp right turn to get back on the line path
        leftSpeed  = MAX_SPEED
        rightSpeed = 0
        
        if counter == 30:
            state = 'forward'
            counter = 0
    
    counter += 1
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    # update sim_time
    time_t = time_t + delta_t
    
    oldEncoderValues = encoderValues.copy()
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    
    
    #COMPAS
    com_vals = compass.getValues()
    val = -math.atan2(com_vals[0], com_vals[1])        
    gyro_phi = val
    
    #GPS Values
    gps_values = gps.getValues() # [x, y, z]
    measured_x = gps_values[0]
    measured_y = gps_values[1] 
    
    # Speed of the wheels
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)

    # Speed of robot, Linear and Agnular
    [u, w] = get_robot_speeds(wl, wr)

    # KF step
    x, y, phi, P = Kf_step(x, y, phi, P, u, w, gyro_phi, delta_t, Q, R_error, measured_x, measured_y)
    
    print(f"sim_time : {time_t:.4f}, x : {x:.4f}, y : {y:.4f}, phi: {phi:.4f}")
