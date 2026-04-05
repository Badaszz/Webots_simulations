"""Line following with obstacle avoidance"""

from controller import Robot, Motor, DistanceSensor
import time
import math

# Radius of the wheels
R = 0.0205
# Distance between wheels
D = 0.052

# create the Robot instance.
robot = Robot()
MAX_SPEED = 6.28

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

gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

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

# initial positions
x = -0.416
y = -0.0292
phi = 2.92

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

    # if line_detected:
        # line_counter += 1
    # else:
        # line_counter = 0
    
    # on_line = line_counter >= LINE_CONFIRM
    
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
    
    # update sim_time
    time_t = time_t + delta_t
    
    oldEncoderValues = encoderValues
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]
            
    # print(f"Counter: {counter}, State: {state}")
    
    ## for debugging
    # print(f"{gsValues[0]}, {gsValues[1]}, {gsValues[2]} ")
    
    # Compute speed of the wheels
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)

    
    [u, w] = get_robot_speeds(wl, wr)

    
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)

    
    print(f"sim_time : {time_t:.4f}, x : {x:.4f}, y : {y:.4f}, phi: {phi:.4f}")
    
    counter += 1
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Enter here exit cleanup code.
