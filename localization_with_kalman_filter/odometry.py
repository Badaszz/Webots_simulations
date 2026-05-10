import math

R = 0.0205
D = 0.052


def get_wheels_speed(current_encoders, previous_encoders, time_change):
    wheel_speeds = []
    for i in range(2):
        angular_change = current_encoders[i] - previous_encoders[i]
        wheel_speeds.append((angular_change * R) / time_change)
    return wheel_speeds


def get_robot_speeds(wheel_left, wheel_right):
    robot_linear = (wheel_right + wheel_left) / 2
    robot_angular = (wheel_right - wheel_left) / D
    return [robot_linear, robot_angular]


def get_robot_pose(linear, angular, x_pos, y_pos, phi_orient, time_change):
    angle_term = phi_orient + ((angular * time_change) / 2)
    new_x = x_pos + (linear * time_change) * math.cos(angle_term)
    new_y = y_pos + (linear * time_change) * math.sin(angle_term)
    new_phi = phi_orient + (angular * time_change)
    return [new_x, new_y, new_phi]
