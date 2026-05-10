import math

import numpy as np


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


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
