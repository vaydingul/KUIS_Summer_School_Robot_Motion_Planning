import numpy as np
from config import *


def cosd(angle_degree):
    return np.cos(np.radians(angle_degree))


def sind(angle_degree):
    return np.sin(np.radians(angle_degree))


def is_inside_circle(px, py, cx, cy, r):
    return (px - cx) ** 2 + (py - cy) ** 2 <= r ** 2


def forward_kinematics(theta1, theta2):
    position = [0, 0]
    position[0] = CONFIG["link_1_length"] * cosd(theta1) + CONFIG["link_2_length"] * cosd(theta1 + theta2) + CONFIG[
        "robot_base_x"]
    position[1] = CONFIG["link_1_length"] * sind(theta1) + CONFIG["link_2_length"] * sind(theta1 + theta2) + CONFIG[
        "robot_base_y"]
    return position


def inverse_kinematics(x, y):
    solutions = [0, 0]
    x = x - CONFIG["robot_base_x"]
    y = y - CONFIG["robot_base_y"]

    if x ** 2 + y ** 2 <= (
            CONFIG["link_1_length"] + CONFIG["link_2_length"]) ** 2:  # Calculate the IK if the point is within reach
        c_2 = (x ** 2 + y ** 2 - CONFIG["link_1_length"] ** 2 - CONFIG["link_2_length"] ** 2) / (
                2 * CONFIG["link_1_length"] * CONFIG["link_2_length"])
        s_2 = np.sqrt(1 - c_2 ** 2)
        theta2 = [np.atan2(-s_2, c_2), np.atan2(s_2, c_2)]

        beta = np.atan2(y, x)
        c_psi = (x ** 2 + y ** 2 + CONFIG["link_1_length"] ** 2 - CONFIG["link_2_length"] ** 2) / (
                2 * CONFIG["link_1_length"] * np.sqrt(x ** 2 + y ** 2))
        psi = np.atan2(np.sqrt(1 - c_psi ** 2), c_psi)
        theta1 = [beta + psi, beta - psi]
        solutions = [theta1, theta2]
    return solutions
