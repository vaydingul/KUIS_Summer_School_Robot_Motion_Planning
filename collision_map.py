import numpy as np
from utils import *
from config import *


def create_collision_map():
    collision_map = np.empty((0, 2), dtype=np.int32)

    alpha_values = np.arange(0, 180 + CONFIG["angle_discretization_param"], CONFIG["angle_discretization_param"])
    beta_values = np.arange(0, 360 + CONFIG["angle_discretization_param"], CONFIG["angle_discretization_param"])
    link_1_values = np.arange(0, CONFIG["link_1_length"] + CONFIG["link_discretization_param"], CONFIG["link_discretization_param"])
    link_2_values = np.arange(0, CONFIG["link_2_length"] + CONFIG["link_discretization_param"], CONFIG["link_discretization_param"])

    for alpha in alpha_values:

        for beta in beta_values:

            print("Alpha = {} \nBeta = {}".format(alpha, beta))
            is_point_collided = False  # Set this to false for every alpha-beta pair

            for link_1 in link_1_values:  # Collision check for link 1
                px = link_1 * cosd(alpha) + CONFIG["robot_base_x"]  # X coordinates of the discretized points on link 1
                py = link_1 * sind(alpha) + CONFIG["robot_base_y"]  # Y coordinates of the discretized points on link 1

                is_point_collided = is_inside_circle(px, py, CONFIG["obstacle_x"], CONFIG["obstacle_y"],
                                                     CONFIG["obstacle_radius"] + CONFIG["rubber_band"]) or px < 0 or py < 0 or py > 100 or px > 100  # Whether the point is colliding with any of the boundary conditions

                if is_point_collided:  # If the link 2 violates any of the boundary conditions, terminate the link 1 check

                    break

            if is_point_collided:  # If link 1 collides with the given alpha-beta pair, then go for the next pair

                collision_map = np.vstack((collision_map, [alpha, beta]))

                continue

            for link_2 in link_2_values:  # Collision check for link 2
                px = CONFIG["link_1_length"] * cosd(alpha) + link_2 * cosd(
                    alpha + beta) + CONFIG["robot_base_x"]  # X coordinates of the discretized points on link 2
                py = CONFIG["link_1_length"] * sind(alpha) + link_2 * sind(
                    alpha + beta) + CONFIG["robot_base_y"]  # Y coordinates of the discretized points on link 2

                is_point_collided = is_inside_circle(px, py, CONFIG["obstacle_x"], CONFIG["obstacle_y"], CONFIG["obstacle_radius"] + CONFIG["rubber_band"]) or (
                            px < 0) or (py < 0) or (py > 100) or (
                                                px > 100)  # Whether the point is colliding with any of the boundary conditions

                if is_point_collided:  # If the link 2 violates any of the boundary conditions, terminate the link 2 check

                    break

            if is_point_collided:  # If link 2 collides with the given alpha-beta pair, then go for the next pair

                collision_map = np.vstack((collision_map, [alpha, beta]))

    return collision_map, alpha_values, beta_values
