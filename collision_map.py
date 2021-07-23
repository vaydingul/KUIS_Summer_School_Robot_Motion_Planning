import numpy as np
from utils import *
from config import *


def calculate_collision_map():
	"""
	It calculates the collision map of the given robot and obstacle configuration.
	Basically, it finds the situations in which there is a collision between
	robot and obstacle.
	"""
	# Collision map initialization
	collision_map = np.empty((0, 2), dtype=np.int32)

	# Constructing a range of alpha and beta values to check if they result in any collision with obstacle
	alpha_values = np.arange(0, 180 + CONFIG["angle_discretization_param"], CONFIG["angle_discretization_param"])
	beta_values = np.arange(0, 360 + CONFIG["angle_discretization_param"], CONFIG["angle_discretization_param"])

	# Construct an array that stores the all possible combinations of the 
	# alpha and beta values
	alpha_grid, beta_grid = np.meshgrid(alpha_values, beta_values)
	map_grid = np.vstack((alpha_grid.flatten(order='F'), beta_grid.flatten(order='F'))).T

	# Discretizing the link of the robot to check whether any portion of them has a contact with the obstacle
	link_1_values = np.arange(0, CONFIG["link_1_length"] + CONFIG["link_discretization_param"], CONFIG["link_discretization_param"])
	link_2_values = np.arange(0, CONFIG["link_2_length"] + CONFIG["link_discretization_param"], CONFIG["link_discretization_param"])

	# For each alpha and beta values


	for (alpha, beta) in map_grid:

		# Verbose printing
		print("Alpha = {}\tBeta = {}".format(alpha, beta), end='\r')

		# Initialization of the checker variable
		is_point_collided = False  # Set this to false for every alpha-beta pair

		# Think of the link 1 as a small version of the original link
		for link_1 in link_1_values:  # Collision check for link 1
			
			px = link_1 * cosd(alpha) + CONFIG["robot_base_x"]  # X coordinates of the discretized points on link 1
			py = link_1 * sind(alpha) + CONFIG["robot_base_y"]  # Y coordinates of the discretized points on link 1

			# Whether the point is colliding with any of the boundary conditions
			is_point_collided = is_inside_circle(px, py, CONFIG["obstacle_x"], CONFIG["obstacle_y"],
													CONFIG["obstacle_radius"] + CONFIG["rubber_band"]) or px < 0 or py < 0 or py > 100 or px > 100 

			if is_point_collided:  
				# If the link 1 violates any of the boundary conditions, terminate the link 1 check
				break

		if is_point_collided:  
			# If link 1 collides with the given alpha-beta pair, then go for the next pair
			collision_map = np.vstack((collision_map, [alpha, beta]))
			continue

		# Think of the link 2 as a small version of the original link
		for link_2 in link_2_values:  # Collision check for link 2

			px = CONFIG["link_1_length"] * cosd(alpha) + link_2 * cosd(
				alpha + beta) + CONFIG["robot_base_x"]  # X coordinates of the discretized points on link 2
			py = CONFIG["link_1_length"] * sind(alpha) + link_2 * sind(
				alpha + beta) + CONFIG["robot_base_y"]  # Y coordinates of the discretized points on link 2

			# Whether the point is colliding with any of the boundary conditions
			is_point_collided = is_inside_circle(px, py, CONFIG["obstacle_x"], CONFIG["obstacle_y"], CONFIG["obstacle_radius"] + CONFIG["rubber_band"]) or (px < 0) or (py < 0) or (py > 100) or (px > 100) 

			if is_point_collided:  
				# If the link 2 violates any of the boundary conditions, terminate the link 2 check
				break

		if is_point_collided:  
		# If link 2 collides with the given alpha-beta pair, then go for the next pair
			collision_map = np.vstack((collision_map, [alpha, beta]))

	# Return the collision map and the discretized alpha and beta values for later use
	return collision_map, alpha_values, beta_values
