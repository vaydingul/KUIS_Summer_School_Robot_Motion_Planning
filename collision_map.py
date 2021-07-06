from robot import Robot
import numpy as np
from utils import *

def create_collision_map(robot : Robot, obstacle_radius=10, obstacle_x=40, obstacle_y=60, link_discretization_param=1, angle_discretization_param=5,rubber_band=0):
	collision_map = np.empty((0, 2), dtype= np.int32)

	alpha_values = np.arange(0, 180 + angle_discretization_param, angle_discretization_param)
	beta_values = np.arange(0, 360 + angle_discretization_param, angle_discretization_param)
	link_1_values = np.arange(0, robot.link_1_length + link_discretization_param, link_discretization_param )
	link_2_values = np.arange(0, robot.link_2_length + link_discretization_param, link_discretization_param )

	for alpha in alpha_values:
		
		for beta in beta_values:
			
			print("Alpha = {} \nBeta = {}".format(alpha, beta))
			is_point_collided = False #  Set this to false for every alpha-beta pair
				
			for link_1 in link_1_values: # Collision check for link 1
				px = link_1 * cosd(alpha) + robot.robot_base_x # X coordinates of the discretized points on link 1
				py = link_1 * sind(alpha) + robot.robot_base_y # Y coordinates of the discretized points on link 1
				
				is_point_collided = is_inside_circle(px, py, obstacle_x, obstacle_y, obstacle_radius + rubber_band) or px < 0 or py < 0 or py > 100 or px > 100  # Whether the point is colliding with any of the boundary conditions
							
				if is_point_collided: # If the link 2 violates any of the boundary conditions, terminate the link 1 check
				
					break
					
				

			if is_point_collided: # If link 1 collides with the given alpha-beta pair, then go for the next pair					
				
				collision_map = np.vstack((collision_map, [alpha, beta]))
					
				continue
			
			for link_2 in link_2_values: # Collision check for link 2
					px = robot.link_1_length * cosd(alpha) + link_2 * cosd(alpha + beta) + robot.robot_base_x # X coordinates of the discretized points on link 2
					py = robot.link_1_length * sind(alpha) + link_2 * sind(alpha + beta) + robot.robot_base_y # Y coordinates of the discretized points on link 2
					
					is_point_collided = is_inside_circle(px, py, obstacle_x, obstacle_y, obstacle_radius + rubber_band) or (px < 0) or (py < 0) or (py > 100) or (px > 100) # Whether the point is colliding with any of the boundary conditions

					if is_point_collided: # If the link 2 violates any of the boundary conditions, terminate the link 2 check
				
						break

			if is_point_collided: # If link 2 collides with the given alpha-beta pair, then go for the next pair
				
				collision_map = np.vstack((collision_map, [alpha, beta]))


	return collision_map