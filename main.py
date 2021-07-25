from potential import calculate_potential_map, draw_potential_map
from trajectory import calculate_trajectory_via_search, calculate_trajectory_via_force_propagation
from utils import draw_collision_map, draw_potential_map, draw_potential_map_3d, forward_kinematics, inverse_kinematics
from collision_map import calculate_collision_map
import matplotlib.pyplot as plt
import numpy as np
from config import *
#import plotly.graph_objects as go


if __name__ == "__main__":
		
		#plt.style.use('classic')

		collision_map, alpha_axis, beta_axis = calculate_collision_map()

		_, ax_col = draw_collision_map(collision_map)

		potential_map = calculate_potential_map(collision_map, alpha_axis, beta_axis)		

		draw_potential_map(potential_map, alpha_axis, beta_axis)
		draw_potential_map_3d(potential_map, alpha_axis, beta_axis)

		start_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["start_x"], CONFIG["start_y"], elbow_up=CONFIG['start_elbow_up']))
		goal_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["goal_x"], CONFIG["goal_y"], elbow_up=CONFIG['goal_elbow_up']))
		trajectory_via_search = calculate_trajectory_via_search(start_theta_degree, goal_theta_degree, collision_map)
		trajectory_via_force_propagation = calculate_trajectory_via_force_propagation(start_theta_degree, goal_theta_degree, collision_map)
		
		trajectory_via_search_forward = np.empty_like(trajectory_via_search)
		trajectory_via_force_propagation_forward = np.empty_like(trajectory_via_force_propagation)

		for k in range(trajectory_via_search.shape[0]):
			trajectory_via_search_forward[k] = forward_kinematics(trajectory_via_search[k, 0], trajectory_via_search[k, 1])

		for k in range(trajectory_via_force_propagation.shape[0]):
			trajectory_via_force_propagation_forward[k] = forward_kinematics(trajectory_via_force_propagation[k, 0], trajectory_via_force_propagation[k, 1])

		fig, ax = plt.subplots()
		circle = plt.Circle((CONFIG["obstacle_x"], CONFIG["obstacle_y"]), CONFIG["obstacle_radius"], color='r')
		ax.plot(trajectory_via_search_forward[:, 0], trajectory_via_search_forward[:, 1], label = "Search")
		ax.plot(trajectory_via_force_propagation_forward[:, 0], trajectory_via_force_propagation_forward[:, 1], label = "Force Propagation")
		ax.add_patch(circle)
		ax.legend()
		
		ax_col.scatter(trajectory_via_search[:, 0], trajectory_via_search[:, 1], label = "Search {}".format(trajectory_via_search.shape[0]), s = 1)
		ax_col.scatter(trajectory_via_force_propagation[:, 0], trajectory_via_force_propagation[:, 1], label = "Force Propagation {}".format(trajectory_via_force_propagation.shape[0]), s = 1)
		ax_col.legend()

		#! Minimum potential finding is useful when the overall scenario is known beforehand
		#! Force propagation is useful in real-time situations

		plt.show()