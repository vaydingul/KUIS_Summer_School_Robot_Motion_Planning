from potential import calculate_potential_map, draw_potential_map
from trajectory import calculate_trajectory
from utils import inverse_kinematics
from collision_map import create_collision_map
import matplotlib.pyplot as plt
import numpy as np
from config import *

if __name__ == "__main__":
		collision_map, alpha_axis, beta_axis = create_collision_map()

		plt.scatter(collision_map[:, 0], collision_map[:, 1], cmap='hot')

		potential_map = calculate_potential_map(collision_map, alpha_axis, beta_axis)		

		draw_potential_map(potential_map, alpha_axis, beta_axis)

		start_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["start_x"], CONFIG["start_y"], elbow_up=CONFIG['start_elbow_up']))
		goal_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["goal_x"], CONFIG["goal_y"], elbow_up=CONFIG['goal_elbow_up']))
		trajectory = calculate_trajectory(start_theta_degree, goal_theta_degree, collision_map)

		plt.show()
