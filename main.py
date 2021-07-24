from potential import calculate_potential_map, draw_potential_map
from trajectory import calculate_trajectory
from utils import draw_collision_map, draw_potential_map, draw_potential_map_3d, inverse_kinematics
from collision_map import calculate_collision_map
import matplotlib.pyplot as plt
import numpy as np
from config import *
#import plotly.graph_objects as go


if __name__ == "__main__":
		
		#plt.style.use('classic')

		collision_map, alpha_axis, beta_axis = calculate_collision_map()

		draw_collision_map(collision_map)

		potential_map = calculate_potential_map(collision_map, alpha_axis, beta_axis)		

		draw_potential_map(potential_map, alpha_axis, beta_axis)
		draw_potential_map_3d(potential_map, alpha_axis, beta_axis)

		start_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["start_x"], CONFIG["start_y"], elbow_up=CONFIG['start_elbow_up']))
		goal_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["goal_x"], CONFIG["goal_y"], elbow_up=CONFIG['goal_elbow_up']))
		trajectory = calculate_trajectory(start_theta_degree, goal_theta_degree, collision_map)

		plt.show()