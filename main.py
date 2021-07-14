from potential import calculate_potential_map, draw_potential_map
from robot import Robot
from collision_map import create_collision_map
import matplotlib.pyplot as plt
from config import *

if __name__ == "__main__":
		print(CONFIG['link_1_length'])
		collision_map, alpha_axis, beta_axis = create_collision_map()
		potential_map = calculate_potential_map(collision_map, alpha_axis, beta_axis)
		plt.scatter(collision_map[:, 0], collision_map[:, 1], cmap='hot')
		draw_potential_map(potential_map, alpha_axis, beta_axis)
		plt.show()
