from robot import Robot
import numpy as np
from utils import *
from config import *
from scipy.spatial import KDTree 
import matplotlib.pyplot as plt

def calculate_potential_map(collision_map, alpha_values, beta_values):

    x_grid, y_grid = np.meshgrid(alpha_values, beta_values)
    map_grid = np.vstack((x_grid.reshape((np.prod(x_grid.shape), ), order='F'), y_grid.reshape((np.prod(y_grid.shape), ), order='F'))).T
    
    kdt = KDTree(collision_map)
    distances, _ = kdt.query(map_grid)
    
    p0 = CONFIG['repulsion_threshold_distance']
    # TODO: Check if it is correct
    distances[distances > p0] = p0
        
    repulsive_potential = 0.5 * p0 * np.power((1 / distances) - (1 / p0), 2) 

    goal_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["goal_x"], CONFIG["goal_y"], elbow_up=CONFIG['goal_elbow_up']))

    attraction_distances = np.sum((map_grid - goal_theta_degree) ** 2, axis=1)

    attractive_potentials = 0.5 * CONFIG["attraction_gain"] * attraction_distances

    potential_map = repulsive_potential + attractive_potentials

    # TODO: Change it!
    potential_map[potential_map == np.inf] = potential_map[np.where(np.isinf(potential_map),-np.Inf,potential_map).argmax()]

    return potential_map


def draw_potential_map(potential_map, alpha_values, beta_values):

    plt.figure()
    alpha_grid, beta_grid = np.meshgrid(alpha_values, beta_values)
    #ax = plt.axes(projection='3d')
    plt.contour(alpha_grid, beta_grid, potential_map.reshape((alpha_values.shape[0], beta_values.shape[0])).T, 60, cmap='hot')
    plt.show()