import numpy as np
from utils import *
from config import *
from scipy.spatial import KDTree 
import matplotlib.pyplot as plt

def calculate_potential_map(collision_map, alpha_values, beta_values):

    """
    It calculates the potential imposed by the obstacle configuration and the
    goal point.
    """
    # Construct an array that stores the all possible combinations of the 
    # alpha and beta values
    alpha_grid, beta_grid = np.meshgrid(alpha_values, beta_values)
    map_grid = np.vstack((alpha_grid.flatten(order='F'), beta_grid.flatten(order='F'))).T
    
    # Initialize a KDTree object via feeding the subject point cloud, which is the
    # collision map
    kdt = KDTree(collision_map)
    # Then, find the distances which are from the point cloud to query point, which 
    # is whole map of our configuration space.
    # Keep in mind that we can query more than one point at the same time. 
    distances, _ = kdt.query(map_grid, workers=4)
        
    # Calculate the repulsive potential given by the formula
    repulsive_potential = 0.5 * CONFIG['repulsion_gain'] * np.power((1 / distances) - (1 / CONFIG['repulsion_threshold_distance']), 2) 

    # If a distance is larger than a threshold, then its effect is zero
    repulsive_potential[distances > CONFIG['repulsion_threshold_distance']] = 0

    # Calculate the corresponding joint angles for the goal world coordinates
    goal_theta_degree = np.rad2deg(inverse_kinematics(CONFIG["goal_x"], CONFIG["goal_y"], elbow_up=CONFIG['goal_elbow_up']))

    # Calculate the distances between every point and the goal point
    attraction_distances = np.sum((map_grid - goal_theta_degree) ** 2, axis=1)

    # Calculate the attractive potentials as if it is a spring acted on a mass
    attractive_potentials = 0.5 * CONFIG["attraction_gain"] * attraction_distances

    # Calculate the total potentials by summing up repulsive and attractive potential
    potential_map = repulsive_potential + attractive_potentials

    # If there is a infinite value, change it with maximum value
    potential_map[potential_map == np.inf] = potential_map[np.isfinite(potential_map)].max()

    return potential_map

