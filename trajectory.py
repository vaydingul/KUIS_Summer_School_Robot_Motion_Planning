import numpy as np
from config import *
from scipy.spatial import KDTree

def calculate_trajectory(trajectory_start: np.ndarray, trajectory_goal: np.ndarray, collision_map: np.ndarray):
    idx = 0
    dI = 0.5
    trajectory = np.array(trajectory_start)
    trajectory = np.reshape(trajectory, (1, 2))
    
    while True:
        local_alpha = np.arange(trajectory[idx, 0] - dI, trajectory[idx, 0] + 2 * dI, dI)
        local_beta = np.arange(trajectory[idx, 1] - dI, trajectory[idx, 1] + 2 * dI, dI)

        alpha_grid, beta_grid = np.meshgrid(local_alpha, local_beta)
        local_map_grid = np.vstack((alpha_grid.reshape((np.prod(alpha_grid.shape), ), order='F'), beta_grid.reshape((np.prod(beta_grid.shape), ), order='F'))).T
        
        kdt = KDTree(collision_map)
        distances, _ = kdt.query(local_map_grid)

        p0 = CONFIG['repulsion_threshold_distance']
        # TODO: Check if it is correct
        distances[distances > p0] = p0

        local_repulsive_potentials = 0.5 * p0 * np.power((1 / distances) - (1 / p0), 2) 

        local_attraction_distances = np.sum((local_map_grid - trajectory_goal) ** 2, axis=1)
        local_attractive_potentials = 0.5 *  CONFIG["attraction_gain"] * local_attraction_distances;

        local_potential_map = local_repulsive_potentials + local_attractive_potentials

        min_potential_point = local_map_grid[np.argmin(local_potential_map),:].reshape((1,2))
        
        trajectory = np.vstack((trajectory,min_potential_point))
        
        if np.linalg.norm(trajectory[-1, :]- trajectory_goal) <= CONFIG["trajectory_proximity_threshold"] or np.array_equal(trajectory[-1,:],trajectory[-2,:]):
            return trajectory

        idx = idx + 1
