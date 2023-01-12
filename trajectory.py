import numpy as np
from config import *
from scipy.spatial import KDTree

def calculate_trajectory(trajectory_start: np.ndarray, trajectory_goal: np.ndarray, collision_map: np.ndarray):
    """
    It calculates the trajectory via searching the minimum potentials points, exhaustively.
    """
    
    # Initialization of the counter
    idx = 0

    # Local searching step in terms of joint angle
    angle_step = 0.5

    # Initialize the trajectory with the starting point
    trajectory = np.array(trajectory_start,)
    trajectory = np.reshape(trajectory, (1, 2))
    
    # Initialize the checker variable
    is_trajectory_not_finished = True

    while is_trajectory_not_finished:
        
        local_alpha_values = np.arange(trajectory[idx, 0] - angle_step, trajectory[idx, 0] + 2 * angle_step, angle_step)
        local_beta_values = np.arange(trajectory[idx, 1] - angle_step, trajectory[idx, 1] + 2 * angle_step, angle_step)

        # Construct an array that stores the all possible combinations of the 
        # alpha and beta values
        alpha_grid, beta_grid = np.meshgrid(local_alpha_values, local_beta_values)
        local_map_grid = np.vstack((alpha_grid.flatten(order='F'), beta_grid.flatten(order='F'))).T
        
        # Initialize a KDTree object via feeding the subject point cloud, which is the
        # collision map
        kdt = KDTree(collision_map)
        # Then, find the distances which are from the point cloud to query point, which 
        # is whole map of our configuration space.
        # Keep in mind that we can query more than one point at the same time. 
        distances, _ = kdt.query(local_map_grid)

        # Setting the configuration parameter to a local variable for ease of use
        p0 = CONFIG['repulsion_threshold_distance']

        # If a distance is larger than a threshold, then its effect is very minimal
        # as if it is at a inifinite distance.
        distances[distances > p0] = np.inf

        # Calculate the repulsive potential given by the formula
        local_repulsive_potentials = 0.5 * CONFIG['repulsion_gain'] * np.power((1 / distances) - (1 / p0), 2) 

        # Calculate the distances between every point and the goal point
        local_attraction_distances = np.sum((local_map_grid - trajectory_goal) ** 2, axis=1)
        # Calculate the attractive potentials as if it is a spring acted on a mass
        local_attractive_potentials = 0.5 *  CONFIG["attraction_gain"] * local_attraction_distances

        # Calculate the total potentials by summing up repulsive and attractive potential
        local_potential_map = local_repulsive_potentials + local_attractive_potentials

        # Calculate the point whose potential is the minimum in the proximity
        # of the local grid
        min_potential_point = local_map_grid[np.argmin(local_potential_map),:].reshape((1,2))
        
        # Add this point to the trajectory
        trajectory = np.vstack((trajectory,min_potential_point))

        # Check whether the desired state is reached
        is_trajectory_not_finished = not (np.linalg.norm(trajectory[-1, :]- trajectory_goal) <= CONFIG["trajectory_proximity_threshold"] or np.array_equal(trajectory[-1,:],trajectory[-2,:]))
        
        # Increase the counter
        idx = idx + 1
    # Return the trajectory
    return trajectory

