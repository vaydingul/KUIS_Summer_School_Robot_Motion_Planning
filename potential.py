from robot import Robot
import numpy as np
from utils import *
from config import *


def calculate_potential_map(collision_map, alpha_values, beta_values):
    x_grid, y_grid = np.meshgrid(alpha_values, beta_values)
    map_grid = np.vstack((x_grid.reshape((np.prod(x_grid.shape), ), order='F'), y_grid.reshape((np.prod(y_grid.shape), ), order='F'))).T
    pass
