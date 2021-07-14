import numpy as np
from utils import cosd,sind

# ! DON'T USE IT

def inverse_kinematics(x, y, L1 = 38, L2 = 38, robot_base_x=20, robot_base_y=0):
    solutions = [0, 0]
    x = x - robot_base_x
    y = y - robot_base_y

    if x**2 + y**2 <= (L1+L2) ** 2: # Calculate the IK if the point is within reach
        c_2 = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
        s_2 = np.sqrt(1 - c_2 ** 2)
        theta2 = [np.atan2(-s_2,c_2), np.atan2(s_2,c_2)]

        beta = np.atan2(y, x);
        c_psi = (x**2 + y**2 + L1 ** 2 - L2 ** 2) / (2 * L1 * np.sqrt(x**2 + y**2))
        psi = np.atan2(np.sqrt(1 - c_psi ** 2),c_psi)
        theta1 = [beta + psi, beta - psi]
        solutions = [theta1, theta2]

        

    
    return solutions


def forward_kinematics(theta1, theta2, L1 = 38, L2 = 38, robot_base_x=20, robot_base_y=0):

    position = [0,0]
    position[0] = L1 * cosd(theta1) + L2 * cosd(theta1 + theta2) + robot_base_x;
    position[1] = L1 * sind(theta1) + L2 * sind(theta1 + theta2) + robot_base_y;
    return position
  
