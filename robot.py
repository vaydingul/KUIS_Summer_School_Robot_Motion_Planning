from utils import cosd, sind
import numpy as np


class Robot:

    def __init__(self, link_1_length=38, link_2_length=38, robot_base_x=20, robot_base_y=0):
        self.link_1_length = link_1_length
        self.link_2_length = link_2_length
        self.robot_base_x = robot_base_x
        self.robot_base_y = robot_base_y

    def inverse_kinematics(self, x, y):
        solutions = [0, 0]
        x = x - self.robot_base_x
        y = y - self.robot_base_y

        if x ** 2 + y ** 2 <= (
                self.link_1_length + self.link_2_length) ** 2:  # Calculate the IK if the point is within reach
            c_2 = (x ** 2 + y ** 2 - self.link_1_length ** 2 - self.link_2_length ** 2) / (
                    2 * self.link_1_length * self.link_2_length)
            s_2 = np.sqrt(1 - c_2 ** 2)
            theta2 = [np.atan2(-s_2, c_2), np.atan2(s_2, c_2)]

            beta = np.atan2(y, x)
            c_psi = (x ** 2 + y ** 2 + self.link_1_length ** 2 - self.link_2_length ** 2) / (
                    2 * self.link_1_length * np.sqrt(x ** 2 + y ** 2))
            psi = np.atan2(np.sqrt(1 - c_psi ** 2), c_psi)
            theta1 = [beta + psi, beta - psi]
            solutions = [theta1, theta2]
        return solutions

    def forward_kinematics(self, theta1, theta2):
        position = [0, 0]
        position[0] = self.link_1_length * cosd(theta1) + self.link_2_length * cosd(
            theta1 + theta2) + self.robot_base_x
        position[1] = self.link_1_length * sind(theta1) + self.link_2_length * sind(
            theta1 + theta2) + self.robot_base_y

        return position
