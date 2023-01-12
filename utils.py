import numpy as np
from config import *


def cosd(angle_degree):
	"""
	Cosinus calculation for an angle in degree
	"""
	return np.cos(np.radians(angle_degree))


def sind(angle_degree):
	"""
	Sinus calculation for an angle in degree
	"""
	return np.sin(np.radians(angle_degree))

def is_inside_circle(point_x, point_y, center_x, center_y, radius_):
	"""
	It checks whether a point (point_x, point_y) is in the interior region of
	a circle whose center coordinates are (center_x, center_y) with a radius of radius_
	"""
	return (point_x - center_x) ** 2 + (point_y - center_y) ** 2 <= radius_ ** 2

def forward_kinematics(theta1, theta2):
	"""
	Calculate the corresponding world coordinates of the end effector for
	a given set of joint angles
	"""
	position = np.zeros((2,))
	position[0] = CONFIG["link_1_length"] * cosd(theta1) + CONFIG["link_2_length"] * cosd(theta1 + theta2) + CONFIG[
		"robot_base_x"]
	
	position[1] = CONFIG["link_1_length"] * sind(theta1) + CONFIG["link_2_length"] * sind(theta1 + theta2) + CONFIG[
		"robot_base_y"]
	
	return position

def inverse_kinematics(x, y, elbow_up = True):
	"""
	Calculate the joint angles that produce the given end effector world coordinates,
	with a predefined condiguration (elbow up or elbow down)
	"""
	
	# Calculate the relative position of the world coordinates with respect to base
	# of the robot
	x = x - CONFIG["robot_base_x"]
	y = y - CONFIG["robot_base_y"]

	# Calculate the IK if the point is within reach
	if x ** 2 + y ** 2 <= (CONFIG["link_1_length"] + CONFIG["link_2_length"]) ** 2:
		
		# Intermediate calculations
		c_2 = (x ** 2 + y ** 2 - CONFIG["link_1_length"] ** 2 - CONFIG["link_2_length"] ** 2) / (
				2 * CONFIG["link_1_length"] * CONFIG["link_2_length"])
		
		s_2 = np.sqrt(1 - c_2 ** 2)
		
		theta2 = np.array([np.arctan2(-s_2, c_2), np.arctan2(s_2, c_2)])

		beta = np.arctan2(y, x)
		
		c_psi = (x ** 2 + y ** 2 + CONFIG["link_1_length"] ** 2 - CONFIG["link_2_length"] ** 2) / (
				2 * CONFIG["link_1_length"] * np.sqrt(x ** 2 + y ** 2))
		
		psi = np.arctan2(np.sqrt(1 - c_psi ** 2), c_psi)
		
		theta1 = np.array([beta + psi, beta - psi])

		

		# Solutions for the elbow up and elbow down situations
		solutions = np.vstack([theta1, theta2]) % (2 * np.pi)

		# Select the solutions based on the configuration preference
		solution = solutions[:, 1 ] % (2 * np.pi) if not elbow_up else solutions[:, 0] % (2 * np.pi)

	return solution
