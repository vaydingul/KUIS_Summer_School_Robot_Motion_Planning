import numpy as np
from config import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import plotly.express as px
import plotly.graph_objects as go

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
		solutions = np.vstack([np.mod(theta1, 2 * np. pi), np.mod(theta2, 2 * np.pi)])
		# Select the solutions based on the configuration preference
		solution = solutions[:, 1] if elbow_up else solutions[:, 0]

	return solution

def draw_collision_map(collision_map):

	fig, ax = plt.subplots()
	
	ax.scatter(collision_map[:, 0], collision_map[:, 1], s = 5, c = "red", alpha = 0.5, marker = "s")
	ax.set_title("Collision Map")
	ax.set_xlabel("Alpha")
	ax.set_ylabel("Beta")

	return fig, ax

def draw_potential_map(potential_map, alpha_values, beta_values):

	fig, ax = plt.subplots()

	alpha_grid, beta_grid = np.meshgrid(alpha_values, beta_values)	
	c = ax.contour(alpha_grid, beta_grid, potential_map.reshape((alpha_values.shape[0], beta_values.shape[0])).T, cmap=cm.magma, levels = 200)
	fig.colorbar(c)
	ax.set_title("Potential Map")
	ax.set_xlabel("Alpha")
	ax.set_ylabel("Beta")

	return fig, ax

def draw_potential_map_3d(potential_map, alpha_values, beta_values):
	
	fig = plt.figure()
	ax = Axes3D(fig)
	fig.add_axes(ax)

	alpha_grid, beta_grid = np.meshgrid(alpha_values, beta_values)	
	c = ax.plot_surface(alpha_grid, beta_grid, potential_map.reshape((alpha_values.shape[0], beta_values.shape[0])).T, cmap=cm.magma, linewidth=0, antialiased=True)
	fig.colorbar(c)
	ax.set_title("Potential Map")
	ax.set_xlabel("Alpha")
	ax.set_ylabel("Beta")

	return fig, ax

def draw_collision_map_plotly(collision_map):

	fig = px.scatter(x=collision_map[:, 0], y=collision_map[:, 1])
	fig.update_layout(xaxis_range=[0, 180], yaxis_range=[0, 360])
	return fig

def draw_potential_map_plotly(potential_map, alpha_values, beta_values):

	fig = go.Figure(data =
    go.Contour(x = alpha_values, y = beta_values, z = potential_map.reshape((alpha_values.shape[0], beta_values.shape[0])).T))
	fig.update_layout(xaxis_range=[0, 180], yaxis_range=[0, 360])

	return fig

def draw_potential_map_3d_plotly(potential_map, alpha_values, beta_values):
	
	fig = go.Figure(data =
    go.Surface(x = alpha_values, y = beta_values, z = potential_map.reshape((alpha_values.shape[0], beta_values.shape[0])).T))
	fig.update_layout(xaxis_range=[0, 180], yaxis_range=[0, 360])

	return fig


def draw_collision_map_plotly(collision_map):

	fig = go.Figure()
	trc = go.Scatter(x=collision_map[:, 0], y=collision_map[:, 1], mode='markers')
	fig.add_trace(trc)

	fig.update_layout(
		autosize=False,
		width=500,
		height=500,
		xaxis_range=[0, 180], yaxis_range=[0, 360],
		xaxis_title = r"$\alpha$",
		yaxis_title = r"$\beta$",
		title={
			'text': "Collision Map",
			'x' : 0.5,
			'y' : 1.0,
			'xanchor': 'center',
			'yanchor': 'top'}
	)

	return fig, trc

def draw_potential_map_plotly(potential_map, alpha_values, beta_values):

	fig = go.Figure()
	trc = go.Contour(x = alpha_values, y = beta_values, z = potential_map.reshape((alpha_values.shape[0], beta_values.shape[0])).T)
	fig.add_trace(trc)
	
	fig.update_layout(
		autosize=False,
		width=500,
		height=500,
		xaxis_range=[0, 180], yaxis_range=[0, 360],
		xaxis_title = r"$\alpha$",
		yaxis_title = r"$\beta$",
		title={
			'text': "Potential Map",
			'x' : 0.5,
			'y' : 0.9,
			'xanchor': 'center',
			'yanchor': 'top'}
	)
	return fig, trc

def draw_potential_map_3d_plotly(potential_map, alpha_values, beta_values):
	

	fig = go.Figure()
	trc = go.Surface(x = alpha_values, y = beta_values, z = potential_map.reshape((alpha_values.shape[0], beta_values.shape[0])).T)
	fig.add_trace(trc)

	fig.update_layout(
		autosize=False,
		width=500,
		height=500,
		xaxis_range=[0, 180], yaxis_range=[0, 360],
		scene = dict(
		xaxis_title = r"$\alpha$",
		yaxis_title = r"$\beta$",
		zaxis_title = "Potential"), 
		title={
			'text': "Potential Map 3D",
			'x' : 0.5,
			'y' : 0.9,
			'xanchor': 'center',
			'yanchor': 'top'}
	)
	return fig, trc

def draw_subplots(trc_collision, trc_potential, trc_potential_3d):

	main_fig = go.make_subplots(
		rows=1, cols=3,
		subplot_titles=("Collision Map", "Potential Map", "Potential Map 3D"),
		specs=[[{}, {},{'type': 'surface'}]])

	main_fig.add_trace(trc_collision, row=1, col=1)
	main_fig.add_trace(trc_potential, row=1, col=2)
	main_fig.add_trace(trc_potential_3d, row=1, col=3)

	main_fig.update_xaxes(title_text=r"$\alpha$", row=1, col=1, range=[0, 180])
	main_fig.update_xaxes(title_text=r"$\alpha$", row=1, col=2, range=[0, 180])

	main_fig.update_yaxes(title_text=r"$\beta$", row=1, col=1, range=[0, 360])
	main_fig.update_yaxes(title_text=r"$\beta$", row=1, col=2, range=[0, 360])

	main_fig.update_scenes(xaxis = dict(title_text=r"Alpha"),
							yaxis = dict(title_text=r"Beta"),
							zaxis = dict(title_text="Potential"))
	return main_fig