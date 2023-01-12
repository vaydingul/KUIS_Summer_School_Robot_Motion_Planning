import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from config import *
from utils import cosd, sind, inverse_kinematics, forward_kinematics


class RobotAnimator:
    def __init__(self, trajectory: np.ndarray, collision_map: np.ndarray):
        self.x_ee, self.y_ee, self.alpha, self.beta = [], [], [], []
        self.set_trajectory(trajectory)
        self.set_collision_map(collision_map)
        self.fig, self.axs = plt.subplots(nrows=1, ncols=2, figsize=[20, 10])

    def set_trajectory(self, trajectory: np.ndarray):
        """Set the trajectory in alpha beta pairs"""
        self.trajectory = trajectory
        self.trajectory_forward = np.empty_like(trajectory)
        for k in range(trajectory.shape[0]):
            self.trajectory_forward[k] = forward_kinematics(
                trajectory[k, 0], trajectory[k, 1])
        self.alpha.append(self.trajectory[0, 0])
        self.beta.append(self.trajectory[0, 1])
        self.x_ee.append(self.trajectory_forward[0, 0])
        self.y_ee.append(self.trajectory_forward[0, 1])

    def set_collision_map(self, collision_map: np.ndarray):
        self.collision_map = collision_map

    def initialize_animation(self):
        goal_theta_degree = np.rad2deg(inverse_kinematics(
            CONFIG["goal_x"], CONFIG["goal_y"], elbow_up=CONFIG['goal_elbow_up']))

        ####### Collision map plot ###################################################
        # Put "start" - "goal" - "end" points on the collision map
        self.axs[1].scatter(self.collision_map[:, 0],
                            self.collision_map[:, 1], s=5, c="red", alpha=0.5, marker="s")
        self.axs[1].scatter(self.trajectory[0, 0],
                            self.trajectory[0, 1], marker=r"$start$", s=1000)
        self.axs[1].scatter(goal_theta_degree[0],
                            goal_theta_degree[1], marker=r"$goal$", s=1000)
        if np.linalg.norm(self.trajectory[-1, :] - [goal_theta_degree[0], goal_theta_degree[1]]) > 2:
            self.axs[1].scatter(self.trajectory[-1, 0],
                                self.trajectory[-1, 1], marker=r"$end$", s=1000)
        # Set collision map title
        self.axs[1].set_title("Collision Map")
        # Put current alpha and beta values on the map
        self.collision_map_angle_text = self.axs[1].text(5, 5, fr"""$\alpha$ : {self.trajectory[0, 0]:.1f} 
        $\beta$ : {self.trajectory[0, 1]:.1f}""")
        # Put the percentage progress of the trajectory to the map
        self.collision_map_percentage_text = self.axs[1].text(
            90, 180, fr"0%", fontsize=100, alpha=0.1, horizontalalignment='center', verticalalignment='center')
        # Draw the starting point
        self.collision_map_trajectory, = self.axs[1].plot(
            self.alpha, self.beta)
        # Set x-axis and y-axis labels/limits
        self.axs[1].set_xlabel("Alpha")
        self.axs[1].set_ylabel("Beta")
        self.axs[1].set_xlim(0, 180)
        self.axs[1].set_ylim(0, 360)
        # self.axs[1].grid(True)
        ##############################################################################

        ####### Cartesian map plot ###################################################
        # Draw the obstacle circle
        circle = plt.Circle(
            (CONFIG["obstacle_x"], CONFIG["obstacle_y"]), CONFIG["obstacle_radius"], color='r')
        # Draw the links of the robot
        self.link_1 = plt.Rectangle((CONFIG["robot_base_x"], CONFIG["robot_base_y"]),
                                    CONFIG["link_1_length"], 1,  self.trajectory[0, 0], color="green")
        # X coordinates of the discretized points on link 1
        self.link_1_ee_x = CONFIG["link_1_length"] * \
            cosd(self.trajectory[0, 0]) + CONFIG["robot_base_x"]
        # Y coordinates of the discretized points on link 1
        self.link_1_ee_y = CONFIG["link_1_length"] * \
            sind(self.trajectory[0, 0]) + CONFIG["robot_base_y"]
        self.link_2 = plt.Rectangle((self.link_1_ee_x, self.link_1_ee_y),
                                    CONFIG["link_2_length"], 1, self.trajectory[0, 0] + self.trajectory[0, 1])

        # Set title/label of the cartesian map
        self.axs[0].set_title("Cartesian Map")
        self.axs[0].grid()
        self.axs[0].set_xlim(0, 100)
        self.axs[0].set_ylim(0, 100)
        # Add circle and links to the map
        self.axs[0].add_patch(circle)
        self.axs[0].add_patch(self.link_1)
        self.axs[0].add_patch(self.link_2)
        # Draw the starting point
        self.cartesian_map_trajectory, = self.axs[0].plot(self.x_ee, self.y_ee)

        # Put "start" - "goal" - "end" points on the collision map
        self.axs[0].scatter(self.trajectory_forward[0, 0],
                            self.trajectory_forward[0, 1], marker=r"$start$", s=1000)
        self.axs[0].scatter(
            CONFIG["goal_x"], CONFIG["goal_y"], marker=r"$goal$", s=1000)
        if np.linalg.norm(self.trajectory[-1, :] - [goal_theta_degree[0], goal_theta_degree[1]]) > 2:
            self.axs[0].scatter(self.trajectory_forward[-1, 0],
                                self.trajectory_forward[-1, 1], marker=r"$end$", s=1000)
        # Put the percentage progress of the trajectory to the map
        self.cartesian_map_percentage_text = self.axs[0].text(
            CONFIG["obstacle_x"], CONFIG["obstacle_y"], fr"0%", fontsize=40, alpha=0.45, horizontalalignment='center', verticalalignment='center')
        # Put the current x and y location to the map
        self.cartesian_map_angle_text = self.axs[0].text(100 * 5 / 180, 100 * 5 / 360, fr"""$x$ : {self.trajectory_forward[0, 0]:.1f} 
        $y$ : {self.trajectory_forward[0, 1]:.1f}""")
        ##############################################################################

    def update_frame(self, n):
        # Clear the frame if it is the first frame
        if(n==0):
            self.x_ee, self.y_ee, self.alpha, self.beta = [], [], [], []
            self.alpha.append(self.trajectory[0, 0])
            self.beta.append(self.trajectory[0, 1])
            self.x_ee.append(self.trajectory_forward[0, 0])
            self.y_ee.append(self.trajectory_forward[0, 1])
        # Add current points to the plots
        self.alpha.append(self.trajectory[n, 0])
        self.beta.append(self.trajectory[n, 1])
        self.x_ee.append(self.trajectory_forward[n, 0])
        self.y_ee.append(self.trajectory_forward[n, 1])
        
        self.collision_map_trajectory.set_data(self.alpha, self.beta)
        self.cartesian_map_trajectory.set_data(self.x_ee, self.y_ee)
        
        percentage = (n+1) / self.trajectory.shape[0] * 100
        # Change link position and angles
        self.link_1.angle = self.alpha[-1]
        self.link_2.angle = self.beta[-1] + self.alpha[-1]
        # X coordinates of the discretized points on link 1
        self.link_1_ee_x = CONFIG["link_1_length"] * \
            cosd(self.trajectory[n, 0]) + CONFIG["robot_base_x"]
        # Y coordinates of the discretized points on link 1
        self.link_1_ee_y = CONFIG["link_1_length"] * \
            sind(self.trajectory[n, 0]) + CONFIG["robot_base_y"]
        self.link_2.set_xy((self.link_1_ee_x, self.link_1_ee_y))

        # Change current angle in collision map
        self.collision_map_angle_text.set_text(fr"$\alpha$ : {self.trajectory[n, 0]:.1f}""\n"fr"$\beta$ : {self.trajectory[n, 1]:.1f}")
        # Change percentage in collision map
        self.collision_map_percentage_text.set_text(fr"{percentage:.1f}%")

        # Change current position in cartesian map
        self.cartesian_map_angle_text.set_text(fr"$x$ : {self.trajectory_forward[n, 0]:.1f}""\n"fr"$y$ : {self.trajectory_forward[n, 1]:.1f}")
        # Change percentage in cartesian map
        self.cartesian_map_percentage_text.set_text(fr"{percentage:.1f}%")

        return self.collision_map_trajectory, self.cartesian_map_trajectory, self.link_1, self.link_2, self.collision_map_angle_text, self.cartesian_map_angle_text, self.collision_map_percentage_text, self.cartesian_map_percentage_text

    def animate(self):
        self.initialize_animation()
        ani = animation.FuncAnimation(fig=self.fig, func=self.update_frame, interval=10, blit=True, frames=np.arange(
            0, self.trajectory.shape[0], 1), repeat_delay=1000)
        plt.show()
