CONFIG = {"link_1_length": 38, # Length of the first link of the robot
          "link_2_length": 37, # Length of the second link of the robot
          "robot_base_x": 20, # X coordinate of the base of the robot
          "robot_base_y": 0, # Y coordinate of the base of the robot
          "obstacle_x": 45, # X coordinate of the obstacle
          "obstacle_y": 60, # Y coordinate of the obstacle
          "obstacle_radius": 10, # Radius of the obstacle
          "link_discretization_param": 1, # Smallest length of the link to check for collision
          "angle_discretization_param": 1, # Smallest angle of a link to check for collision
          "rubber_band": 0, # Extra distance to add to the obstacle radius to avoid collision
          "start_x": 20, # X coordinate of the end effector at start
          "start_y": 70, # Y coordinate of the end effector at start
          "goal_x": 70, # Desired X coordinate of the end effector at goal
          "goal_y": 35, # Desired Y coordinate of the end effector at goal
          "start_elbow_up": False, # Whether the robot is in elbow-up configuration at start
          "goal_elbow_up": True, # Whether the robot is in elbow-up configuration at goal
          "attraction_gain": 1, # How much the robot is attracted to the goal
          "repulsion_gain": 5e3, # How much the robot is repelled from the obstacles
          "repulsion_threshold_distance": 20, # Distance at which the repulsion is applied
          "trajectory_proximity_threshold": 0.1 # Distance at which the robot is considered to have reached the goal
          } 

