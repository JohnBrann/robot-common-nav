import numpy as np



def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
                reached_waypoint, terminated, linear_velocity, angular_velocity):
    # Initialize reward
    reward = 0.0

    # Reward for progress toward the goal
    if distance_to_goal < initial_distance_to_goal:
        reward += (initial_distance_to_goal - distance_to_goal) * 10  # Scaled reward for getting closer to the goal

    # Penalty for deviation from goal alignment
    reward -= abs(angle_to_goal) * 2  # Penalize larger angular deviation

    # Penalty for proximity to obstacles
    if min_obstacle_distance < 0.25:
        reward -= 20  # Harsh penalty for being too close to obstacles

    # Rewards or penalties based on velocities
    reward -= abs(angular_velocity) * 2  # Penalize excessive angular velocity
    # reward += linear_velocity * 10  # Reward forward motion (assuming positive linear velocity)
    if linear_velocity < 0.15:
        reward -= 1

    # Reward for success
    if reached_waypoint:
        reward += 200  # High reward for successfully reaching the waypoint

    # Penalty for termination
    if terminated:
        reward -= 100  # Penalty for termination (e.g., collision or timeout)


    reward = round(reward, 2)
    # Return the calculated reward
    return reward



# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
#     # Initial reward
#     reward = 0.0

#     # [-1, 1]
#     r_distance = (2 * initial_distance_to_goal) / (initial_distance_to_goal + distance_to_goal) - 1

#     # [-1.8, 0]
#     r_angle = -1 * abs(angle_to_goal)

#     # [-20, 0]
#     if min_obstacle_distance < 0.20:
#         r_obstacle = -20
#     else:
#         r_obstacle = 0

#     # [-4, 0]
#     r_vangular = -1 * (angular_velocity**2)

#     # [-2 * (2.2^2), 0]
#     r_vlinear = -1 * (((0.22 - linear_velocity) * 10) ** 2)

#     # [0, 1000]
#     if reached_waypoint == False:
#         r_success = 0
#     else: 
#         r_success = 1000
    
#     # [-2000, 0]
#     if terminated == False:
#         r_terminated = 0
#     else:
#         r_terminated = -2000



#     reward = reward + (2 * r_distance) + (1 * r_angle) + r_obstacle + r_success  + r_terminated
#     #  reward = reward + r_distance + r_angle + r_obstacle + r_vangular + r_vlinear + r_success  + r_terminated
#     return reward

# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
#     # Reward for reducing distance to the goal
#     r_distance = np.log(initial_distance_to_goal + 1) - np.log(distance_to_goal + 1)

#     # Reward for maintaining alignment to the goal
#     r_angle = -0.5 * abs(angle_to_goal)

#     # Penalty for proximity to obstacles
#     r_obstacle = -1 / (min_obstacle_distance + 0.01) if min_obstacle_distance < 0.5 else 0

#     # Penalty for excessive angular velocity
#     r_vangular = -0.5 * (angular_velocity**2) if abs(angular_velocity) > 0.5 else 0

#     # Reward for forward motion
#     r_vlinear = 2 * linear_velocity

#     # Reward for reaching the goal
#     r_success = 100 if reached_waypoint else 0

#     # Penalty for termination
#     r_terminated = -100 if terminated else 0

#     # Combine all components
#     reward = (2 * r_distance) + (1 * r_angle) + r_obstacle + r_vangular + r_vlinear + r_success + r_terminated 

#     return reward
