import numpy as np

def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
                reached_waypoint, terminated, linear_velocity, angular_velocity):
    """
    Reward function for TurtleBot navigation.
    
    - Encourages getting closer to the goal.
    - Penalizes collisions and excessive angular velocity.
    - Rewards reaching the waypoint.
    - Encourages safe distances from obstacles.
    """

    # Initialize reward
    reward = 0.0

    # Reward for progress towards the goal
    progress_reward = (initial_distance_to_goal - distance_to_goal) * 10.0
    reward += progress_reward

    # Penalize large deviation from goal direction
    angle_penalty = -abs(angle_to_goal) * 0.5
    reward += angle_penalty

    # Collision penalty
    if min_obstacle_distance < 0.15:  # Threshold for collision risk
        reward -= 50.0

    # Encourage smooth movement
    velocity_reward = linear_velocity * 5.0  # Encourages forward movement
    reward += velocity_reward

    # Penalize excessive spinning
    angular_penalty = -abs(angular_velocity) * 10.0
    reward += angular_penalty

    # Reward reaching the waypoint
    if reached_waypoint:
        reward += 200.0  # Large reward for success

    # Termination penalty
    if terminated:
        reward -= 100.0  # Large penalty for failure

    return reward


# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
    
#     reward = 0.0
    
#     r_vlinear = linear_velocity
#     if linear_velocity > 0.05:
#         r_vlinear = 0.5
#         if linear_velocity > 0.1:  # and linear_velocity < 0.9:
#             r_vlinear = 1.0
#             if linear_velocity > 0.18:
#                 r_vlinear = 3.0
#     else: 
#         r_vlinear = -1.0
#     # r_vlinear = linear_velocity


#     if abs(angular_velocity) > 0.8:
#         r_vangular = 5.0 
#     elif abs(angular_velocity) > 0.4:
#         r_vangular = 3.0
#     elif abs(angular_velocity) > 0.2:
#         r_vangular = 1
#     else:
#         r_vangular = 0

#     # r_vangular = 2.0 * abs(angular_velocity)

#     reward = r_vlinear - r_vangular
#     return round(reward, 2)

#     # reward = 10.0

#     # return reward



# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
    
#     # Define max values for normalization
#     max_obstacle_range = 3.0  # Assume max LiDAR range
#     max_speed = 0.22  # Max linear speed for TurtleBot3
#     max_angular_speed = 1.0  # Max angular speed for TurtleBot3
    
#     # Normalize inputs
#     norm_distance = distance_to_goal / initial_distance_to_goal  # 0 (goal) to 1 (start)
#     norm_angle = abs(angle_to_goal) / 3.14  # 0 (aligned) to 1 (180° off)
#     norm_obstacle = min(1.0, min_obstacle_distance / max_obstacle_range)  # 0 (close) to 1 (safe)
#     norm_linear_vel = linear_velocity / max_speed  # 0 (stopped) to 1 (max forward)
#     norm_angular_vel = abs(angular_velocity) / max_angular_speed  # 0 (stable) to 1 (max spin)
    
#     reward = 0.0

#     # Reward progress toward goal
#     reward += 15 * (1 - norm_distance)  # Higher reward as distance decreases

#     # Penalize large angle deviations
#     reward -= norm_angle * 5  

#     # Penalize excessive rotation
#     reward -= (norm_angular_vel ** 1.5) * 3  

#     # Reward stable movement, penalize excessive spinning
#     if norm_angular_vel > 0.5:
#         reward -= 10  # High penalty for spinning too fast
#     elif norm_angular_vel < 0.1:
#         reward += 5  # Small reward for stability

#     # Encourage forward motion
#     reward += norm_linear_vel * 10  
#     if norm_linear_vel < 0.2:
#         reward -= 5  # Penalize slow movement

#     # Gradual penalty for being close to obstacles
#     if norm_obstacle < 0.3:
#         reward -= (1 / (norm_obstacle + 0.01)) * 10  # More penalty for closer obstacles

#     # High reward for reaching the waypoint
#     if reached_waypoint:
#         reward += 200  

#     # Penalty for termination (collision or timeout)
#     if terminated:
#         reward -= 200  

#     return round(reward, 2)







# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
#     reward = 0.0

#     # Reward for moving toward the goal (normalized)
#     if distance_to_goal < initial_distance_to_goal:
#         reward += 15 * (1 - (distance_to_goal / initial_distance_to_goal))

#     # Penalize deviation from goal direction (more when farther from goal)
#     reward -= (abs(angle_to_goal) / 3.14) * (5 if distance_to_goal > 0.5 else 2)

#     # Penalize excessive rotation
#     reward -= (abs(angular_velocity) ** 1.5) * 3  # Harsher penalty for spinning

#     # Reward for reducing angular velocity when spinning too fast
#     if abs(angular_velocity) > 0.5:
#         reward -= 10  # Strong penalty for fast spinning
#     elif abs(angular_velocity) < 0.1:
#         reward += 5  # Small reward for stabilizing

#     # Encourage forward motion
#     reward += linear_velocity * 10  
#     if linear_velocity < 0.05:
#         reward -= 5  # Penalize standing still

#     # Gradual penalty for being too close to obstacles
#     if min_obstacle_distance < 0.3:
#         reward -= (1 / min_obstacle_distance) * 10  

#     # High reward for reaching the waypoint
#     if reached_waypoint:
#         reward += 200  

#     # Penalty for termination (collision or timeout)
#     if terminated:
#         reward -= 100  

#     return round(reward, 2)

# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
#     # Initialize reward
#     reward = 0.0

#     # Reward for progress toward the goal
#     if distance_to_goal < initial_distance_to_goal:
#         reward += (initial_distance_to_goal - distance_to_goal) * 10  # Scaled reward for getting closer to the goal

#     # Penalty for deviation from goal alignment
#     reward -= abs(angle_to_goal) * 2  # Penalize larger angular deviation

#     # Penalty for proximity to obstacles
#     if min_obstacle_distance < 0.25:
#         reward -= 20  # Harsh penalty for being too close to obstacles

#     # Rewards or penalties based on velocities
#     # reward -= abs(angular_velocity) * 2  # Penalize excessive angular velocity
#     # reward += linear_velocity * 10  # Reward forward motion (assuming positive linear velocity)
#     if linear_velocity < 0.15:
#         reward -= 1

#     # Reward for success
#     if reached_waypoint:
#         reward += 200  # High reward for successfully reaching the waypoint

#     # Penalty for termination
#     if terminated:
#         reward -= 100  # Penalty for termination (e.g., collision or timeout)


#     reward = round(reward, 2)
#     # Return the calculated reward
#     return reward


# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
#     # Initialize reward
#     reward = 0.0

#     # Reward for progress toward the goal
#     if distance_to_goal < initial_distance_to_goal:
#         reward += (initial_distance_to_goal - distance_to_goal) * 5  # Scaled reward for getting closer to the goal

#     # Penalty for deviation from goal alignment
#     reward -= abs(angle_to_goal) * 5  # Penalize larger angular deviation

#     # Penalty for proximity to obstacles
#     if min_obstacle_distance < 0.25:
#         reward -= 20  # Harsh penalty for being too close to obstacles

#     # Rewards or penalties based on velocities
#     reward -= abs(angular_velocity) * 2  # Penalize excessive angular velocity
#     reward += linear_velocity * 2  # Reward forward motion (assuming positive linear velocity)
#     # if linear_velocity < 0.15:
#     #     reward -= 1

#     reward -= (((0.22 - linear_velocity) * 10) ** 2)

#     # Reward for success
#     if reached_waypoint:
#         reward += 400  # High reward for successfully reaching the waypoint

#     # Penalty for termination
#     if terminated:
#         reward -= 100  # Penalty for termination (e.g., collision or timeout)


#     reward = round(reward, 2)
#     # Return the calculated reward
#     return reward



# def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
#                 reached_waypoint, terminated, linear_velocity, angular_velocity):
#     # Initial reward
#     reward = 0.0

#     # [-1, 1]
#     r_distance = (2 * initial_distance_to_goal) / (initial_distance_to_goal + distance_to_goal) - 1

#     # [-1.8, 0]
#     r_angle = -2 * abs(angle_to_goal)

#     # [-20, 0]
#     if min_obstacle_distance < 0.20:
#         r_obstacle = -20
#     else:
#         r_obstacle = 0

#     # [-4, 0]
#     r_vangular = -2 * (angular_velocity**2)

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



#     # reward = reward + (2 * r_distance) + (1 * r_angle) + r_obstacle + r_success  + r_terminated
#     reward = reward + r_distance + r_angle + r_obstacle + r_vangular + r_vlinear + r_success  + r_terminated
#     return round(reward, 2)

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
#     reward = (1 * r_distance) + (1 * r_angle) + r_obstacle + r_vangular + r_vlinear + r_success + r_terminated 

#     return reward
