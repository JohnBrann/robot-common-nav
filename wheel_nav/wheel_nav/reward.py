

def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance, 
                reached_waypoint, terminated, linear_velocity, angular_velocity):
    # Initial reward
    reward = 0.0

    # [-1, 1]
    r_distance = (2 * initial_distance_to_goal) / (initial_distance_to_goal + distance_to_goal) - 1

    # [-1.8, 0]
    r_angle = -1 * abs(angle_to_goal)

    # [-20, 0]
    if min_obstacle_distance < 0.20:
        r_obstacle = -20
    else:
        r_obstacle = 0

    # [-4, 0]
    r_vangular = -1 * (angular_velocity**2)

    # [-2 * (2.2^2), 0]
    r_vlinear = -1 * (((0.22 - linear_velocity) * 10) ** 2)

    # [0, 1000]
    if reached_waypoint == False:
        r_success = 0
    else: 
        r_success = 1000
    
    # [-2000, 0]
    if terminated == False:
        r_terminated = 0
    else:
        r_terminated = -2000

    reward = reward + r_distance + r_angle + r_obstacle + r_vangular + r_vlinear + r_success  + r_terminated
    return reward