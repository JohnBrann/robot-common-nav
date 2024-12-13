

def calc_reward(distance_to_goal, initial_distance_to_goal, angle_to_goal, min_obstacle_distance):
    reward = 0.0

    # [-1, 1]
    r_distance = (2 * initial_distance_to_goal) / (initial_distance_to_goal + distance_to_goal) - 1
    # r_distance = -distance_to_goal / initial_distance_to_goal

    # [-1.8, 0]
    r_angle = -1 * abs(angle_to_goal)

    # [-20, 0]
    if min_obstacle_distance < 0.20:
        r_obstacle = -20
    else:
        r_obstacle = 0


    reward = reward + r_distance + r_angle + r_obstacle

    return reward