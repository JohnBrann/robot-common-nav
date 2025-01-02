# This message stores important data used in reinforcement learning.
# Most data is used for when calculating rewards, learning, and testing

float32 distance_to_goal
float32 angle_to_goal
float32[] scan_data
float32 min_obstacle_distance
float32 linear_velocity
float32 angular_velocity
bool success
bool terminated
float32 reward