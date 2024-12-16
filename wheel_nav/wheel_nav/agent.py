import rclpy
from rclpy.node import Node
from wheel_nav_msgs.srv import Step

class Agent(Node):
    def __init__(self):
        super().__init__('agent')
        self.step_client = self.create_client(Step, "/step_service")

        # Wait for the service to become available
        while not self.step_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the Step service to become available...")

        self.max_episodes = 50
        self.max_steps = 2000

        # Timer to handle steps every 0.25 seconds
        # self.timer = self.create_timer(0.25, self.step_timer_callback)

    def run(self):
        for episode in range(self.max_episodes):
            # Initialize environment
            terminated = False
            step_count = 0
            episode_reward = 0
            success = False

            while not terminated and not success and step_count < self.max_steps:
                # Take a step and get step info
                env_info = self.step_request()

                if env_info is None:
                    self.get_logger().error("Failed to get response from Step service.")
                    break

                reward = env_info.reward
                terminated = env_info.terminated
                success = env_info.success
                # self.get_logger().info(f'Step {step_count} reward: {reward}')

                # Train
                episode_reward += reward
                step_count += 1

            self.get_logger().info(f'Episode {episode} reward {episode_reward}')

            # if success:
            #     self.get_logger().info(f'SUCCESS! Episode {episode} reward {episode_reward}')
            # elif terminated:
            #     self.get_logger().info(f'FAILURE! Episode {episode} reward {episode_reward}')
            

    def step_request(self):
        req = Step.Request()
        future = self.step_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed!")
            return None

def main(args=None):
    rclpy.init(args=args)
    agent = Agent()
    agent.run()
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
