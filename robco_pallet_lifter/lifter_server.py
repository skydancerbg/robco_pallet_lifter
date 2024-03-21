#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from robco_interfaces.action import LifterControl
import lgpio

class LifterServer(Node):
    def __init__(self):
        super().__init__('lifter_server')

        # Initialize GPIO pins
        self.extract_pin = 4
        self.retract_pin = 17
        self.gpio = lgpio.gpiochip_open(0)

        # Claim GPIO pins as outputs
        lgpio.gpio_claim_output(self.gpio, self.extract_pin)
        lgpio.gpio_claim_output(self.gpio, self.retract_pin)

        # Create action server
        self.action_server = ActionServer(self, LifterControl, 'lifter_control', self.execute_callback)
        self.shutting_down = False

    async def execute_callback(self, goal_handle):
        # Extract action type from the goal
        action_type = goal_handle.request.action_type

        # Execute action based on action type
        if action_type == 4:  # Extend
            lgpio.gpio_write(self.gpio, self.extract_pin, 1)
            lgpio.gpio_write(self.gpio, self.retract_pin, 0)
            self.get_logger().info('EXECUTING Extend')
        elif action_type == 2:  # Retract
            lgpio.gpio_write(self.gpio, self.extract_pin, 0)
            lgpio.gpio_write(self.gpio, self.retract_pin, 1)
            self.get_logger().info('EXECUTING Retract')
        elif action_type == 1:  # Stop
            lgpio.gpio_write(self.gpio, self.extract_pin, 0)
            lgpio.gpio_write(self.gpio, self.retract_pin, 0)
            self.get_logger().info('EXECUTING Stop')

        # Mark the goal as succeeded
        goal_handle.succeed()

        # Return the result (not used in this example)
        return LifterControl.Result()

    def cleanup(self):
        # Set GPIO pins to 0
        lgpio.gpio_write(self.gpio, self.extract_pin, 0)
        lgpio.gpio_write(self.gpio, self.retract_pin, 0)
        self.get_logger().info('GPIO pins set to 0')

def main(args=None):
    rclpy.init(args=args)
    node = LifterServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
        node.shutting_down = True
        node.cleanup()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
