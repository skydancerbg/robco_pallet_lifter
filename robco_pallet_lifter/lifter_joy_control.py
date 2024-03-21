#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.action import ActionClient
from robco_interfaces.action import LifterControl

class LifterClient(Node):
    def __init__(self):
        super().__init__('lifter_joy_control')
        self.action_client = ActionClient(self, LifterControl, 'lifter_control')
        self.prev_button_states = [0, 0, 0, 0]  # Initialize previous button states for three buttons
        self.shutting_down = False
        print()
        print('   lifter_joy_control lifter action server client node started')
        print()
        print('                    Logitech F710 Gamepad:')
        print('  Hold the left or right back button while sending the commands!')
        print('             Button Y to extend the pallet lifter')
        print('             Button A to retract the pallet lifter')
        print('             Button X to stop the pallet lifter movement')
        print()


        # Subscribe to /joy topic for joystick input
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # F710 button indexes:
        # 0   X
        # 1   A
        # 2   B
        # 3   Y
        # 4   LeftBack
        # 5   RightBback
        # ....

        for i in range(4):  # Check each button
            button_state = msg.buttons[i]
            if button_state == 1 and self.prev_button_states[i] == 0 and i != 2 and (msg.buttons[4] == 1 or msg.buttons[5] == 1):  # skip button B with Index 2
                action_type = i + 1  # Set action_type based on button index
                self.get_logger().info(f'Sending goal: {action_type}')

                goal_msg = LifterControl.Goal()
                goal_msg.action_type = action_type

                future = self.action_client.send_goal_async(goal_msg)
                future.add_done_callback(self.goal_response_callback)

            self.prev_button_states[i] = button_state  # Update previous button state

        # if len(msg.buttons) >= 4:  # Assuming at least 3 buttons are present
        #     for i in range(4):  # Check each button
        #         button_state = msg.buttons[i]
        #         if button_state == 1 and self.prev_button_states[i] == 0 and i != 2:  # skip button B with Index 2
        #             action_type = i + 1  # Set action_type based on button index
        #             self.get_logger().info(f'Sending goal: {action_type}')

        #             goal_msg = LifterControl.Goal()
        #             goal_msg.action_type = action_type

        #             future = self.action_client.send_goal_async(goal_msg)
        #             future.add_done_callback(self.goal_response_callback)

        #         self.prev_button_states[i] = button_state  # Update previous button state
        # else:
        #     self.get_logger().warn('Not enough buttons in the message')

    def goal_response_callback(self, future):
        if not self.shutting_down:
            result = future.result()
            if result is not None:
                if result.accepted:
                    self.get_logger().info('Goal accepted by server')
                else:
                    self.get_logger().warn('Goal rejected by server')
            else:
                self.get_logger().error('Failed to get result from server')

def main(args=None):
    rclpy.init(args=args)
    node = LifterClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
        node.shutting_down = True
        rclpy.shutdown()

if __name__ == '__main__':
    main()
