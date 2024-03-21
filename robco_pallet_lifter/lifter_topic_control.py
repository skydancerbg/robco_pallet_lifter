#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from robco_interfaces.action import LifterControl
from std_msgs.msg import String, UInt8

class TopicClient:
    def __init__(self):
        self.node = rclpy.create_node('lifter_topic_control')
        self.action_client = ActionClient(self.node, LifterControl, 'lifter_control')
        self.subscription = self.node.create_subscription(String, '/robco_lifter_command', self.callback, 10)
        # self.node.get_logger().info('lifter_topic_control lifter_server client node started')
        # self.node.get_logger().info('Valid std_msgs/String commands on /robco_lifter_command topic are:')
        # self.node.get_logger().info('\"extend\", \"retract\" and \"stop\"')
        print()
        print('  lifter_topic_control pallet lifter action server client node started')
        print()
        print('Valid std_msgs/String commands on /robco_lifter_command topic are:')
        print('                   \"extend\", \"retract\" and \"stop\"')
        print()



    def callback(self, msg):
        action_type = self.convert_message_to_action_type(msg.data)
        if action_type is not None:
            self.send_goal(action_type)
            self.node.get_logger().info(f'Sending goal request \"{msg.data}\" to the action server')

        else:
            # self.node.get_logger().warn('Received unknown message: %s' % msg.data)
            self.node.get_logger().warn('Invalid value received on /robco_lifter_command topic: %s' % msg.data)
            # self.node.get_logger().info('Valid std_msgs/String values on /robco_lifter_command topic are:')
            # self.node.get_logger().info('\"extend\", \"retract\" and \"stop\": %s' % msg.data)
            print('Valid std_msgs/String values on /robco_lifter_command topic are:')
            print('\"extend\", \"retract\" and \"stop\": %s' % msg.data)

    # def topic_callback(self, msg):
    #     action_type = msg.data
    #     if action_type in [1, 2, 3]:
    #         self.get_logger().info(f'Received value: {action_type}, sending goal request...')
    #         self.send_goal(action_type)
    #     else:
    #         self.get_logger().info(f'Invalid value received: {action_type}')


    def convert_message_to_action_type(self, message):
        if message == 'extend':
            return 4
        elif message == 'retract':
            return 2
        elif message == 'stop':
            return 1
        else:
            return None

    def send_goal(self, action_type):
        goal_msg = LifterControl.Goal()
        goal_msg.action_type = action_type
        # self.node.get_logger().info('Sending goal message with action type: %d' % action_type)
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('Goal rejected')
        else:
            self.node.get_logger().info('Goal accepted')

def main(args=None):
    rclpy.init(args=args)
    node = TopicClient()

    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        node.node.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        if rclpy.ok():
            node.node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

