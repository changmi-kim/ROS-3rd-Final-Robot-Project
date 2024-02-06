# EVCS: Electric Vehicle Charging Station
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import numpy as np
import math
import time
from datetime import timedelta

class EVCSNavigator(Node):
    def __init__(self):
        super().__init__('EVCS_navigator')
        self.subscriber_charge_pillar_id = self.create_subscription(String, '/charge_pillar_id', self.charge_pillar_id_callback, 10)
        # self.subscriber_vision_detection_result = self.create_subscription(String, '/detection_result', self.vision_detection_result_callback, 10)
        # self.subscriber_lidar_depth = self.create_subscription(UInt16, '/lidar_depth', self.lidar_depth_callback, 10)
        # self.subscriber_arucomarker_id = self.create_subscription(UInt16, '/arucomarker_id', self.arucomarker_id, 10)

        # self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def charge_pillar_id_callback(self, msg):
        if msg.data == 'robot_0':
            self.goal_charge_hub_0()
        elif msg.data in {'0', '1', '2', '3'}:
            self.goal_charge_pillar(int(msg.data))
        else:
            self.get_logger().warn(f'Invalid pillar ID: {msg.data}')

    def create_goal_pose(self, position_x, position_y, position_z, orientation_x,  orientation_y, orientation_z, orientation_w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = position_z
        goal_pose.pose.orientation.x = orientation_x
        goal_pose.pose.orientation.y = orientation_y
        goal_pose.pose.orientation.z = orientation_z
        goal_pose.pose.orientation.w = orientation_w

        return goal_pose

    def goal_charge_hub_0(self):
        goal_pose = self.create_goal_pose(0.239762048683475, -1.2045658922123377, 0.001021641637081884, 0.9999983326642647)
        self.send_goal(goal_pose)

    def goal_charge_pillar(self, pillar_id):
        pillar_positions = [
            (0.6356891087237043, -0.8164219814925002, 0, 0, 0, 0.09352443469149051, 0.995616984646022),
            (1.401678884726616, -0.7519872729787022, 0, 0, 0, 0.9991115499496068, 0.04214392906806647),
            (1.3725857049388175, -0.4437872703170306, 0, 0, 0, -0.9982873476374894, 0.058501038853224364),
            (0.5561160143525906, -0.5244608377884499, 0, 0, 0, 0.03360435697535025, 0.9994352141045828)
        ]
        goal_pose = self.create_goal_pose(*pillar_positions[pillar_id])
        self.send_goal(goal_pose)

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'Sending goal: {goal_pose}')
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result().result
            self.get_logger().info(f'Goal result: {result}')

def main(args=None):
    rclpy.init(args=args)

    EVCS_navigator = EVCSNavigator()

    rclpy.spin(EVCS_navigator)

    EVCS_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
