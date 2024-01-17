# EVCS: Electric Vehicle Charging Station
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
# UInt16 선정 이유: object까지의 라이다, 초음파센서 거리값, 아르코마크 ID값을 받아 올 것이므로
from std_msgs.msg import String, UInt16

from tf_transformations import euler_from_quaternion

import numpy as np
import math
import time
from datetime import timedelta

class EVCSNavigator(Node) :
    def __init__(self) :
        super().__init__('EVCS_navigator')
        self.subscriber_charge_pillar_id = self.create_subscription(String, '/charge_pillar_id', self.charge_pillar_id_callback, 10)
        # self.subscriber_vision_detection_result = self.create_subscription(String, '/detection_result', self.vision_detection_result_callback, 10)
        # self.subscriber_lidar_depth = self.create_subscription(UInt16, '/lidar_depth', self.lidar_depth_callback, 10)
        # self.subscriber_arucomarker_id = self.create_subscription(UInt16, '/arucomarker_id', self.arucomarker_id, 10)

        # self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

        self.goal = []
        self.navigator = BasicNavigator()

    def charge_pillar_id_callback(self, msg):
        self.goal = []
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Waiting for Navigator Start')

        if msg.data == 'robot_0':
            self.goal_charge_hub_0(self.navigator, self.goal)
        elif msg.data in {'0', '1', '2', '3'}:
            self.goal_charge_pillar(self.navigator, self.goal, int(msg.data))
        elif msg.data in {'0', '1', '2', '3'}:
            self.goal_charge_pillar(self.navigator, self.goal, int(msg.data))
        else:
            self.get_logger().warn(f'Invalid pillar ID: {msg.data}')

        self.get_logger().info(f'Configured Navigation Goal: {msg.data}')

        # print(self.goal)
        # self.navigator.goToPose(self.goal)
        self.navigator.goThroughPoses(self.goal)
        self.get_logger().info('Moving to Goal')

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()

            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
            
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
                self.navigator.cancelTask()

            if not feedback:
                self.get_logger().info('No feedback received.')

        # # 여러 목표 지점들을 차례대로
        # i = 0
        # for goal in self.goals:
        #     print(goal)
        #     self.navigator.goThroughPoses([goal])
        #     self.get_logger().info('Moving to Goal')

        #     while not self.navigator.isTaskComplete():
        #         i = i + 1
        #         feedback = self.navigator.getFeedback()
        #         # print(feedback)

        #         if feedback and i % 5 == 0:
        #             print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
                
        #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=10.0):
        #             self.navigator.cancelTask()
                
        #         if not feedback:
        #             self.get_logger().info('No feedback received.')

        result = self.navigator.getResult()

        result_messages = {
            TaskResult.SUCCEEDED: 'Goal achieved successfully!',
            TaskResult.CANCELED: 'Goal was canceled!',
            TaskResult.FAILED: 'Goal failed!',
        }

        print(result_messages.get(result))

    def create_goal_pose(self, navigator, position_x, position_y, position_z, orientation_x,  orientation_y, orientation_z, orientation_w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = position_z
        goal_pose.pose.orientation.x = orientation_x
        goal_pose.pose.orientation.y = orientation_y
        goal_pose.pose.orientation.z = orientation_z
        goal_pose.pose.orientation.w = orientation_w

        return goal_pose

    def goal_charge_hub_0(self, navigator, goal):
        goal_pose = self.create_goal_pose(navigator, 0.239762048683475, -1.2045658922123377, 0.001021641637081884, 0.9999983326642647)
        goal.append(goal_pose)

    def goal_charge_pillar(self, navigator, goal, pillar_id):
        pillar_positions = [
            (0.6356891087237043, -0.8164219814925002, 0.0, 0.0, 0.0, 0.09352443469149051, 0.995616984646022),
            (1.401678884726616, -0.7519872729787022, 0.0, 0.0, 0.0, 0.9991115499496068, 0.04214392906806647),
            (1.3725857049388175, -0.4437872703170306, 0.0, 0.0, 0.0, -0.9982873476374894, 0.058501038853224364),
            (0.5561160143525906, -0.5244608377884499, 0.0, 0.0, 0.0, 0.03360435697535025, 0.9994352141045828)
        ]
        goal_pose = self.create_goal_pose(navigator, *pillar_positions[pillar_id])
        goal.append(goal_pose)

def main(args=None):
    rclpy.init(args=args)

    EVCS_navigator = EVCSNavigator()

    rclpy.spin(EVCS_navigator)

    EVCS_navigator.navigator.lifecycleShutdown()
    EVCS_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()