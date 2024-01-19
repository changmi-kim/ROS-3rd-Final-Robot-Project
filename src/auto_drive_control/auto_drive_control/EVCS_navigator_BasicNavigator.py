# EVCS: Electric Vehicle Charging Station
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, UInt16

from tf_transformations import euler_from_quaternion

import numpy as np
import math
import time
from datetime import timedelta

class EVCSNavigator(Node) :
    def __init__(self) :
        super().__init__('EVCS_navigator')
        self.subscriber_goal = self.create_subscription(String, '/goal', self.goal_timer_callback, 10)
        # self.subscriber_arucomarker_id = self.create_subscription(UInt16, '/arucomarker_id', self.arucomarker_id, 10)

        self.get_logger().info('Waiting for Navigator Start')

        self.goal = []
        self.navigator = BasicNavigator()

    def goal_timer_callback(self, msg):
        self.goal = []

        # print("debug!!!!!")
        # BasicNavigator.waitUntilNav2Active()
        self.current_msg = msg

        if msg.data == 'c':
            self.navigator.cancelTask()
            self.get_logger().warn('Navigation execution canceled.')
        elif msg.data in {'o', 'i'}:
            self.goal_charge_hub_inout(msg.data)
            self.send_to_goal()
        elif msg.data in {'00', '01', '02'}:
            self.goal_charge_hub(int(msg.data))
            self.send_to_goal()
        elif msg.data in {'0', '1', '2', '3'}:
            self.goal_charge_pillar(int(msg.data))
            self.send_to_goal()
        else:
            self.get_logger().warn(f'Invalid Data: {msg.data}')
        
    def send_to_goal(self):
        if self.current_msg is not None:
            self.get_logger().info(f'Configured Navigation Goal: {self.current_msg.data}')

        # self.navigator.goThroughPoses(self.goal)
        # self.get_logger().info('Moving to Goal')

        # i = 0
        # while not self.navigator.isTaskComplete():
        #     i = i + 1
        #     feedback = self.navigator.getFeedback()

        #     if feedback and i % 5 == 0:
        #         print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
            
        #     if Duration.from_msg(feedback.navigation_time) > Duration(seconds=15.0):
        #         self.navigator.cancelTask()

        #     if not feedback:
        #         self.get_logger().info('No feedback received.')

        for goal_index, goal_pose in enumerate(self.goal, start=1):
            self.navigator.goToPose(goal_pose)
            self.get_logger().info(f'Moving to Goal({goal_index})')

            i = 0
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()

                if feedback and i % 5 == 0:
                    print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')

                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=10.0):
                    self.navigator.cancelTask()

                if not feedback:
                    self.get_logger().info('No feedback received.')

            self.get_logger().info(f'Goal {goal_index} passed successfully!')

        result = self.navigator.getResult()

        result_messages = {
            TaskResult.SUCCEEDED: 'Goal achieved successfully!',
            TaskResult.CANCELED: 'Goal was canceled!',
            TaskResult.FAILED: 'Goal failed!',
        }

        print(result_messages.get(result))

    def create_goal_pose(self, position_x, position_y, orientation_z, orientation_w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.orientation.z = orientation_z
        goal_pose.pose.orientation.w = orientation_w

        return goal_pose

    def goal_charge_hub_inout(self, hub_io):
        if hub_io == 'o':
            goal_pose = self.create_goal_pose(0.36135944724082947, -1.8798696994781494, 0.2099868162147609, 0.9777042175504759)
            self.goal.append(goal_pose)
        elif hub_io == 'i':
            hub_i_list = [(0.36685978020193416, 0.021821063240631013, -0.9916437223207283, 0.12900669743036722),
                          (-0.09214386734683028, 0.00018066425420607584, -0.7966940838329873, 0.6043827734023505),
                          (0.24503996661760177, -0.6289435257197941, -0.6565887096430582, 0.7542488093257184)]
            for i in range(len(hub_i_list)):
                goal_pose = self.create_goal_pose(*hub_i_list[i])                                 
                self.goal.append(goal_pose)

    def goal_charge_hub(self, hub_id):
        # goal_pose = self.create_goal_pose(navigator, 0.239762048683475, -1.2045658922123377, 0.001021641637081884, 0.9999983326642647)
        hub_positions = [
            (0.11041894462359753, -1.3425289515265082, -0.0037382883967639322, 0.9999930125755193),
            (0.08464269493819884, -1.0156754459932178, -0.0037382883967639322, 0.9999930125755193),
            (0.05308452747301681, -0.7240572198469294, -0.0037382883967639322, 0.9999930125755193)
        ]
        goal_pose = self.create_goal_pose(*hub_positions[hub_id])
        self.goal.append(goal_pose)

    def goal_charge_pillar(self, pillar_id):
        pillar_positions = [
            (0.6356891087237043, -0.8164219814925002, 0.09352443469149051, 0.995616984646022),
            (1.401678884726616, -0.7519872729787022, 0.9991115499496068, 0.04214392906806647),
            (1.3725857049388175, -0.4437872703170306, -0.9982873476374894, 0.058501038853224364),
            (0.5561160143525906, -0.5244608377884499, 0.03360435697535025, 0.9994352141045828)
        ]
        goal_pose = self.create_goal_pose(*pillar_positions[pillar_id])
        self.goal.append(goal_pose)

def main(args=None):
    rclpy.init(args=args)

    EVCS_navigator = EVCSNavigator()

    rclpy.spin(EVCS_navigator)


    # print("debug1")
    EVCS_navigator.navigator.lifecycleShutdown()
    # print("debug2")
    EVCS_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()