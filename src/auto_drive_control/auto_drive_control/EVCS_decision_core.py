import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from std_msgs.msg import String, Bool
from vision_object_detector_msgs.msg import DetectionResultArray

class EVCSDecisionCore(Node):
    def __init__(self):
        super().__init__('navigation_control_node')
        self.subscriber_robot_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_timer_callback, 10)
        self.subscriber_goal = self.create_subscription(String, '/goal', self.goal_timer_callback, 10)
        self.subscriber_obstacle_found = self.create_subscription(Bool, '/obstacle_found', self.obstacle_found_callback, 10)
        self.subscriber_is_docking_area = self.create_subscription(Bool, '/is_docking_area', self.is_docking_area_callback, 10)

        self.publisher_control_event = self.create_publisher(String, '/control_event', 10)

        self.hub_coordinates = [
            Point(x = -0.12973056733608246, y = -0.2408529669046402, z = -0.001434326171875),
            Point(x = -0.012470369227230549, y = -1.5600498914718628, z = -0.005340576171875),
            Point(x = 0.475780725479126, y = -1.5604393482208252, z = -0.005340576171875),
            Point(x = 0.4685480296611786, y = -0.2408529669046402, z = -0.001434326171875)
        ]

        self.navigator = BasicNavigator()

        self.control_start_event = ''  # charging_hub = 0, parking_zone = 1, obstacle_found = 2, obstacle_navi_restart = 3, docking_area = 4
        self.save_goal = String()
        self.obstacle_found = False
        self.obstacle_navi_restart = False
        self.is_docking_area = False

    def pose_timer_callback(self, msg):
        control_msg = String()
        is_inside_parking_zone = self.is_inside_parking_zone(msg)

        if not is_inside_parking_zone:
            self.control_start_event = '0'  # charging_hub
            # self.navigator.cancelTask()
        elif is_inside_parking_zone:
            self.control_start_event = '1'  # parking_zone
        elif self.obstacle_found:
            self.control_start_event = '2'  # obstacle_found
            # self.navigator.cancelTask()
        elif self.obstacle_navi_restart:
            self.control_start_event = '3,' + self.save_goal.data  # obstacle_navi_restart
        elif self.is_docking_area:
            self.control_start_event = '4'  # docking_area

        control_msg.data = self.control_start_event
        self.publisher_control_event(control_msg)
        print(f'Control_start_event: {self.control_start_event}')
    
    def goal_timer_callback(self, msg: String):
        self.save_goal = msg

    def obstacle_found_callback(self, msg: Bool):
        self.obstacle_found = msg
        
        if not self.obstacle_found:
            self.obstacle_navi_restart = True

    def is_docking_area_callback(self, msg: Bool):
        self.is_docking_area = msg

    def is_inside_parking_zone(self, pose_msg):
        current_x = pose_msg.pose.pose.position.x
        current_y = pose_msg.pose.pose.position.y

        return self.check_point_inside_polygon(current_x, current_y, self.hub_coordinates)

    def check_point_inside_polygon(self, x, y, polygon):
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0].x, polygon[0].y
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n].x, polygon[i % n].y
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            x_intersect = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= x_intersect:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

def main(args=None):
    rclpy.init(args=args)

    EVCS_decision_core = EVCSDecisionCore()
    
    rclpy.spin(EVCS_decision_core)

    EVCS_decision_core.navigator.lifecycleShutdown()
    EVCS_decision_core.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()