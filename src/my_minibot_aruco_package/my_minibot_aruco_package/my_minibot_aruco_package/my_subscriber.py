import rclpy as rp
from rclpy.node import Node

from turtlesim.msg import Pose


class MinibotSubscriber(Node):
	def __init__(self):
		super().__init__('minibit_subscriber')
		self.subscription = self.create_subscription(
			Pose,
			'/minibot/pose',
			self.callback,10)
		self.subscription

	def callback(self,msg):
		print("X : ", msg.x, ",Y : ", msg.y)


def main(args = None):
	rp.init(args = args)

	turtlesim_subscriber = MinibotSubscriber()
	rp.spin(turtlesim_subscriber)

	turtlesim_subscriber.destroy_node()
	rp.shutdown()

if __name__ =='__main__':
	main()

