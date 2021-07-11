import rclpy
import json 
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg._odometry import Odometry

# this part can receive odometry message by subscribing to /odom topic

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('py_sub_spiral_node')
        # self.subscriber_ = self.create_subscription(Twist, '/cmd_vel', self.subscribe_message, 1)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.subscribe_message, 1)
        self.subscriber_  # prevent unused variable warning

    def subscribe_message(self, msg):
        # self.get_logger().info('Recieved - Linear Velocity : %f, Angular Velocity : %f' % (msg.linear.x, msg.angular.z))
        self.get_logger().info(str(msg.header))


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

###this part can receive odometry message by subscribing to /odom topic

# this part calculate the movement and distance from the topic /odom



### this part calculate the movement and distance from the topic /odom
