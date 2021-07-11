import sys
import time

import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg._odometry import Odometry
from std_msgs.msg import Float64


class MovementDetector(Node):

    def __init__(self):
        super().__init__('movement_detector_mpo_node')
        self._moved_distance = Float64()
        self._moved_distance.data = 0.0
        self._current_position = None
        self._initial_position_found_flag = False
        # self.get_init_position()

        self.distance_moved_pub = self.create_publisher(Float64, '/moved_distance', 1)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

    # def subscribe_message(self, msg):
    #     # self.get_logger().info(str(msg))
    #     pass
    #
    # def subscribe_message_test1(self, msg):
    #     # self.get_logger().info(str(msg.pose.pose.position))
    #     # self._current_position = Point()
    #     # self._current_position.x = msg.pose.pose.position.x
    #     # self.get_logger().info(str(self._current_position.x))
    #     if msg.pose.pose.position != None:
    #         self._initial_position_found_flag = True
    #         print(msg)
    #
    #
    # def get_init_position(self):
    #     data_odom = None
    #     while data_odom is None:
    #         try:
    #             data_odom = self.add_waitable()
    #             data_odom = self.create_subscription(Odometry,'/odom',self.subscribe_message_test1,1)
    #
    #         except Exception as e:
    #             print("error: ",e)
    #

    def odom_callback(self, msg):
        if self._current_position is None:
            self._current_position = msg.pose.pose.position

        New_Position = msg.pose.pose.position
        self._moved_distance.data += self.calculate_distance(New_Position, self._current_position)
        self.update_current_position(New_Position)
        if self._moved_distance.data < 0.00001:
            aux = Float64()
            aux.data = 0.0
            self.distance_moved_pub.publish(aux)
        else:
            self.distance_moved_pub.publish(self._moved_distance)

    def update_current_position(self, new_Position):
        self._current_position.x = new_Position.x
        self._current_position.y = new_Position.y
        self._current_position.z = new_Position.z

    def calculate_distance(self, new_Position, old_Position):
        x2 = new_Position.x
        x1 = old_Position.x
        y2 = new_Position.y
        y1 = old_Position.y
        dist = math.hypot(x2 - x1, y2 - y1)

        return dist


def main(args=None):
    rclpy.init(args=args)
    movement_detector = MovementDetector()
    rclpy.spin(movement_detector)
    movement_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
