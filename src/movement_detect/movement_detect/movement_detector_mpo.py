
import rclpy
from rclpy.node import Node
import math
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

    def odom_callback(self, msg):
        if self._current_position is None:
            self._current_position = msg.pose.pose.position
        New_Position = msg.pose.pose.position
        self._moved_distance.data += self.calculate_distance(New_Position, self._current_position)
        self.update_current_position(New_Position)
        if self._moved_distance.data < 0.001:
            aux = Float64()
            aux.data = 0.0
            self.distance_moved_pub.publish(aux)
            # print("published dist: "+str(aux))
            # self.get_logger().info(str(msg.header))

        else:
            self.distance_moved_pub.publish(self._moved_distance)
            # print("published dist: "+str(self._moved_distance))


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
        # a tolerance value of 0.001 is selected to adjust with very small garbage value published by the simulator
        # even when the robot is not moving.
        if dist > 0.001:
            self.get_logger().info(str(self._moved_distance))
            return dist
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    movement_detector = MovementDetector()
    rclpy.spin(movement_detector)
    movement_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
