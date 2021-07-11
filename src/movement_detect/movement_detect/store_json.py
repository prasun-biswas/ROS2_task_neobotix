"""
    this program subscribe to odom topic and store date in a dictionary. if stopped with keyboard interrupt using
    ctrl+C the the data is stored as json.
    the commented out section block from line: 27 to line:45 can be used to slow down the subscription rate which
    is also used in store_goal_waypoint.py to get odom value at a time interval.

"""

import rclpy
import json
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg._odometry import Odometry


# this part can receive odometry message by subscribing to /odom topic

class MinimalSubscriber(Node):
    data_obj = {}  # gets populated with each callback on /odom topic and used for json dump on keyboard interrupt

    def __init__(self):
        super().__init__('py_sub_spiral_node')
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.subscribe_message, 1)
        self.odom_count_ = 0

        # self.subscriber_Odom_ = self.create_subscription(Odometry, '/odom', self.update_odom_value, 1)
        # self.subscriber_ = self.create_subscription(Odometry, '/odom_at_interval', self.subscribe_message, 1)
        # self.publisher_Odom_ = self.create_publisher(Odometry, '/odom_at_interval', 1)
        # timer_period = 0.2  # this controls the time interval form odometry value to be published
        # self.publish_odom_timer_ = self.create_timer(timer_period, self.publish_message) # controls publish rate on /odom_at_interval

        # def update_odom_value(self, msg):
        #     """ continuously updates the odom value from topic /odom
        #     """
        #     self.current_odom = msg
        #     # print("odom received")

        # def publish_message(self):
        #     """ publishes odometry value with publisher_Odom_ when called by self.timer_
        #     """
        #     if self.current_odom != None:
        #         message = self.current_odom
        #         self.publisher_Odom_.publish(message)
        #         # print('\n published on odom_at_interval \n')

    def subscribe_message(self, msg):
        self.odom_count_ += 1
        MinimalSubscriber.data_obj[str(self.odom_count_)] = {'position': {'x': msg.pose.pose.position.x,
                                                                          'y': msg.pose.pose.position.y,
                                                                          'time': msg.header.stamp}}


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        with open('allodoms.json', 'w') as jsonFile:
            json.dump(MinimalSubscriber.data_obj, jsonFile, indent=4)
            jsonFile.close()


if __name__ == '__main__':
    main()
