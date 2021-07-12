"""
    this program subscribe to odom topic and store date in a dictionary. if stopped with keyboard interrupt using
    ctrl+C the the data is stored as json.

"""

import rclpy
import json
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg._odometry import Odometry
from std_msgs.msg import Float64



# this part can receive odometry message by subscribing to /odom topic

class MinimalSubscriber(Node):
    # data_obj = {}  # gets populated with each callback on /odom topic and used for json dump on keyboard interrupt

    def __init__(self):
        super().__init__('py_sub_spiral_node')
        self.data_obj = {}
        self.current_odom = None
        self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.subscribe_message, 1)
        self.subscriber_dist_ = self.create_subscription(Float64, '/moved_distance', self.update_dist_odom, 1)
        self.odom_count_ = 0
        print("initialized____")

    def update_dist_odom(self,msg):
        if msg.data > 0.0:
            self.odom_count_+=1
            # print("robot moving"+ str(msg))
            self.data_obj[str(self.odom_count_)] = {'position': {'x': self.current_odom.pose.pose.position.x,
                                                                  'y': self.current_odom.pose.pose.position.y}}
            print("data added")
        else:
            print("idle")

    def subscribe_message(self, msg):
        self.current_odom = msg
        # self.odom_count_ += 1
        # MinimalSubscriber.data_obj[str(self.odom_count_)] = {'position': {'x': msg.pose.pose.position.x,
        #                                                                   'y': msg.pose.pose.position.y}}


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        with open('allodoms.json', 'w') as jsonFile:
            # json.dump(MinimalSubscriber.data_obj, jsonFile, indent=4)
            json.dump(minimal_subscriber.data_obj, jsonFile, indent=4)
            jsonFile.close()


if __name__ == '__main__':
    main()
