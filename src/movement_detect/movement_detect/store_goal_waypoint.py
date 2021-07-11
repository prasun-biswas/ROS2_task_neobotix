""" this store_goal_waypoint.py file is dependent on movement_detector_mpo.py to get the message /moved_distance topic.
    value received from /moved_distance_topic is used here to calculate driven_distance before dump data as json for
    each new goal_point

    So, movement_detector_mpo.py should be run on terminal with "python3 movement_detector_mpo.py" command.
    Then, store_goal_waypoint.py should be run on terminal with "python3 store_goal_waypoint.py" command

"""
import rclpy
import json
import math
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg._odometry import Odometry
from nav_msgs.msg._path import Path
from visualization_msgs.msg._marker_array import MarkerArray
from std_msgs.msg import Float64



class GoalSubscriber(Node):
    # initialize with inheritance from ros2 Node class
    def __init__(self):
        super().__init__('GoalSubscriber_node')
        self.no_of_waypoint = 0  # value increases with each new goal is selected on rviz for autonomous navigation
        self.current_odom = None  # value gets updated from callback function: update_odom_value via topic /odom
        self.goal_point = Point()  # value gets updated from callback function: update_plan_path via topic /plan
        self.current_dist = Float64()  # value gets updated from callback function: update_dist via topic /moved_distance
        self.current_dist.data = 0.0
        self.subscriber_goal_waypoints_ = self.create_subscription(MarkerArray, '/waypoints', self.subscribe_message, 1)
        self.publisher_Odom_ = self.create_publisher(Odometry, '/odom_at_interval', 1)
        self.subscriber_Odom_ = self.create_subscription(Odometry, '/odom', self.update_odom_value, 1)
        self.subscriber_plan_ = self.create_subscription(Path, '/plan', self.update_plan_path, 1)
        self.subscriber_dist_ = self.create_subscription(Float64, '/moved_distance', self.update_dist, 1)
        timer_period = 0.2  # this controls the time interval form odometry value to be published
        self.publish_odom_timer_ = self.create_timer(timer_period, self.publish_message) # controls publish rate on /odom_at_interval

    def update_dist(self, msg):
        """ updates self.current_dist with each call from subscriber_dist_
        """
        # print(msg)
        self.current_dist = msg

    def update_plan_path(self, path_msg):
        """ updates self.goal_point with the last value of data from the topic /plan.
            plan receives a list of path data that the robot is supposed to follow
        """
        # print('received path:')
        index_goal_point = len(path_msg.poses) - 1
        self.goal_point = path_msg.poses[index_goal_point].pose.position
        # print(path_msg.poses[index_goal_point].pose.position)
        # print(self.goal_point)
        # print(' path print ended')

    def update_odom_value(self, msg):
        """ continuously updates the odom value from topic /odom
        """
        self.current_odom = msg
        # print("odom received")

    def publish_message(self):
        """ publishes odometry value with publisher_Odom_ when called by self.timer_
        """
        if self.current_odom != None:
            message = self.current_odom
            self.publisher_Odom_.publish(message)
            # print('\n published on odom_at_interval \n')

    def populate_Json(self, new_no_of_waypoint):
        """ gets called from if_goal_changed() and called only once with each new goal and performs all necessary
            calculations for that specific call only.

        """
        beginning_dist = self.current_dist  # value set with the self.current_dist at the beginning of new goal
        # print('inside populate json')
        data_odom = {}  # this dictionary gets populated with necessary data to be dumped as json
        idle_count_limit = 50  # if this count limit is reached with idle count get_message() is ready to dump json data
        idle_count = 0  # if co-ordinate don't change this gets increment
        mov_odom_count = 0  # increases with each non idle odom value and inserted as key for odom values
        process_finished = False  # flag to indicate if calculations for this specific populate_json call is completed

        def get_Message(new_msg):
            # print('getting value from Topic: odom_at_interval')
            # print("getting odom for: " + str(self.no_of_waypoint))
            nonlocal idle_count_limit, idle_count, process_finished, mov_odom_count
            curr_X = new_msg.pose.pose.position.x
            curr_Y = new_msg.pose.pose.position.y

            #  used a value of 0.001 for tolerance limit for twist
            if new_msg.twist.twist.linear.x > 0.001 and new_msg.twist.twist.linear.y > 0.001:
                idle_count = 0
                mov_odom_count += 1
                data_odom[str(mov_odom_count)] = {'position': {'x': curr_X,
                                                               'y': curr_Y}
                                                  }
                # print("odom added: " + str(self.no_of_waypoint) + "." + str(mov_odom_count))

            elif idle_count >= idle_count_limit and process_finished == False:
                process_finished = True

                if abs(self.goal_point.x - curr_X) < 0.1 and abs(self.goal_point.y - curr_Y) < 0.1:
                    data_odom["goal_reached"] = "True"
                else:
                    data_odom["goal_reached"] = "False"

                driven_dist = abs(beginning_dist.data - self.current_dist.data)
                data_odom["driven_distance"] = driven_dist

                wpn = 'example' + str(new_no_of_waypoint) + '.json'
                with open(wpn, 'w') as jsonFile:
                    json.dump(data_odom, jsonFile, indent=4)
                    jsonFile.close()
                # process_finished = True
                idle_count = 0

                print(f'process finished: {process_finished} and data stored in json \n')
            else:
                idle_count += 1
                # print("idle count:" + str(idle_count))
                return

        # subscriber_Odom_ = self.create_subscription(Odometry, '/odom', get_Message, 1)
        subscriber_Odom_at_interval = self.create_subscription(Odometry, '/odom_at_interval', get_Message, 1)

        def on_finish_end_subscription():
            """ when process_finished subscription to topic /odom_at_interval is ended """
            if process_finished:
                print(f'stopping subscription node on process end : {process_finished}')
                # subscriber_Odom_.destroy()
                Node.destroy_subscription(self, subscriber_Odom_at_interval)
                Node.destroy_timer(self, timer_)

            else:
                # threading.Timer(0.5,on_finish_end_subscription).start()
                pass

        # on_finish_end_subscription()

        timer_period = 0.5  # seconds
        timer_ = self.create_timer(timer_period, on_finish_end_subscription)
        return

    def subscribe_message(self, msg):
        """ gets called from subscriber_goal_waypoints_ via topic /waypoints
        """
        print("new goal selected")
        # self.new_waypoint = True
        new_no_of_waypoint = self.no_of_waypoint + 1
        self.if_goal_changed(new_no_of_waypoint)
        self.get_logger().info(str(msg))

    def if_goal_changed(self, new_no_of_waypoint):
        """ gets called from subscrib_message() and runs populate_Json method if new goal selection is confirmed"""
        value_changed = self.no_of_waypoint != new_no_of_waypoint
        if value_changed:
            self.no_of_waypoint = new_no_of_waypoint
            print("new waypoint: " + str(self.no_of_waypoint))
            self.populate_Json(new_no_of_waypoint)
        else:
            print("value didn't change")


def main(args=None):
    rclpy.init(args=args)
    # initialize and run GoalSubscriber Class instance
    goal_subscriber = GoalSubscriber()
    rclpy.spin(goal_subscriber)
    goal_subscriber.destroy_node()
    ### end GoalSubscriber Class instance

    rclpy.shutdown()


if __name__ == '__main__':
    main()
