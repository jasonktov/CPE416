import rclpy
from rclpy.node import Node

from queue import Queue

import math
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Object orientated nodes for ROS in python
class Explorer(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('explorer_node')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.odom_callback,
            10
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose_out',
            10)

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/explorer_map',
            10
        )

        self.cur_map = None
        self.frontier_map = None
        self.cur_odom = None

        self.groups = [[] for _ in range(255)]

        timer_period = 0.50 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def map_callback(self, msg):
        self.cur_map = msg

    def odom_callback(self, msg):
        self.cur_odom = msg

    # Callback for the events
    def timer_callback(self):
        if(self.cur_map is None):
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'diff_drive/base_link'
        self.goal_publisher.publish(goal)

        self.frontier_map = OccupancyGrid()
        self.frontier_map = self.explore(self.cur_map, self.frontier_map)
        self.group_frontiers(self.frontier_map)
        self.map_publisher.publish(self.frontier_map)

    def explore(self, map:OccupancyGrid, frontier_map:OccupancyGrid):
        width = map.info.width
        height = map.info.height
        map_array = map.data

        frontier_map.info = map.info
        frontier_map.header = map.header
        frontier_map.header.stamp = self.get_clock().now().to_msg()
        frontier_map.data = [-1] * width * height

        for i in range(width*height):
            if(map_array[i] == 0):
                if(self.check_neighbors(i,  map)):
                    frontier_map.data[i] = 0 #cell is frontier

        return frontier_map

    def check_neighbors(self, i, map: OccupancyGrid):
        #returns whether or not the cell is a frontier cell
        width = map.info.width
        map_array = map.data

        neighbor_is = [i - width - 1, i - width, i - width + 1,
                       i - 1,                    i + 1,
                       i + width - 1, i + width, i + width + 1]

        for index in neighbor_is:
            if(map_array[index] == -1):
                return True
        return False

    def group_frontiers(self, frontier_map:OccupancyGrid):
        width = frontier_map.info.width
        height = frontier_map.info.height
        frontier_map_array = frontier_map.data

        cur_group = 0

        for i in range(width * height):
            if (frontier_map_array[i] == 0):
                cur_group = cur_group + 1
                if(cur_group < len(self.groups)):
                    self.expand(frontier_map, i, cur_group)

        self.get_logger().info(f"# of groups : {len(self.groups)}")
        return frontier_map

    def expand(self, frontier_map, i, group):
        width = frontier_map.info.width
        frontier_map_array = frontier_map.data

        neighbor_is = [i - width - 1, i - width, i - width + 1,
                       i - 1, i + 1,
                       i + width - 1, i + width, i + width + 1]

        self.get_logger().info(f"i: {i}")
        frontier_map_array[i] = group
        self.groups[group].append(i)
        for index in neighbor_is:
            if(index > 0 and index < len(frontier_map_array)):
                if(frontier_map_array[index] == 0):
                    self.expand(frontier_map, index, group)

def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(explorer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
