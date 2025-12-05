import rclpy
from rclpy.node import Node

from queue import Queue

import math
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


def euler_from_quaternion(orientation):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z  # in radians

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return [qx, qy, qz, qw]

# Object orientated nodes for ROS in python
class Explorer(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('explorer_node')
        self.odom_subscription = self.create_subscription(
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
        self.groups = [[0]] * 100

        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def map_callback(self, msg):
        self.cur_map = msg

    # Callback for the events
    def timer_callback(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'diff_drive/base_link'
        self.goal_publisher.publish(goal)

        self.frontier_map = OccupancyGrid()
        self.frontier_map = self.explore(self.cur_map, self.frontier_map)
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
                    frontier_map.data[i] = 100 #cell is frontier

        return frontier_map

    def check_neighors(self, i, map: OccupancyGrid):
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
