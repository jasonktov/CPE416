import rclpy
from rclpy.node import Node

from queue import Queue

import math
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

def world_to_cell(pose_grid: PoseStamped, grid: OccupancyGrid):
    x_world = pose_grid.pose.position.x
    y_world = pose_grid.pose.position.y

    origin = grid.info.origin
    x0 = origin.position.x
    y0 = origin.position.y
    res = grid.info.resolution

    # Convert quaternion -> yaw
    q = origin.orientation
    # yaw from quaternion (z, w) assuming 2D
    siny_cosp = 2.0 * (q.w * q.z)
    cosy_cosp = 1.0 - 2.0 * (q.z * q.z)
    theta0 = math.atan2(siny_cosp, cosy_cosp)

    # Translate to origin
    dx = x_world - x0
    dy = y_world - y0

    # Rotate into grid frame (undo origin yaw)
    xg =  math.cos(theta0) * dx + math.sin(theta0) * dy
    yg = -math.sin(theta0) * dx + math.cos(theta0) * dy

    col = int(math.floor(xg / res))
    row = int(math.floor(yg / res))

    if col < 0 or col >= grid.info.width or row < 0 or row >= grid.info.height:
        return None  # outside map

    index = row * grid.info.width + col
    return index

def cell_index_to_world_pose(index: int, grid: OccupancyGrid, stamp) -> PoseStamped:
    """
    Convert a linear cell index in an OccupancyGrid to a PoseStamped in the grid frame.
    The pose is located at the center of the cell.
    """

    width  = grid.info.width
    height = grid.info.height
    res    = grid.info.resolution

    if index < 0 or index >= width * height:
        raise IndexError(f"Index {index} out of bounds for grid of size {width}x{height}")

    # Row-major layout: index = row * width + col
    row = index // width
    col = index % width

    # --- grid-aligned coordinates (before origin translation/rotation) ---
    # cell center in grid-local coordinates
    xg = (col + 0.5) * res
    yg = (row + 0.5) * res

    # --- origin pose (translation + yaw) ---
    origin = grid.info.origin
    x0 = origin.position.x
    y0 = origin.position.y

    q = origin.orientation
    # yaw from quaternion, assuming 2D (x-y plane, z-up)
    siny_cosp = 2.0 * (q.w * q.z)
    cosy_cosp = 1.0 - 2.0 * (q.z * q.z)
    theta0 = math.atan2(siny_cosp, cosy_cosp)

    c = math.cos(theta0)
    s = math.sin(theta0)

    # Rotate + translate into world/grid frame coordinates
    x_world = x0 + c * xg - s * yg
    y_world = y0 + s * xg + c * yg

    # --- build PoseStamped ---
    pose = PoseStamped()
    pose.header.frame_id = grid.header.frame_id
    pose.header.stamp = stamp if stamp is not None else grid.header.stamp

    pose.pose.position.x = x_world
    pose.pose.position.y = y_world
    pose.pose.position.z = 0.0

    # Orientation: identity (facing along +x). You can set something else if needed.
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    return pose

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cur_map = None
        self.frontier_map = None
        self.cur_odom = None

        self.groups = [[] for _ in range(255)]
        self.num_groups = 0

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
        self.frontier_map = OccupancyGrid()
        self.frontier_map = self.explore(self.cur_map, self.frontier_map)
        self.group_frontiers(self.frontier_map)
        self.map_publisher.publish(self.frontier_map)

        transformed_pose = self.tf_buffer.transform(
                self.cur_odom,
                self.frontier_map.header.frame_id,
                rclpy.time.Duration(seconds=0.2)
            )
        bot_i = world_to_cell(transformed_pose, self.frontier_map)
        goal_i = self.select_basic(self.frontier_map, bot_i)

        goal = cell_index_to_world_pose(goal_i, self.frontier_map, self.get_clock().now().to_msg())
        self.goal_publisher.publish(goal)
        

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
        height = map.info.height
        map_array = map.data

        neighbor_is = [i - width - 1, i - width, i - width + 1,
                       i - 1,                    i + 1,
                       i + width - 1, i + width, i + width + 1]

        for index in neighbor_is:
            if(index > 0 and index < width * height):
                if(map_array[index] == -1): 
                    return True
        return False

    def group_frontiers(self, frontier_map:OccupancyGrid):
        width = frontier_map.info.width
        height = frontier_map.info.height
        frontier_map_array = frontier_map.data

        self.groups = [[] for _ in range(255)]
        self.num_groups = 0

        for i in range(width * height):
            if (frontier_map_array[i] == 0):
                self.num_groups = self.num_groups + 1
                if(self.num_groups < len(self.groups)):
                    self.expand(frontier_map, i, self.num_groups, 0)

        self.get_logger().info(f"# of groups : {self.num_groups}")
        return frontier_map

    def expand(self, frontier_map, i, group, depth):
        width= frontier_map.info.width
        frontier_map_array = frontier_map.data

        neighbor_is = [i - width - 1, i - width, i - width + 1,
                       i - 1, i + 1,
                       i + width - 1, i + width, i + width + 1]

        #self.get_logger().info(f"depth: {depth}")
        frontier_map_array[i] = (group % 127) + 1
        self.groups[group].insert(group, i)
        if(depth > 900):
            return
        for index in neighbor_is:
            if(index > 0 and index < len(frontier_map_array)):
                if(frontier_map_array[index] == 0):
                    self.expand(frontier_map, index, group, depth + 1)

    def select_basic(self, frontier_map:OccupancyGrid, bot_i):
        width = frontier_map.info.width
        frontier_map_array = frontier_map.data

        shortest_dist = 100000
        shortest_index = None

        for index in range(len(frontier_map_array)):
            if(frontier_map_array[index] != -1):
                index_x = index % width
                index_y = math.floor(index / width)

                bot_x = bot_i % width
                bot_y = math.floor(bot_i / width)

                dist = math.sqrt(((index_x - bot_x) ** 2) + ((index_y - bot_y) ** 2))

                if(shortest_index is None):
                    shortest_index = index
                    shortest_dist = dist
                elif(dist < shortest_dist):
                    shortest_index = index
                    shortest_dist = dist
        return shortest_index

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
