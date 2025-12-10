import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import heapq
import math

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose

from action_msgs.msg import GoalStatus


def world_to_cell(pose_grid: Pose, grid: OccupancyGrid):
    x_world = pose_grid.position.x
    y_world = pose_grid.position.y

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

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cur_map = None
        self.frontier_map = None
        self.cur_odom = None

        self.groups = [[] for _ in range(255)]
        self.num_groups = 0

        self.goal_cells = []
        self.cur_goal_cells_i = 0;

        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def map_callback(self, msg):
        self.cur_map = msg

    def odom_callback(self, msg):
        self.cur_odom = msg

    # Callback for the events
    def timer_callback(self):
        if(self.cur_map is None or self.cur_odom is None):
            return
        self.frontier_map = OccupancyGrid()
        self.frontier_map = self.explore(self.cur_map, self.frontier_map)
        self.group_frontiers(self.frontier_map)
        self.map_publisher.publish(self.frontier_map)

        cur_pose = PoseStamped()
        cur_pose.header = self.cur_odom.header
        cur_pose.pose = self.cur_odom.pose.pose

        transform = self.tf_buffer.lookup_transform(
            self.frontier_map.header.frame_id,  # target
            cur_pose.header.frame_id,  # source ("odom")
            rclpy.time.Time(),  # latest available
            timeout = Duration(seconds = 0.2)
        )

        transformed_pose = do_transform_pose(cur_pose.pose, transform)

        bot_i = world_to_cell(transformed_pose, self.frontier_map)
        self.select_advanced(self.frontier_map, self.groups, bot_i)
        self.send_goal(self.goal_cells[self.cur_goal_cells_i][1])

    def send_goal(self, goal_i):
        goal = cell_index_to_world_pose(goal_i, self.frontier_map, self.get_clock().now().to_msg())
        self.goal_publisher.publish(goal)

        self._client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        # Action result future
        result_msg = future.result()
        status = result_msg.status  # numeric GoalStatus code
        result = result_msg.result  # NavigateToPose_Result

        # Convert numeric status → readable string
        status_text = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, "INVALID_STATUS")

        self.get_logger().info(f"Navigation finished with status {status} ({status_text})")
        if status == GoalStatus.STATUS_ABORTED:
            self.cur_goal_cells_i += 1
            self.send_goal(self.goal_cells[self.cur_goal_cells_i])


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

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

                if(shortest_index is None or shortest_dist > dist):
                    shortest_dist = dist
                    shortest_index = index
                #heapq.heappush(self.goal_cells, (dist, index))
        return shortest_index

    def select_advanced(self, frontier_map:OccupancyGrid, groups, bot_i):
        width = frontier_map.info.width
        frontier_map_array = frontier_map.data

        shortest_center_dist = 100000
        closest_edge_index = None

        bot_x = bot_i % width
        bot_y = math.floor(bot_i / width)

        for group in groups:
            #get group size & center node
            group_size = 0
            x_sum = 0
            y_sum = 0
            closest_edge = None
            shortest_edge_dist = 100000

            for cell_i in group:
                group_size += 1
                x_sum += cell_i % width
                y_sum += math.floor(cell_i / width)

                index_x = cell_i % width
                index_y = math.floor(cell_i / width)

                edge_dist = math.sqrt(((index_x - bot_x) ** 2) + ((index_y - bot_y) ** 2))

                if(closest_edge is None or edge_dist < shortest_edge_dist):
                    closest_edge = cell_i
                    shortest_edge_dist = edge_dist

            center_x = x_sum/group_size
            center_y = y_sum.group_size
            center_dist = math.sqrt(((center_x - bot_x) ** 2) + ((center_y - bot_y) ** 2))

            self.cur_goal_cells_i = 0
            heapq.heappush(self.goal_cells, (center_dist, closest_edge_index))
            #if (closest_edge_index is None or center_dist < shortest_center_dist):
                #closest_edge_index = closest_edge
                #shortest_center_dist = center_dist
        return closest_edge_index

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
