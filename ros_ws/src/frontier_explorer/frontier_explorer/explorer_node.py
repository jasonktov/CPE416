import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import heapq
import math
from concurrent.futures import Future
import threading

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

def get_input(msg):
    future = Future()

    def worker():
        user_text = input(msg)
        future.set_result(user_text)

    t = threading.Thread(target=worker, daemon=True)
    t.start()

    return future.result()

class Group():
    def __init__(self, grid : OccupancyGrid, id):
        self.group_id = id
        self.cells = []
        self.center_cell = None

        self.grid = grid
        self.grid_width = grid.info.width
        self.grid_height = grid.info.height
        self.grid_size = grid.info.width * grid.info.height

    def add_cell(self, index):
        self.cells.append(index)
        self.update_center()

    def update_center(self):
        sum_x = 0
        sum_y = 0

        for cell_i in self.cells:
            sum_x += cell_i % self.grid_width
            sum_y += math.floor(cell_i/self.grid_width)

        self.center_cell = (sum_x/len(self.cells)) + (self.grid_width * (sum_y/len(self.cells)))

    def get_closest_edge(self, bot_i):
        shortest_dist = 100000
        closest_i = None

        bot_x = bot_i % self.grid_width
        bot_y = math.floor(bot_i / self.grid_width)

        for cell_i in self.cells:
            cell_x = cell_i % self.grid_width
            cell_y = math.floor(cell_i/self.grid_width)

            delta_x = abs(cell_x - bot_x)
            delta_y = abs(cell_y - bot_y)

            dist = math.sqrt(delta_x**2 + delta_y**2)

            if(closest_i is None or dist < shortest_dist):
                shortest_dist = dist
                closest_i = cell_i

        return closest_i

    def get_center_dist(self, bot_i):
        center_x = self.center_cell % self.grid_width
        center_y = math.floor(self.center_cell / self.grid_width)

        bot_x = bot_i % self.grid_width
        bot_y = math.floor(bot_i / self.grid_width)

        delta_x = abs(center_x - bot_x)
        delta_y = abs(center_y - bot_y)

        return math.sqrt(delta_x ** 2 + delta_y ** 2)

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
        self.cur_odom = None

        self.frontier_cells = []
        self.frontier_map = None

        self.groups = []

        self.goal_cells = []
        self.cur_goal_cells_i = 0

        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def map_callback(self, msg):
        self.cur_map = msg

    def odom_callback(self, msg):
        self.cur_odom = msg

    # Callback for the events
    def timer_callback(self):
        if(self.cur_map is None or self.cur_odom is None):
            return

        self.timer.cancel()
        #create frontier map
        self.frontier_map = OccupancyGrid()
        self.create_frontier(self.cur_map)
        self.get_logger().info(f"{len(self.frontier_cells)} frontier cells")
        self.map_publisher.publish(self.frontier_map)

        self.group_frontiers()
        self.get_logger().info(f"{len(self.groups)} groups")
        self.recolor_frontier_map()
        #self.map_publisher.publish(self.frontier_map)

        #get bot location pose
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

        #create list of goals
        self.create_goals(bot_i)

        self.cur_goal_cells_i = 0
        self.send_next_goal()

    def create_frontier(self, map:OccupancyGrid):
        width = map.info.width
        height = map.info.height
        map_array = map.data

        self.frontier_map.info = map.info
        self.frontier_map.header = map.header
        self.frontier_map.header.stamp = self.get_clock().now().to_msg()
        self.frontier_map.data = [-1] * width * height

        self.frontier_cells = []

        for i in range(width*height):
            if(map_array[i] == 0):
                if(self.check_neighbors(map, i)):
                    self.frontier_map.data[i] = 0 #cell is frontier
                    self.frontier_cells.append(i)

    def check_neighbors(self, map:OccupancyGrid, i):
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

    def group_frontiers(self):
        self.groups = []
        cur_group_id = 0

        for cell in self.frontier_cells:
            if self.frontier_map.data[cell] != 100:
                group = Group(self.frontier_map, cur_group_id)
                self.frontier_map.data[cell] = 100 #mark as grouped
                group.add_cell(cell)
                self.expand(cell, group, 1)

                #only add groups with more than one frontier cell
                if len(group.cells) > 1:
                    self.groups.append(group)
                    cur_group_id += 1

    def expand(self, cell_i, group, depth):
        self.get_logger().info(f"{depth}")
        width = self.frontier_map.info.width

        neighbor_is = [cell_i - width - 1, cell_i - width, cell_i - width + 1,
                       cell_i - 1, cell_i + 1,
                       cell_i + width - 1, cell_i + width, cell_i + width + 1]

        for neighbor in neighbor_is:
            if neighbor in self.frontier_cells and self.frontier_map.data[neighbor] != 100:
                self.frontier_map.data[neighbor] = 100
                group.add_cell(neighbor)
                self.expand(neighbor, group, depth + 1)

    def recolor_frontier_map(self):
        for group in self.groups:
            for cell_i in group.cells:
                self.frontier_map.data[cell_i] = group.group_id

    def create_goals(self, bot_i):
        self.goal_cells = []

        for group in self.groups:
            center_dist = group.get_center_dist(bot_i)
            closest_edge_i = group.get_closest_edge(bot_i)
            heapq.heappush(self.goal_cells, (center_dist, closest_edge_i))

    def send_next_goal(self):
        goal_cell = self.goal_cells[self.cur_goal_cells_i][1]
        goal = cell_index_to_world_pose(goal_cell, self.frontier_map, self.get_clock().now().to_msg())
        self.goal_publisher.publish(goal)

        self.get_logger().info(f"Send goal #{self.cur_goal_cells_i}? | goal cell:{goal_cell} | goal pose:{goal}")
        self.cur_goal_cells_i += 1
        user_input = get_input("y:send goal n:next goal r:abort & redraw groups")
        if(user_input == "y"):
            self.get_logger().info(f"Goal Sent")
            self._client.wait_for_server()
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal

            send_goal_future = self._client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
        elif(user_input == "n"):
            #get next goal cell
            self.send_next_goal()
        elif(user_input == "r"):
            self.get_logger().info(f"Aborted")
            self.timer.reset()
        else:
            self.get_logger().info(f"Invalid Input, Aborted")
            self.timer.reset()


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.timer.reset()
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
            self.send_next_goal()
        else:
            self.timer.reset()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Moving...')


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
