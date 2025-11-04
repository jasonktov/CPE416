import rclpy
from rclpy.node import Node

# Needed imports
from math import cos, sin, isfinite
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion


class LocalCostmap(Node):
    def __init__(self):
        super().__init__('local_costmap')

        # Map config
        self.map_width = 300  # cells
        self.map_height = 300  # cells
        self.map_resolution = 0.05  # resolution * cells  => 15 m x 15 m map

        # Occupancy conventions
        self.UNKNOWN = -1
        self.FREE = 0
        self.OCCUPIED = 100

        # Outgoing OccupancyGrid message (we'll fill .info and .header)
        self.publish_map = OccupancyGrid()
        self.publish_map.data = [-1] * (self.map_width * self.map_height)
        self._init_map_info()

        # Precompute robot's cell (center of grid)
        self.cx = self.map_width // 2
        self.cy = self.map_height // 2

        self.laser_scan = None
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)

        self.publisher_ = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    ''' Initialize static OccupancyGrid.info and origin so the robot is at the map center. '''

    def _init_map_info(self):
        self.publish_map.info.resolution = self.map_resolution
        self.publish_map.info.width = self.map_width
        self.publish_map.info.height = self.map_height

        # Place (0,0) of the grid so that the robot (base frame origin) is at the center cell.
        # That means the map origin (bottom-left corner in world coords) is shifted negative by half-size.
        origin = Pose()
        origin.position.x = - (self.map_width * self.map_resolution) / 2.0
        origin.position.y = - (self.map_height * self.map_resolution) / 2.0
        origin.position.z = 0.0
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.publish_map.info.origin = origin

    ''' Meters in robot frame -> map indices (mx, my). Returns None if out of bounds. '''

    # Input: (x, y) coordinates of a point in the Cartesian plane
    # Output: Corresponding cell in the occupancy grid
    def world_to_map(self, x_m, y_m):
        if((x_m > self.map_width * self.map_resolution) or (y_m > self.map_height * self.map_resolution)):
            return None
        mx = x_m / self.map_resolution
        my = y_m / self.map_resolution
        mx += self.cx
        my += self.cy
        #self.get_logger().info(f"{x_m}, {y_m}, {self.map_resolution}, {mx}, {my}")
        return round(mx), round(my)

    # Bresenham's Line Algorithm: inclusive endpoints
    # Input: 2-points on the Cartesian plane (i.e. a line)
    # (The first point is the robot origin, while the sencond is a single beam's endpoint)
    # Output: All the cells that that the beam crosses. i.e. the free cells.
    def bresenham_line_algorithm(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        if(x0 < x1):
            sx = 1
        else:
            sx = -1
        if(y0 < y1):
            sy = 1
        else:
            sy = -1

        error = dx + dy
        free_space_cells = []
        while(True):
            free_space_cells.append((x0, y0))
            e2 = 2*error
            if(e2 >= dy):
                if(x0 == x1):
                    break
                else:
                    error += dy
                    x0 += sx
            if(e2 <= dx):
                if(y0 == y1):
                    break
                else:
                    error += dx
                    y0 += sy

        return free_space_cells

    ''' Cache the most recent LaserScan'''

    def laser_callback(self, msg: LaserScan):
        self.laser_scan = msg
        return

    def timer_callback(self):
        self.build_occupancy_grid()
        return

    # Input: x & y coordinates;
    # Output: list of free cells along the ray (excludes the last cell)
    def raytrace(self, x_cell, y_cell):
        # Compute free cells for a single beam
        # This function should call self.bresenham_line_algorithm
        free_cells = self.bresenham_line_algorithm(self.cx, self.cy, x_cell, y_cell)
        return free_cells

    ''' Build and Publish the Occupancy Grid from the most recent LiDAR Scan '''

    def build_occupancy_grid(self):
        # First, check that the scan data is ready
        if(self.laser_scan is None):
            return
        # Second, iterate through beams to create the map!
        size = len(self.laser_scan.ranges)
        a = self.laser_scan.angle_increment
        min = self.laser_scan.angle_min

        for index in range(size):
            dist = self.laser_scan.ranges[index]
            if dist > 0 and dist < 15:
                angle = min + (a * index)
                x_m = cos(angle) * dist
                y_m = sin(angle) * dist
                mx, my = self.world_to_map(x_m, y_m)
                
                for cell in self.raytrace(mx, my):
                    if(cell[0] + (cell[1]*self.map_width) < len(self.publish_map.data)):
                        self.publish_map.data[cell[0] + (cell[1] * self.map_width)] = 0
                if(mx+(my*self.map_width) < len(self.publish_map.data)):
                    self.publish_map.data[mx+(my*self.map_width)] = 100

        # Populate OccupancyGrid message
        self.publish_map.header.stamp = self.get_clock().now().to_msg()
        # Set frame to match your robot frame that LaserScan is in (commonly "base_link" or "laser")
        # The simulation and hardware will have different names for this frame
        self.publish_map.header.frame_id = 'rplidar_link'
        # Publish
        self.publisher_.publish(self.publish_map)


def main(args=None):
    rclpy.init(args=args)

    # Node creation and spin
    local_costmap = LocalCostmap()
    rclpy.spin(local_costmap)

    # Node cleanup
    local_costmap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
