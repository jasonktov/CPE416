import rclpy
from rclpy.node import Node

from queue import Queue

import math
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance


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
class Filter(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('ekf')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/kiss/odometry',
            self.odom_callback,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            '/oakd/imu/data',
            self.imu_callback,
            10)

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ekf_out',
            10)

        self.odom_queue = Queue(10)
        self.imu_queue = Queue(10)

        self.odom_cur = None
        self.imu_cur = None

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.state = np.array([[0],
                               [0],
                               [0]])
        self.cov = np.array([[0.0001, 0.0001, 0.0001],
                             [0.0001, 0.0001, 0.0001],
                             [0.0001, 0.0001, 0.0001]])

    def imu_callback(self, msg):
        if(self.imu_cur is None):
            self.imu_cur = msg
        else:
            self.imu_queue.put(msg)

    def odom_callback(self, msg):
        if(self.odom_cur is None):
            self.odom_cur = msg
        else:
            self.odom_queue.put(msg)

    # Callback for the events
    def timer_callback(self):
        # Initialize the message to be sent
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'diff_drive/base_link'

        #run ekf
        if(self.odom_cur is None or self.imu_cur is None):
            return
        while(not self.odom_queue.empty() and not self.imu_queue.empty()):
            #both queues not empty
            self.odom_cur = self.odom_queue.get()
            self.imu_cur = self.imu_queue.get()
            self.predict(self.imu_cur, 0.05)
            self.update(self.odom_cur.pose)
        while(not self.odom_queue.empty()):
            #imu_queue empty
            self.odom_cur = self.odom_queue.get()
            self.predict(self.imu_cur, 0.05)
            self.update(self.odom_cur.pose)
        while (not self.odom_queue.empty()):
            # odom_queue empty
            self.imu_cur = self.imu_queue.get()
            self.predict(self.imu_cur, 0.05)
            self.update(self.odom_cur.pose)

        #publish message
        msg.pose = PoseWithCovariance()

        msg.pose.pose = Pose()
        msg.pose.pose.position.x = self.mu[0][0]
        msg.pose.pose.position.y = self.mu[1][0]
        quat = euler_to_quaternion(0, 0, self.mu[2][0])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.x = quat[2]
        msg.pose.pose.orientation.x = quat[3]

        msg.pose.covariance = np.array([[0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0]])
        idx = [0, 1, 5] #x, y, theta positions in covariance matrix
        msg.pose.covariance[np.ix_(idx, idx)] = self.cov

        self.publisher_.publish(msg)

    def predict(self, imu:Imu, dt):
        #predict state
        w = -1 * imu.angular_velocity.x
        theta_pred = self.state[2][0] * w * dt
        if(theta_pred > math.pi):
            theta_pred = theta_pred - (2 * math.pi)
        if(theta_pred < -1 * math.pi):
            theta_pred = theta_pred + (2 * math.pi)
        self.state[2][0] = theta_pred

        #predict covariance
        theta_cov = imu.angular_velocity_covariance[3][3]
        Q = np.array([[0.0001, 0.0001, 0.0001],
                      [0.0001, 0.0001, 0.0001],
                      [0.0001, 0.0001, theta_cov]])
        self.cov = self.cov + Q


    def update(self, odom:PoseWithCovariance):
        theta_p = euler_from_quaternion(odom.orientation)
        Z = np.array([[odom.position.x],
                      [odom.position.y],
                      [theta_p]])
        idx = [0, 1, 5]
        R = np.array(odom.covariance).reshape(6, 6)[np.ix_(idx, idx)]

        K = self.cov @ np.linalg.inv(self.cov + R)
        self.state = self.state + K @ (Z - self.state)
        self.cov = (np.eye(3) - K) @ self.cov

def main(args=None):
    rclpy.init(args=args)

    filter = Filter()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
