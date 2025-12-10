import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped

# Object orientated nodes for ROS in python
class Echoer(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('echo_node')
        # Create a publisher object
        # Arguments include the type of message, the name of the topic,
        # and the queue size
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.publisher_.publish(out)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Echoer()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
