import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def listener_callback(msg):
    node.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    node = Node('my_subscriber_node')
    subscription = node.create_subscription(
        String,
        'my_topic',
        listener_callback,
        10
    )

    # Process callbacks
    while rclpy.ok():
        rclpy.spin_some(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
