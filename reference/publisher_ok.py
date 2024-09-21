import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    node = Node('my_publisher_node')
    publisher = node.create_publisher(String, 'my_topic', 10)

    timer_period = 1.0  # seconds
    count = 0
    timer_callback_time = 0.0

    while rclpy.ok():
        # Check if it's time to publish a message
        if timer_callback_time >= timer_period:
            msg = String()
            msg.data = f'Hello, ROS 2! Count: {count}'

            publisher.publish(msg)

            node.get_logger().info(f'Publishing: "{msg.data}"')
            count += 1
            timer_callback_time = 0.0  # Reset timer

        # Call spin_some to handle callbacks
        rclpy.spin_some(node)

        # Increment the elapsed time
        # Convert nanoseconds to seconds
        timer_callback_time += node.get_clock().now().nanoseconds * 1e-9

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
