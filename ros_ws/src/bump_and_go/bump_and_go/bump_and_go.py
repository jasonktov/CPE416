import rclpy
from rclpy.node import Node


'''
    We're going to implement an open loop controller/FSM to have the turtlebot
    draw a square on the screen. Below I have commented parts of the code that
    you need to fill in to make the logic complete.
'''

# We have to use the geometry_msgs/msg/Twist to control robots
# Write in here what the correct import should be
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class BumpAndGo(Node):

    def __init__(self):
        super().__init__('bump_and_go')

        #Publisher @ 10Hz
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.state = "Forward"
        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = float('1.1')

        self.turn_msg = TwistStamped()
        self.turn_msg.twist.angular.z = float('0.5')

        self.back_msg = TwistStamped()
        self.back_msg.twist.linear.x = float('-1.1')

        self.stop_msg = TwistStamped()

        #Subscribe to /scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

    # Callback for the events
    def timer_callback(self):
        if(self.state == "Forward"):
            self.get_logger().info("Going Forward")
            self.publisher_.publish(self.forward_msg)
        elif(self.state == "Turn"):
            self.get_logger().info("Turning")
            self.publisher_.publish(self.turn_msg)
        elif(self.state == "Back"):
            self.get_logger().info("Going Backwards")
            self.publisher_.publish(self.back_msg)
        elif(self.state == "Stop"):
            self.publisher_.publish(self.stop_msg)

    def listener_callback(self, msg):
        mid_i = (int)(len(msg.ranges) / 2)
        sweep_i = (int)(len(msg.ranges) / 8)
        bump_threshold = 3
        detect_threshold = 5

        f_bump = False
        f_detect = False
        b_bump = False
        b_detect = False

        for i in range(sweep_i):
            if(msg.ranges[i] < bump_threshold or msg.ranges[-i] < bump_threshold):
                b_bump = True
            if(msg.ranges[mid_i + i] < bump_threshold or msg.ranges[mid_i - i] < bump_threshold):
                f_bump = True
            if (msg.ranges[i] < detect_threshold or msg.ranges[-i] < detect_threshold):
                b_detect = True
            if (msg.ranges[mid_i + i] < detect_threshold or msg.ranges[mid_i - i] < detect_threshold):
                f_detect = True

        if(self.state == "Forward"):
            if(f_bump):
                self.state = "Back"
        elif (self.state == "Back"):
            if (b_bump or not f_bump):
                self.state = "Turn"
        elif(self.state == "Turn"):
            if (not f_detect):
                self.state = "Forward"



def main(args=None):
    rclpy.init(args=args)

    bump_and_go = BumpAndGo()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(bump_and_go)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bump_and_go.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
