import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/calpoly/Desktop/CPE416/ros_ws/src/obstacle_layer/install/obstacle_layer'
