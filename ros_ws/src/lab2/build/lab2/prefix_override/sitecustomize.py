import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/CPE416/ros_ws/src/lab2/install/lab2'
