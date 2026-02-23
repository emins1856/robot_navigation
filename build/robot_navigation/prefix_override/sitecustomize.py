import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emin/ros2_ws/src/robot_navigation/install/robot_navigation'
