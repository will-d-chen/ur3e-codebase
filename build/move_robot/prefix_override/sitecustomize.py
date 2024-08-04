import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bigchungus/ros2_iron_ws/src/move_robot/install/move_robot'
