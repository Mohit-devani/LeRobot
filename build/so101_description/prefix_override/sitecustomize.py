import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mohit-devani/ros2_ws/src/so101_description/install/so101_description'
