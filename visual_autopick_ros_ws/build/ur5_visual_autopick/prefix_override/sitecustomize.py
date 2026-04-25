import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/laboratorio/TFM/visual_autopick_ros_ws/install/ur5_visual_autopick'
