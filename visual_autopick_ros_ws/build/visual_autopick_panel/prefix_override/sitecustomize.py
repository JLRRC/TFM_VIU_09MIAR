import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/laboratorio/TFM/visual_autopick_ros_ws/install/visual_autopick_panel'
