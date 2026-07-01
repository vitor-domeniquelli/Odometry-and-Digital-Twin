import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vitor/artigo_ieee_ws/install/robot_digital_twin'
