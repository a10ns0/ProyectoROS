import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sick/Documents/GitHub/ProyectoROS/install/truck_positioning'
