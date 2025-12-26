import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sick/Documents/Proyecto/ProyectoROS/install/truck_positioning'
