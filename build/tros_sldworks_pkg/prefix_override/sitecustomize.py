import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rara/tros_sldworks_pkg/install/tros_sldworks_pkg'
