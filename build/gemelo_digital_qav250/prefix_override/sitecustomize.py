import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bris/Desktop/reto/gemelo_digital_qav250/install/gemelo_digital_qav250'
