import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jachin/parc_main/PARC2026-ENG-ARCX6/install/test_publisher'
