import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
import time

robot = Marvin_Robot()

# 连接
if robot.connect('192.168.1.190') == 0:
    print('failed to connect')
    exit(0)
time.sleep(0.5)

# # SDK 库版本
# sdk_ver = robot.SDK_version()
# print(f'SDK version: {sdk_ver}')

# 控制器固件版本
ret, version = robot.get_param('int', 'VERSION')
print(f'Controller version: {version} (ret={ret})')

robot.release_robot()
