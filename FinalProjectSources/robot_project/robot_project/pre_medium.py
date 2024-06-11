# from flask import Flask, request
# import numpy as np

# app = Flask(__name__)

# @app.route('/upload', methods=['POST'])
# def upload_file():
#     # JSON 데이터 받기
#     data = request.get_json()
#     width = data['width']
#     center_x = data['center_x']

#     # 받은 데이터 출력
#     print("Width:", width)
#     print("Center X Coordinate:", center_x)

#     return 'I received successfully'

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5000)



# # 로봇암 영점 조절
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyACM0", 115200)
mc.send_angles([0,0,0,0,0,0],70)
time.sleep(3)



# # 그리퍼 캘리
# from pymycobot.mycobot import MyCobot
# import time

# mc = MyCobot('/dev/ttyACM0',115200)
# mc.set_gripper_calibration()
# mc.set_gripper_mode(0)
# mc.init_eletric_gripper()
# time.sleep(1)
# print('GRIPPER MODE = ' + str(mc.get_gripper_mode()))
# while True:
#   print('OPEN')
#   mc.set_eletric_gripper(1)
#   mc.set_gripper_value(100,20)
#   time.sleep(2)
#   print('CLOSE')
#   mc.set_eletric_gripper(0)
#   mc.set_gripper_value(0,20)
#   time.sleep(2)
