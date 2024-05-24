#!/usr/bin/env python
import rospy
import termios
from geometry_msgs.msg import Twist
import sys, os
from flask import Flask, request, jsonify
from threading import Thread

# Flask 앱 초기화
app = Flask(__name__)

# 전역 변수 초기화
cobot_command = None

# Flask 서버의 라우트
@app.route('/upload', methods=['POST'])
def upload():
    global cobot_command
    data = request.get_json()
    if not data:
        return jsonify({'error': 'No data provided'}), 400

    # 받은 데이터 출력
    print(f"Received data: {data}")

    # 필요한 경우 데이터 처리
    cobot_command = data.get('order_')
    class_name = data.get('class_name')
    print(f"class_name = {class_name}, order = {cobot_command}")

    response = f"Received data Successfully {cobot_command}"
    return jsonify(response), 200

def flask_thread():
    app.run(host='0.0.0.0', port=5000)

# ROS 노드 및 로직
def main():
    global cobot_command
    rospy.init_node('robot_control_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    try:
        print("Robot control. Waiting for cobot signals...")

        while not rospy.is_shutdown():
            twist = Twist()

            if cobot_command == 'PUSH' or cobot_command == 'FIRST FLOOR' or cobot_command == 'close' or cobot_command == 'white cane':
                pass  # twist가 이미 0으로 설정됨 (멈춤)
            elif cobot_command == 'open' or cobot_command == 'go':
                twist.linear.x = 0.2  # 선속도를 1로 설정 (주행) > 시각장애인 평균 속도에 맞춤
            else:
                if cobot_command == '\x03':
                    break

            pub.publish(twist)
            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Flask 서버를 별도의 스레드에서 실행
    flask_thread = Thread(target=flask_thread)
    flask_thread.start()

    main()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)