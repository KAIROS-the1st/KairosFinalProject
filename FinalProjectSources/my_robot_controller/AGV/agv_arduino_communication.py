#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket
from flask import Flask, request, jsonify
import threading

app = Flask(__name__)

# ESP32 IP 주소와 포트 설정 (ESP32 시리얼 모니터에 표시된 IP 주소를 사용)
esp32_ip = '172.30.1.49'
esp32_port = 80

# slam_sound 노드로부터 메세지 받음
def ros_subscriber():
    rospy.init_node('agv_arduino_communication')
    rospy.Subscriber('agv_arduino_communication', String, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo("SLAM으로부터 받은 메시지: %s", data.data)
    # 받은 메시지를 rc car 에게 전송
    if data.data == 'index=9':
        send_message_to_esp32('Your Turn')

# ROS 퍼블리셔 설정
pub = rospy.Publisher('esp32_message', String, queue_size=10)

def send_message_to_esp32(message):
    rospy.loginfo(f"Publishing message to ROS: {message}")
    pub.publish(message)

    # 소켓 생성
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((esp32_ip, esp32_port))

        # 메시지 전송
        msg = message + "\r\n"
        client_socket.sendall(msg.encode())

        # 서버로부터 응답 수신
        response = client_socket.recv(1024)
        print("Received from ESP32:", response.decode())
    except socket.error as e:
        print(f"Socket error: {e}")
    finally:
        client_socket.close()

@app.route('/other', methods=['POST'])
def receivemessage():
    data = request.json
    message = data.get('message')
    response = {
        'messagereceived': message
    }

    if message == "Your Trun":
        # 메시지를 ROS 토픽으로 발행
        rospy.loginfo(f"Publishing message to ROS: {message}")
        pub.publish(message)

        send_message_to_esp32("Your Turn")

    return jsonify(response)

if __name__ == '__main__':
    
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=True)).start()
    
    ros_subscriber()
