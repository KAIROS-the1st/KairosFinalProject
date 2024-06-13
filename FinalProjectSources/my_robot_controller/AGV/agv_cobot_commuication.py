#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from flask import Flask, request, jsonify
import requests
import threading

app = Flask(__name__)

# slam_sound 노드로부터 메세지 받음
def ros_subscriber():
    rospy.Subscriber('agv_cobot_communication', String, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo("SLAM으로부터 받은 메시지: %s", data.data)
    # 받은 메시지를 cobot에게 전송
    send_messages_to_cobot()

def send_messages_to_cobot():
    robotarm_url = 'http://172.20.179.107:5000/release'
    message = 'release'

    response = requests.post(robotarm_url, json={'message': message})
    
    if response.status_code == 200:
        print('메시지를 성공적으로 보냈습니다.')
    else:
        print('메시지 전송에 실패했습니다.')

# ROS 노드 초기화
def init_ros_node():
    rospy.init_node('agv_cobot_communication', anonymous=True)
    # ROS Subscriber를 별도의 스레드에서 실행
    subscriber_thread = threading.Thread(target=ros_subscriber)
    subscriber_thread.start()

# /agv_com 으로 들어오는 메세지 받음: open, close from Master / release OK from ARM
@app.route('/agv_com', methods=['POST'])
def run_script():
    data = request.get_json()
    if not data or 'order_' not in data:
        return jsonify({"status": "failed", "reason": "Invalid message"}), 400

    message = data['order_']
    rospy.loginfo(f"Received message: {message}")
    
    if message == "open":
        rospy.loginfo("Received open message")
        pub.publish("go")
    elif message == "release OK":
        rospy.loginfo("Received ok message")
        pub.publish("go")
    elif message == "close":
        rospy.loginfo("Received close message")
        pub.publish("stop")
    else:
        return jsonify({"status": "failed", "reason": "Invalid message"}), 400

    return jsonify({"status": "success"})

if __name__ == '__main__':
    # slam_sound 노드로 메세지 보냄
    pub = rospy.Publisher('agv_cobot_communication', String, queue_size=10)

    # ROS 노드 초기화
    init_ros_node()

    app.run(host='0.0.0.0', port=5000)