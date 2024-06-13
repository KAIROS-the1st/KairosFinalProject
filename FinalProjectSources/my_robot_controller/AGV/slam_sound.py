#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from std_msgs.msg import String

operation_flag = False
operation_mode = "stop"

def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)

def move_to_goal(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw
    
    client.send_goal(goal)
    
    while not rospy.is_shutdown():
        if operation_mode == "stop":
            client.cancel_goal()
            rospy.sleep(1) #상태 체크 주기
            continue
        elif client.wait_for_result(rospy.Duration(1.0)):
            return client.get_result()
    return None

# 콜백 함수: 신호에 따라 동작 상태를 변경
def signal_callback(msg):
    global operation_mode
    if msg.data == "stop":
        operation_mode = "stop"
    elif msg.data == "go":
        operation_mode = "go"

if __name__ == '__main__':
    rospy.init_node('slam_sound')
    
    tts_publisher = rospy.Publisher('tts_topic', String, queue_size=10)
    cobot_publisher = rospy.Publisher('agv_cobot_communication', String, queue_size=10)
    # 신호 구독
    if not operation_flag:
        rospy.Subscriber('agv_cobot_communication', String, signal_callback)
        operation_flag = True
    
    goals = [
        (4.65, -2.78, 6.01),  # A (603 문 앞)
        (7.33, -2.37, 20.45), # B (엘베 앞) (도달 전에 멈춤)
        (8.91, -2.08, -179.34), # C (엘베 안)
        (5.50, -2.64, -177.71), # D (603 문 앞)
        (3.99, 1.63, -87.66),   # E (외부)
        (4.65, -2.78, 6.01),  # A (603 문 앞)
        (7.33, -2.37, 20.45), # B (엘베 앞) (도달 전에 멈춤)
        (8.91, -2.08, -179.34), # C (엘베 안)
        (5.50, -2.64, -177.71), # D (603 문 앞)
        (1.25, 1.37, 109.00),   # F (승강장 앞 front)
        (0.10, 2.07, -86.02),   # G (승강장 앞 back)
        (0.20, -2.76, 0.26)   # start point
    ]

    messages = [
        "안내를 시작합니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요", # A까지 가는 동안
        "엘리베이터를 인식중입니다, 잠시만 기다려주세요", # B에서
        "엘리베이터에 탑승하겠습니다, 좌측 버튼을 눌러주세요", # C에서
        "내리겠습니다", # D까지 가는 동안
        "안녕하세요, 선릉역 도움 로봇 길동이입니다, 지팡이를 수직으로 올려주세요, 이용자를 인식중입니다, 잠시만 기다려주세요, 인식이 완료되었습니다, 지팡이 잡은 손을 앞으로 뻗어주세요, 지팡이를 잡겠습니다", # E에서
        "내리겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요", # A까지 가는 동안
        "엘리베이터를 인식중입니다, 잠시만 기다려주세요", # B에서
        "엘리베이터에 탑승하겠습니다, 좌측 버튼을 눌러주세요", # C에서
        "내리겠습니다", # D까지 가는 동안
        "최종 목적지인 강남역에 도움 로봇 길동이가 마중나와있을 예정입니다, 지팡이를 풀어드리겠습니다, 좌측에 곧 열차의 문이 열립니다, 지금 탑승하시면 됩니다, 안녕히 가세요", # F에서
        "지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요, 지나가겠습니다, 비켜주세요", # G를 출발하면서
        "안내가 종료 되었습니다" # start point에 도착했을 때
    ]

    for index, goal in enumerate(goals):
        x, y, yaw = goal
        if index == 0:  # A로 이동하기 전에
            tts_publisher.publish(messages[index])
        elif index == 1: # B 위치로 이동하기 전에
            rospy.loginfo("10초 동안 멈춤")
            tts_publisher.publish(messages[index])
            rospy.sleep(10) # 10초 동안 멈춘다
        elif index == 2: # C 위치에서
            tts_publisher.publish(messages[index])
            rospy.sleep(10) # 10초 동안 멈춘다
        elif index == 3: # D로 이동하기 전에
            tts_publisher.publish(messages[index])
        elif index == 4:  # A로 이동하기 전에
            tts_publisher.publish(messages[index])
        elif index == 5: # B 위치로 이동하기 전에
            rospy.loginfo("10초 동안 멈춤")
            tts_publisher.publish(messages[index])
            rospy.sleep(10) # 10초 동안 멈춘다
        elif index == 6: # C 위치에서
            tts_publisher.publish(messages[index])
            rospy.sleep(10) # 10초 동안 멈춘다
        elif index == 7: # D로 이동하기 전에
            tts_publisher.publish(messages[index])
        elif index == 8: # E 위치에서
            tts_publisher.publish(messages[index])
            rospy.sleep(10) # 10초 동안 멈춘다
        elif index == 9: # F 위치에서
            tts_publisher.publish(messages[index])
            cobot_publisher.publish("stage 9")
            rospy.sleep(10) # 10초 동안 멈춘다
        elif index == 10: # G를 출발하면서
            tts_publisher.publish(messages[index])
        elif index == 11: # start point에 도착했을 때
            result = move_to_goal(x, y, yaw)
            if result:
                rospy.loginfo("목표 실행 완료!")
                tts_publisher.publish(messages[index])
                break

        result = move_to_goal(x, y, yaw)
        if result:
            rospy.loginfo("목표 실행 완료!")

    rospy.spin()


