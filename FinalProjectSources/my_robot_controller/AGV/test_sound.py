#!/usr/bin/env python3

import rospy
from gtts import gTTS
from playsound import playsound
import os
from std_msgs.msg import String
import hashlib
import threading

def play_sound(filename):
    playsound(filename)
    if os.path.exists(filename):
        os.remove(filename)

def callback(data):
    rospy.loginfo(f"Received message: {data.data}")
    
    # 메시지의 해시값을 이용하여 고유한 파일명 생성
    hash_object = hashlib.md5(data.data.encode())
    filename = "/tmp/speech_" + hash_object.hexdigest() + ".mp3"
    
    try:
        tts = gTTS(text=data.data, lang='ko')
        tts.save(filename)
        
        # 비동기적으로 음성 재생
        threading.Thread(target=play_sound, args=(filename,)).start()
    except Exception as e:
        rospy.logerr(f"Error in processing text to speech: {e}")

def tts_node():
    rospy.init_node('tts_node')
    rospy.Subscriber("tts_topic", String, callback)
    
    rospy.spin()

if __name__ == '__main__':
    tts_node()


    
# # #!/usr/bin/env python3

# import rospy
# from gtts import gTTS
# from playsound import playsound
# import os
# # String 메시지 타입을 임포트합니다.
# from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo(f"Received message: {data.data}")
#     tts = gTTS(text=data.data, lang='ko')
#     tts.save("/tmp/speech.mp3")
#     playsound("/tmp/speech.mp3")

# def tts_node():
#     rospy.init_node('tts_node')
#     rospy.Subscriber("tts_topic", String, callback)
    
#     # 메시지 발행자 생성
#     pub = rospy.Publisher('tts_topic', String, queue_size=10)
    
#     # 발행할 메시지
#     message = "출발합니다, 비켜주세요, 지나가겠습니다, 엘리베이터를 인식중입니다, 엘리베이터 입니다, 버튼을 눌러주세요"
#                 # 비켜주세요, 지나가겠습니다, 안녕하세요, agv입니다, 사용자를 인식 중입니다, 잠시만 기다려주세요, 인식이 완료되었습니다,
#                 # 출발합니다, 비켜주세요, 지나가겠습니다,엘리베이터를 인식중입니다, 엘리베이터 안입니다, 버튼을 눌러주세요,
#                 # 비켜주세요, 지나가겠습니다, 승강장에 도착했습니다, 잠시만 기다려주세요, 곧 열차가 도착합니다, 안녕히 가세요,
#                 # 비켜주세요, 지나가겠습니다, 안내가 종료 되었습니다."
    
#     # ROS가 시작될 때까지 잠시 대기
#     rospy.sleep(1)
    
#     # 메시지 발행
#     rospy.loginfo(f"Publishing message: {message}")
#     pub.publish(String(data=message))
    
#     rospy.spin()

# if __name__ == '__main__':
#     tts_node()