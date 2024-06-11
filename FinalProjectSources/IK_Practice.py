import numpy as np
import cv2
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from pymycobot.mycobot import MyCobot
from ultralytics import YOLO

# 로봇 팔 연결
mc = MyCobot('COM8', 115200)

# 모델
model = YOLO(r"runs\detect\train2\weights\best.pt")

# 로봇 팔 구성
# robot_arm_chain이라는 이름의 로봇 팔 정의. 각 관절은 URDFLink로 정의되며, 각 관절의 위치와 회전 축이 설정됨
robot_arm_chain = Chain(name='mycobot', links=[
    OriginLink(),
    URDFLink(
        name="joint1",
        origin_translation=[0, 0, 0.173],  # joint2_to_joint1의 origin xyz
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1]  # axis xyz
    ),
    URDFLink(
        name="joint2",
        origin_translation=[0, -0.086, 0],  # joint3_to_joint2의 origin xyz
        origin_orientation=[0, -1.5708, 1.5708],
        rotation=[0, 1, 0]  # axis xyz
    ),
    URDFLink(
        name="joint3",
        origin_translation=[0.13635, 0, -0.086],  # joint4_to_joint3의 origin xyz
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0]  # axis xyz
    ),
    URDFLink(
        name="joint4",
        origin_translation=[0.1195, 0, 0.082],  # joint5_to_joint4의 origin xyz
        origin_orientation=[0, 0, 1.57080],
        rotation=[0, 1, 0]  # axis xyz
    ),
    URDFLink(
        name="joint5",
        origin_translation=[0, -0.09415, 0],  # joint6_to_joint5의 origin xyz
        origin_orientation=[1.5708, 0, 0],
        rotation=[0, 0, 1]  # axis xyz
    ),
    URDFLink(
        name="joint6",
        origin_translation=[0, 0.055, 0],  # joint6output_to_joint6의 origin xyz
        origin_orientation=[-1.5708, 0, 0],
        rotation=[0, 1, 0]  # axis xyz
    )
], active_links_mask=[False, False, False, False, False, True, True])

# 카메라 설정
cap = cv2.VideoCapture(0)

# detect_target 함수는 이미지의 중앙 좌표를 계산해 반환
def detect_target(image):
    # height, width, _ = image.shape
    # target_x, target_y = width // 2, height // 2
    # return target_x, target_y
    
    center_x, center_y = None, None
    
    result = model(image, stream=True)
    class_name = "white cane"
    
    for r in result:
        boxes = r.boxes
        for box in boxes: 
            # 객체의 바운딩 박스 좌표 추출
            lx, ly, rx, ry = box.xyxy[0]
            lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)
            
            # 신뢰도로 한번 거름
            conf = round(float(box.conf[0]), 2)
            if conf >= 0.6:
                # 바운딩 박스 그리기
                if class_name == "white cane":
                    cv2.rectangle(frame, (lx, ly), (rx, ry), (255,0,255), 3)
                
                    # 객체의 중심점 계산 및 표시
                    center_x = (lx + rx) // 2
                    center_y = (ly + ry) // 2
                    cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                    print(center_x, center_y)

                    # 신뢰도와 클래스 이름 표시
                    cv2.putText(frame, str(conf), (center_x, center_y+25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(frame, str(class_name), (center_x, center_y-25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return frame, center_x, center_y

def move_robot_arm(target_x, target_y):
    target_vector = [target_x / 100.0, target_y / 100.0, 0.1]  # 단위를 맞추기 위해 100으로 나눔
    target_frame = np.eye(4)
    target_frame[:3, 3] = target_vector

    ik_solution = robot_arm_chain.inverse_kinematics(target_position=target_vector)
    print(f"IK Solution: {ik_solution}")
    
    angles = np.degrees(ik_solution[1:7])
    print(f"Angles to send to MyCobot: {angles}")
    
    mc.send_angles([angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]], 50)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame, target_x, target_y = detect_target(frame)
    
    if target_x is not None and target_y is not None:
        cv2.circle(frame, (target_x, target_y), 10, (0, 0, 255), -1)
        move_robot_arm(target_x, target_y)
    
    cv2.imshow('Frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


