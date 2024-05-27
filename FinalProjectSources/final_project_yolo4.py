# for test without app


import cv2
from ultralytics import YOLO
import requests
import time

# 모델 클래스 이름
class_names = ["close", "open", "white cane"]
model = YOLO(r"runs\detect\train2\weights\best.pt")

# 카메라 설정
cap = cv2.VideoCapture(0)

# 서버 URL
robotarm_url = 'http://172.20.179.107:5000/upload'
agv_url = 'http://172.30.1.89:5000/upload'

# 메시지 전송 주기 설정 (초)
send_interval = 0.5
last_send_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    center_x = 0  # center_x 변수 초기화
    order_ = ""
    className = ""
    
    largest_box = None
    largest_area = 0

    result = model(frame, stream=True)
    
    for r in result:
        boxes = r.boxes
        for box in boxes:
            cls = box.cls[0]
            class_name = class_names[int(cls)]  # 클래스 번호에 해당하는 클래스 이름 가져오기
            
            # 객체의 바운딩 박스 좌표 추출
            lx, ly, rx, ry = box.xyxy[0]
            lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)
            
            # 신뢰도로 필터링
            conf = round(float(box.conf[0]), 2)
            if conf >= 0.6:
                # 객체의 면적 계산
                area = (rx - lx) * (ry - ly)
                
                # 가장 큰 객체 선택
                if area > largest_area:
                    largest_area = area
                    largest_box = box

    if largest_box:
        cls = largest_box.cls[0]
        class_name = class_names[int(cls)]

        # 객체의 바운딩 박스 좌표 추출
        lx, ly, rx, ry = largest_box.xyxy[0]
        lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)

        # 바운딩 박스 그리기
        if class_name == "white cane":
            cv2.rectangle(frame, (lx, ly), (rx, ry), (0, 255, 255), 3)
            order_ = "go"; className = "go"
        elif class_name == "open":
            cv2.rectangle(frame, (lx, ly), (rx, ry), (255, 255, 0), 3)
            order_ = "go"; className = "open"
        elif class_name == "close":
            cv2.rectangle(frame, (lx, ly), (rx, ry), (255, 0, 255), 3)
            order_ = "stop"; className = "close"

        # 객체의 중심점 계산 및 표시
        center_x = (lx + rx) // 2
        center_y = (ly + ry) // 2
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

        # 신뢰도와 클래스 이름 표시
        conf = round(float(largest_box.conf[0]), 2)
        cv2.putText(frame, str(conf), (center_x, center_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, str(class_name), (center_x, center_y - 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    current_time = time.time()
    if current_time - last_send_time >= send_interval:
        # 데이터를 JSON 형식으로 서버에 전송 -- Robotarm
        payload = {
            'width': width,
            'center_x': center_x,
        }
        # 데이터를 JSON 형식으로 서버에 전송 -- AGV
        payload2 = {
            'order_': order_,
            'class_name': className,
        }
        
        # Robotarm 서버로 데이터 전송
        robotarm_response = requests.post(robotarm_url, json=payload)
        print('Robotarm server response:', robotarm_response.text)
        
        # # AGV 서버로 데이터 전송
        # agv_response = requests.post(agv_url, json=payload2)
        # print('AGV server response:', agv_response.text)
        
        last_send_time = current_time  # 마지막 전송 시간 갱신

    # 표시된 프레임 출력
    cv2.imshow("YOLO", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        break

cap.release()
cv2.destroyAllWindows()
