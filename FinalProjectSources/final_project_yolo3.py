# To Linux, now discard
# Object detection send data


import cv2
from ultralytics import YOLO
import requests

# model
class_names = ["close", "open", "white cane"]
model = YOLO(r"runs\detect\train2\weights\best.pt")

# camera
cap = cv2.VideoCapture(0)

# 서버 URL
url = 'URL/PROTOCOL'

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    center_x = 0  # center_x 변수 초기화
    
    result = model(frame, stream=True)
    
    for r in result:
        boxes = r.boxes
        for box in boxes:
            cls = box.cls[0]
            class_name = class_names[int(cls)]  # 클래스 번호에 해당하는 클래스 이름 가져오기
            
            # 객체의 바운딩 박스 좌표 추출
            lx, ly, rx, ry = box.xyxy[0]
            lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)
            
            # 신뢰도로 한번 거름
            conf = round(float(box.conf[0]), 2)
            if conf >= 0.6:
                # 바운딩 박스 그리기
                if class_name == "white cane":
                    cv2.rectangle(frame, (lx, ly), (rx, ry), (0,255,255), 3)
                elif class_name == "open":
                    cv2.rectangle(frame, (lx, ly), (rx, ry), (255,255,0), 3)
                elif class_name == "close":
                    cv2.rectangle(frame, (lx, ly), (rx, ry), (255,0,255), 3)
                
                # 객체의 중심점 계산 및 표시
                center_x = (lx + rx) // 2
                center_y = (ly + ry) // 2
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

                # 신뢰도와 클래스 이름 표시
                cv2.putText(frame, str(conf), (center_x, center_y+25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, str(class_name), (center_x, center_y-25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 데이터를 JSON 형식으로 서버에 전송
    payload = {
        'width': width,
        'center_x': center_x,
    }
    response = requests.post(url, json=payload)

    # 응답 출력
    print(response.text)
    
    # 표시된 프레임 출력
    cv2.imshow("YOLO", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        break

cap.release()
cv2.destroyAllWindows()