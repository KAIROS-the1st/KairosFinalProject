# for test
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
agv_url = 'http://172.30.1.50:5000/agv_com'

# 메시지 전송 주기 설정 (초)
send_interval = 0.5
last_send_time = time.time()

def send_to_server(url, payload):
    response = requests.post(url, json=payload)
    print(f'Server response from {url}:', response.text)

def detect_largest_object(frame):
    largest_box = None
    largest_area = 0
    result = model(frame, stream=True)

    for r in result:
        for box in r.boxes:
            cls = box.cls[0]
            class_name = class_names[int(cls)]

            lx, ly, rx, ry = map(int, box.xyxy[0])
            conf = round(float(box.conf[0]), 2)

            if conf >= 0.6:
                area = (rx - lx) * (ry - ly)
                if area > largest_area:
                    largest_area = area
                    largest_box = (box, class_name, lx, ly, rx, ry)

    return largest_box

def process_frame(frame):
    largest_box = detect_largest_object(frame)

    if largest_box:
        box, class_name, lx, ly, rx, ry = largest_box

        # 바운딩 박스 그리기
        color = (0, 255, 255) if class_name == "white cane" else (255, 255, 0) if class_name == "open" else (255, 0, 255)
        cv2.rectangle(frame, (lx, ly), (rx, ry), color, 3)

        # 객체의 중심점 계산 및 표시
        center_x = (lx + rx) // 2
        center_y = (ly + ry) // 2
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

        # 신뢰도와 클래스 이름 표시
        conf = round(float(box.conf[0]), 2)
        cv2.putText(frame, str(conf), (center_x, center_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, class_name, (center_x, center_y - 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return frame, class_name, center_x
    return frame, None, 0

def detect_qr_code(frame):
    qr = cv2.QRCodeDetector()
    data, box, _ = qr.detectAndDecode(frame)

    if data:
        lefttop = int(box[0][0][0]), int(box[0][0][1])
        rightbottom = int(box[0][2][0]), int(box[0][2][1])
        cv2.rectangle(frame, lefttop, rightbottom, (0, 0, 255), 5)

    return frame, data

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame, class_name, center_x = process_frame(frame)

    current_time = time.time()
    if current_time - last_send_time >= send_interval:
        
        # response = {'order_': "open"}
        # send_to_server(agv_url, response)
    
        frame, qr_data = detect_qr_code(frame)  # QR 코드를 검출하고 데이터를 얻음
        if qr_data:
            payload = {'qr_data': qr_data}
            send_to_server(robotarm_url, payload)
            last_send_time = current_time  # QR 코드를 감지한 경우 메시지를 전송하고 마지막 전송 시간 갱신
        elif class_name == "white cane":
            payload = {'width': frame.shape[1], 'center_x': center_x}
            send_to_server(robotarm_url, payload)
            payload2 = {'order_': "close"}
            send_to_server(agv_url, payload2)
        elif class_name in ["open", "close"]:
            payload = {'order_': "go" if class_name == "open" else "stop"}
            send_to_server(agv_url, payload)
        
        last_send_time = current_time  # 마지막 전송 시간 갱신

    cv2.imshow("YOLO & QR Code Detection", frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
