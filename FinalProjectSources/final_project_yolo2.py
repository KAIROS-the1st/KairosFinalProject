import cv2
from ultralytics import YOLO

class_names = ["white cane", "open", "close", "person"]

model = YOLO(r"runs\detect\train17\weights\best.pt")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    
    # 화면을 세로로 세분할해서 가운데 부분을 ROI로 지정
    height, width, _ = frame.shape
    roi_start = int(width / 3)
    roi_end = int(width * 2 / 3)
    roi_frame = frame[:, roi_start:roi_end]
    
    # ROI 영역을 화면에 표시
    cv2.rectangle(frame, (roi_start, 0), (roi_end, height), (0, 0, 255), 2)
    
    result = model(roi_frame, stream=True)
    
    for r in result:
        boxes = r.boxes
        for box in boxes:
            cls = box.cls[0]
            class_name = class_names[int(cls)]  # 클래스 번호에 해당하는 클래스 이름 가져오기
            
            # 객체의 바운딩 박스 좌표 추출
            lx, ly, rx, ry = box.xyxy[0]
            lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)
            
            # roi_frame에서 전체 frame 좌표로 변환
            lx += roi_start
            rx += roi_start
            
            # 바운딩 박스 그리기
            if class_name == "white cane":
                cv2.rectangle(frame, (lx, ly), (rx, ry), (0,255,255), 3)
            elif class_name == "open":
                cv2.rectangle(frame, (lx, ly), (rx, ry), (255,255,0), 3)
            elif class_name == "close":
                cv2.rectangle(frame, (lx, ly), (rx, ry), (255,0,255), 3)
            elif class_name == "person":
                cv2.rectangle(frame, (lx, ly), (rx, ry), (255,255,255), 3)
            
            # 객체의 중심점 계산 및 표시
            center_x = (lx + rx) // 2
            center_y = (ly + ry) // 2
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            print(center_x, center_y)

            # 신뢰도와 클래스 이름 표시
            conf = round(float(box.conf[0]), 2)
            cv2.putText(frame, str(conf), (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, str(class_name), (center_x, center_y-25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 표시된 프레임 출력
    cv2.imshow("YOLO", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        break;

cap.release()
cv2.destroyAllWindows()
