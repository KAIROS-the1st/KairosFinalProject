import cv2
from roboflow import Roboflow

rf = Roboflow(api_key="API")
project = rf.workspace().project("white-cane-hyjat")
model = project.version(2).model

cap = cv2.VideoCapture(0)

# frame rate
cap.set(cv2.CAP_PROP_FPS, 30)

# image size
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # 카메라에서 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 읽어올 수 없습니다.")
        break

    # Roboflow 모델에 프레임 전달하여 예측 결과 얻기
    prediction_result = model.predict(frame, confidence=40, overlap=30).json()
    print(prediction_result)

    # 예측 결과를 프레임에 표시
    for detection in prediction_result["predictions"]:
        x, y, width, height = detection["x"], detection["y"], detection["width"], detection["height"]
        label = detection["class"]
        confidence = detection["confidence"]
        cv2.rectangle(frame, (int(x - width / 2), int(y - height / 2)), (int(x + width / 2), int(y + height / 2)), (36, 255, 12), 2)
        cv2.putText(frame, f"{label}: {confidence:.2f}", (int(x - width / 2), int(y - height / 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 2)

    # 프레임 화면에 표시
    cv2.imshow('Roboflow Object Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()