#this

import cv2
import numpy as np

# 카메라 열기
cap = cv2.VideoCapture(0)

while True:
    # 프레임 캡처
    ret, frame = cap.read()
    
    # QR 코드 검출
    qr = cv2.QRCodeDetector()
    data, box, straight_qrcode = qr.detectAndDecode(frame)
    
    # QR 코드 영역 표시
    if data:
        print('QR코드 데이터: {}'.format(data))
        print('QR코드 위치: {}'.format(box))
        
        # 사각형 그리기
        lefttop = int(box[0][0][0]), int(box[0][0][1])
        rightbottom = int(box[0][2][0]), int(box[0][2][1])
        cv2.rectangle(frame, lefttop, rightbottom, (0, 0, 255), 5)
    
    # 결과 화면 출력
    cv2.imshow('QR Code Detection', frame)
    
    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 카메라 및 창 종료
cap.release()
cv2.destroyAllWindows()
