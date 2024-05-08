if __name__ == '__main__':
    from ultralytics import YOLO
    
    model = YOLO('yolov8n.pt')
    model.train(data="C:\\white_cane\\wc.yaml", epochs=50)

# from ultralytics import YOLO
# import cv2

# class_names = ["white cane", "person", "open", "close"]

# model = YOLO(r"runs\detect\train9\weights\best.pt")

# cap = cv2.VideoCapture(0)

# while True:
#     ret, frame = cap.read()
#     result = model(frame, stream=True)
    
#     for r in result:
#         boxes = r.boxes
#         for box in boxes:
#             cls = box.cls[0]
#             class_name = class_names[int(cls)]  # 클래스 번호에 해당하는 클래스 이름 가져오기
            
#             lx, ly, rx, ry = box.xyxy[0]
#             lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)
            
#             cv2.rectangle(frame, (lx, ly), (rx, ry), (0,255,255), 3)
#             conf = round(float(box.conf[0]), 2)
#             cv2.putText(frame, str(conf), (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
#             cv2.putText(frame, str(class_name), (lx, ly-25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
#     if cv2.waitKey(10) & 0xFF == ord('q'): break;
    
#     cv2.imshow("YOLO", frame)

# cap.release()
# cv2.destroyAllWindows()



