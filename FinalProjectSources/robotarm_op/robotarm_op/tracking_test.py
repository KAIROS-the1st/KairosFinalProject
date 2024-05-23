# # test1 PUB
# import cv2
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from ultralytics import YOLO

# class ObjectTracking(Node):
#     def __init__(self):
#         super().__init__('object_tracking')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5 # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.model = YOLO(r"/home/user/Downloads/best.pt")
#         self.class_names = ["close", "open", "white cane"]

#     def timer_callback(self):
#         msg = String()

#         # Object detection
#         cap = cv2.VideoCapture(0)
#         ret, frame = cap.read()
#         result = self.model(frame, stream=True)

#         for r in result:
#             boxes = r.boxes
#             for box in boxes:
#                 cls = box.cls[0]
#                 class_name = self.class_names[int(cls)]

#                 lx, ly, rx, ry = box.xyxy[0]
#                 lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)

#                 conf = round(float(box.conf[0]), 2)
#                 if conf >= 0.6:
#                     if class_name == "white cane":
#                         cv2.rectangle(frame, (lx, ly), (rx, ry), (0,255,255), 3)
#                         msg.data = "white cane"
#                     elif class_name == "open":
#                         cv2.rectangle(frame, (lx, ly), (rx, ry), (255,255,0), 3)
#                         msg.data = "open"
#                     elif class_name == "close":
#                         cv2.rectangle(frame, (lx, ly), (rx, ry), (255,0,255), 3)
#                         msg.data = "close"

#                     center_x = (lx + rx) // 2
#                     center_y = (ly + ry) // 2
#                     cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

#                     cv2.putText(frame, str(conf), (center_x, center_y+25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
#                     cv2.putText(frame, str(class_name), (center_x, center_y-25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         # Show frame
#         cv2.imshow("YOLO", frame)
#         if cv2.waitKey(10) & 0xFF == ord('q'):
#             cap.release()
#             cv2.destroyAllWindows()
#             self.get_logger().info('Camera close')
#             self.destroy_node()
#             rclpy.shutdown()

#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)

# def main(args=None):
#     rclpy.init(args=args)
#     object_tracking = ObjectTracking()
#     rclpy.spin(object_tracking)
#     object_tracking.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import json

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(Image, 'camera/image', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'detection/cane_coords', 10)
        self.bridge = CvBridge()
        self.model = YOLO("/home/user/Downloads/best.pt")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame)

        cane_coords = Float32MultiArray()
        for result in results:
            for bbox in result.boxes:
                print(bbox.cls)
                if bbox.cls == 'white cane':
                    x_center = (bbox.xyxy[0] + bbox.xyxy[2]) / 2
                    y_center = (bbox.xyxy[1] + bbox.xyxy[3]) / 2
                    cane_coords.data = [x_center, y_center]
                    self.publisher_.publish(cane_coords)
                    return

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
