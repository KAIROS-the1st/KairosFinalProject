# ProcessingNode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        self.subscription = self.create_subscription(Image, 'camera/image', self.listener_callback, 10)
        self.subscription
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Float32MultiArray, 'order', 10)
        # self.timer = self.create_timer(1, self.timer_callback)

        self.model = YOLO("/home/user/Downloads/best.pt")
        self.class_names = ["close", "open", "white cane"]

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _ = frame.shape

        result = self.model(frame, stream=True)

        for r in result:
            boxes = r.boxes
            for box in boxes:
                cls = box.cls[0]
                class_name = self.class_names[int(cls)]

                lx, ly, rx, ry = box.xyxy[0]
                lx, ly, rx, ry = int(lx), int(ly), int(rx), int(ry)

                conf = round(float(box.conf[0]), 2)
                if conf >= 0.6:
                    if class_name == "white cane":
                        cv2.rectangle(frame, (lx, ly), (rx, ry), (0,255,255), 3)
                        self.get_logger().info("*************white cane**************")
                        center_x = (lx + rx) // 2
                        center_y = (ly + ry) // 2
                        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                        self.calculation_infos(float(height), float(center_x))
                    elif class_name == "open":
                        cv2.rectangle(frame, (lx, ly), (rx, ry), (255,255,0), 3)
                        print("open")
                    elif class_name == "close":
                        cv2.rectangle(frame, (lx, ly), (rx, ry), (255,0,255), 3)
                        print("close")

                    cv2.putText(frame, str(conf), (center_x, center_y+25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(frame, str(class_name), (center_x, center_y-25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        cv2.imshow("Wind", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'): 
            self.cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info('Camera close')
            self.destroy_node()
            rclpy.shutdown()
    
    # def timer_callback(self):
    #     odr = String()
    #     odr.data = "test test 123"
    #     self.publisher_.publish(odr)
    #     self.get_logger().info('Sending successfully: "%s"' % odr.data)

    def calculation_infos(self, height, center_x):
        msg = Float32MultiArray()
        msg.data = [height, center_x]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing height: %d, center_x: %d' % (height, center_x))

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
