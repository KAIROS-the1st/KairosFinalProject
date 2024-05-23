# PUB
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time, random
import cv2
from ultralytics import YOLO

class ObjectTracking(Node):
    def __init__(self):
        super().__init__('object_tracking')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        # example
        signs = ["right", "left", "middle"]
        n = random.randint(0,2)

        if self.i == 5:
            msg.data = "grab"
            self.publisher_.publish(msg)
            self.get_logger().info("Go to grab")
            self.destroy_node()
            rclpy.shutdown()

        if signs[n] == "right": msg.data = "right"
        elif signs[n] == "left": msg.data = "left"
        elif signs[n] == "middle":
            msg.data = "middle"
            self.i = self.i + 1
        else: msg.data = "zero"

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)        

def main(args=None):
    rclpy.init(args=args)

    object_tracking = ObjectTracking()

    rclpy.spin(object_tracking)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_tracking.destroy_node()
    rclpy.shutdown()

def camera_op():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        
        cv2.imshow("Win", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'): 
            break;

if __name__ == '__main__':
    main()





# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import cv2
# import threading

# class CameraThread(threading.Thread):
#     def __init__(self, node):
#         super().__init__()
#         self.node = node
#         self.running = True

#     def run(self):
#         cap = cv2.VideoCapture(0)

#         while self.running:
#             ret, frame = cap.read()
#             cv2.imshow("Win", frame)

#             if cv2.waitKey(10) & 0xFF == ord('q'):
#                 break

#         # Release resources
#         cap.release()
#         cv2.destroyAllWindows()

#     def stop(self):
#         self.running = False


# class ObjectTracking(Node):
#     def __init__(self):
#         super().__init__('object_tracking')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 10  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#         self.camera_thread = CameraThread(self)
#         self.camera_thread.start()

#     def timer_callback(self):
#         # Publish ROS message
#         msg = String()
#         msg.data = "zero"
#         self.node.publisher_.publish(msg)
#         self.node.get_logger().info('Publishing: "%s"' % msg.data)

#     def shutdown(self):
#         self.camera_thread.stop()
#         self.camera_thread.join()


# def main(args=None):
#     rclpy.init(args=args)

#     object_tracking = ObjectTracking()

#     try:
#         rclpy.spin(object_tracking)
#     finally:
#         object_tracking.shutdown()

#         # Destroy the node explicitly
#         object_tracking.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

