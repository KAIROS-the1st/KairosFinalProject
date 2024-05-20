# MovingNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyACM0", 115200)

class TopicCheck(Node):
    def __init__(self):
        super().__init__('apply_yolo')
        self.subscription = self.create_subscription(Float32MultiArray, 'order', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, odr):
        height, center_x = odr.data
        self.get_logger().info('What I Received >> height: %d, center_x: %d' % (height, center_x))

        partition = height // 3
        if partition > center_x: # right side
            self.get_logger().info("Adjusting1")
            mc.send_angles([0,0,0,0,20,0],50)
            time.sleep(1)
        elif partition * 2 < center_x: # left side
            self.get_logger().info("Adjusting2")
            mc.send_angles([0,0,0,0,-20,0],50)
            time.sleep(1) 
        else:
            self.get_logger().info("I grab")
            mc.init_eletric_gripper()
            mc.set_eletric_gripper(1)
            mc.set_gripper_value(10, 30)
            time.sleep(5)
            self.get_logger().info("I release")
            mc.init_eletric_gripper()
            mc.set_eletric_gripper(0)
            mc.set_gripper_value(50, 30)
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = TopicCheck()
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