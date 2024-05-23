# SUB
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyACM0", 115200)

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.subscription = self.create_subscription(
            String,
            'flask_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 20

    def listener_callback(self, msg):
        if msg.data == "right":
            self.get_logger().info('What I Received: "%s"' % msg.data)
            mc.send_angles([0,0,0,0,self.i,0],50)
            time.sleep(1)
            self.i += 1
        elif msg.data == "left":
            self.get_logger().info('What I Received: "%s"' % msg.data)
            mc.send_angles([0,0,0,0,-self.i,0],50)
            time.sleep(1)
            self.i -= 1
        elif msg.data == "middle":
            self.get_logger().info('What I Received: "%s"' % msg.data)
            time.sleep(1)
        else:
            self.get_logger().info('I Received None: "%s"' % msg.data)
        
        if msg.data == "grab":
            self.get_logger().info("I grab")
            mc.init_eletric_gripper()
            mc.set_eletric_gripper(1)
            mc.set_gripper_value(10, 30)
            time.sleep(10)
            self.get_logger().info("I release")
            mc.init_eletric_gripper()
            mc.set_eletric_gripper(0)
            mc.set_gripper_value(50, 30)
            time.sleep(3)

def main(args=None):
    rclpy.init(args=args)

    robot_control = RobotControl()

    rclpy.spin(robot_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
