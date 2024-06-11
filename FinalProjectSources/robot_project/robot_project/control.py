# SUB
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymycobot.mycobot import MyCobot
import time

import requests

mc = MyCobot("/dev/ttyACM0", 115200)

def send_message_to_agvNode():
    agv_url = 'http://172.30.1.50:5000/agv_com'
    message = 'release OK'

    response = requests.post(agv_url, json={'message': message})
    
    if response.status_code == 200:
        print('메시지를 성공적으로 보냈습니다.')
    else:
        print('메시지 전송에 실패했습니다.')

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.subscription = self.create_subscription(
            String,
            'flask_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

    def listener_callback(self, msg):
        if msg.data == "right":
            self.get_logger().info('What I Received: "%s"' % msg.data)
            mc.send_angles([0,0,0,0,self.i,0],50)
            self.i += 5
        elif msg.data == "left":
            self.get_logger().info('What I Received: "%s"' % msg.data)
            mc.send_angles([0,0,0,0,self.i,0],50)
            self.i -= 5
        elif msg.data == "middle":
            self.get_logger().info('What I Received: "%s"' % msg.data)        
        elif msg.data == "grab":
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
        elif msg.data == "FIRST FLOOR":
            self.get_logger().info("first floor")
            mc.send_angles([0,0,0,0,0,0],70)
            time.sleep(5)

            mc.send_angles([90,35,50,-100,-90,0],70)
            time.sleep(3)

            mc.init_eletric_gripper()
            mc.set_eletric_gripper(1)
            mc.set_gripper_value(10,30)
            time.sleep(2)

            mc.send_angles([90,35,50,-90,-90,0],70)
            time.sleep(5)
            
            self.get_logger().info("after click")
            mc.send_angles([0,0,0,0,0,0],70)
            time.sleep(3)

            mc.init_eletric_gripper()
            mc.set_eletric_gripper(0)
            mc.set_gripper_value(90,30)
            time.sleep(2)
        elif msg.data == "PUSH":
            self.get_logger().info("push")
            mc.send_angles([0,0,0,0,0,0],70)
            time.sleep(5)

            mc.send_angles([90,35,50,-100,-90,0],70)
            time.sleep(3)

            mc.init_eletric_gripper()
            mc.set_eletric_gripper(1)
            mc.set_gripper_value(10,30)
            time.sleep(2)

            mc.send_angles([90,35,50,-90,-90,0],70)
            time.sleep(5)
            
            self.get_logger().info("after click")
            mc.send_angles([0,0,0,0,0,0],70)
            time.sleep(3)

            mc.init_eletric_gripper()
            mc.set_eletric_gripper(0)
            mc.set_gripper_value(90,30)
            time.sleep(2)
        elif msg.data == "release":
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
            send_message_to_agvNode()
        else:
            self.get_logger().info('I Received None: "%s"' % msg.data)

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