# # test1 SUB
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# from pymycobot.mycobot import MyCobot
# import time

# mc = MyCobot("/dev/ttyACM0", 115200)

# class RobotControl(Node):
#     def __init__(self):
#         super().__init__('robot_control')
#         self.subscription = self.create_subscription(
#             String,
#             'topic',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         if msg.data == "white cane":
#             self.get_logger().info('What I Received: "%s"' % msg.data)
#             mc.send_angles([0,0,0,0,-60,0],50)
#             time.sleep(1)
#             mc.send_angles([0,0,0,0,60,0],50)
#             time.sleep(1)
#             mc.send_angles([0,0,0,0,0,0],50)
#             time.sleep(1)
#         elif msg.data == "close":
#             self.get_logger().info('What I Received: "%s" means STOP' % msg.data)
#         elif msg.data == "open":
#             self.get_logger().info('What I Received: "%s" means GO' % msg.data)
#         else:
#             self.get_logger().info('What I Received: "%s"' % msg.data)
        
#         if msg.data == "grab":
#             self.get_logger().info("I grab")
#             mc.init_eletric_gripper()
#             mc.set_eletric_gripper(1)
#             mc.set_gripper_value(10, 30)
#             time.sleep(10)
#             self.get_logger().info("I release")
#             mc.init_eletric_gripper()
#             mc.set_eletric_gripper(0)
#             mc.set_gripper_value(50, 30)
#             time.sleep(3)

# def main(args=None):
#     rclpy.init(args=args)

#     robot_control = RobotControl()

#     rclpy.spin(robot_control)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     robot_control.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymycobot.mycobot import MyCobot

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'detection/cane_coords', self.listener_callback, 10)
        self.mycobot = MyCobot("/dev/ttyACM0", 115200)

    def listener_callback(self, msg):
        if not msg.data:
            return  # No cane detected

        x_center, y_center = msg.data

        # Define regions in the camera window
        window_width = 640  # Adjust based on your camera resolution
        right_threshold = window_width * 2 / 3
        left_threshold = window_width / 3

        if x_center > right_threshold:
            self.turn_joint_5('right')
        elif x_center < left_threshold:
            self.turn_joint_5('left')
        else:
            self.turn_joint_5('center')

    def turn_joint_5(self, direction):
        angles = self.mycobot.get_angles()
        joint_5_angle = angles[4]

        if direction == 'right':
            joint_5_angle += 5  # Adjust the increment as needed
        elif direction == 'left':
            joint_5_angle -= 5  # Adjust the decrement as needed
        elif direction == 'center':
            pass  # Optionally, implement logic to center the joint

        self.mycobot.send_angles([angles[0], angles[1], angles[2], angles[3], joint_5_angle, angles[5]], 50)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()