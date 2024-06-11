# PUB
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request

app = Flask(__name__)

class FlaskPublisher(Node):
    def __init__(self):
        super().__init__('flask_publisher_node')
        self.publisher = self.create_publisher(String, 'flask_data', 10)
        self.part = 0
        self.consecutive_middle_count = 0  # 연속 middle 메시지 카운트
        self.grab_sent = False  # grab 메시지 전송 여부
        self.grab_ff = False  # first floor 메시지 전송 여부
        self.grab_push = False  # push 메시지 전송 여부
        self.release_sent = False

    def process_robotarm_message(self, data):
        if 'qr_data' in data:
            msg = String()
            msg.data = data['qr_data']
            if not self.grab_ff and msg.data == "FIRST FLOOR":
                self.publisher.publish(msg)
                self.get_logger().info("I sent QR message: %s" % (msg.data))
                self.grab_ff = True
            elif not self.grab_push and msg.data == "PUSH":
                self.publisher.publish(msg)
                self.get_logger().info("I sent QR message: %s" % (msg.data))
                self.grab_push = True
            return 'QR data processed successfully'
        
        if 'width' in data and 'center_x' in data:
            width = data['width']
            center_x = data['center_x']

            self.get_logger().info("Width: %s, Center X Coordinate: %s" % (width, center_x))
            msg = String()
            
            if not self.grab_sent:
                if center_x != 0:
                    self.part = width // 5
                    if self.part * 2 > center_x:
                        msg.data = "right"
                        self.consecutive_middle_count = 0
                    elif center_x > self.part * 3:
                        msg.data = "left"
                        self.consecutive_middle_count = 0
                    else:
                        msg.data = "middle"
                        self.consecutive_middle_count += 1
                        if self.consecutive_middle_count == 5:
                            msg.data = "grab"
                            self.grab_sent = True
                else:
                    msg.data = "no detection"
                    self.consecutive_middle_count = 0
                
                self.publisher.publish(msg)
                self.get_logger().info("I sent %s" % (msg.data))
            
            return 'White cane data processed successfully'

        return 'Invalid data received'
    
    def process_release_message(self, data):
        msg = String()
        msg.data = data.get['message']
        if not self.release_sent:
            self.publisher.publish(msg)
            self.get_logger().info("I sent release message: %s" % (msg.data))
            self.release_sent = True
        return 'Release data processed successfully'

    def upload_file(self):
        # JSON 데이터 받기
        data = request.get_json()
        response = self.process_robotarm_message(data)
        return response

    def upload_file_wrapper(self):
        return self.upload_file()
    
    def release(self):
        data = request.get_json()
        response = self.process_release_message(data)
        return response

    def release_wrapper(self):
        return self.release()

def main(args=None):
    rclpy.init(args=args)
    flask_node = FlaskPublisher()
    app.add_url_rule('/upload', 'upload_file', flask_node.upload_file_wrapper, methods=['POST'])
    app.add_url_rule('/release', 'release', flask_node.release_wrapper, methods=['POST'])
    app.run(host='0.0.0.0', port="5000")
    rclpy.spin(flask_node)
    flask_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



