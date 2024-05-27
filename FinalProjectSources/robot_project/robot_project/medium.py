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

    def upload_file(self):
        # JSON 데이터 받기
        data = request.get_json()
        width = data['width']
        center_x = data['center_x']

        # 받은 데이터 출력
        self.get_logger().info("Width: %s, Center X Coordinate: %s" % (width, center_x))
        msg = String()
        
        if not self.grab_sent:
            if center_x != 0:
                self.part = width // 5
                if self.part * 2 > center_x:
                    msg.data = "right"
                    self.consecutive_middle_count = 0  # 연속 카운트 초기화
                elif center_x > self.part * 3:
                    msg.data = "left"
                    self.consecutive_middle_count = 0  # 연속 카운트 초기화
                else:
                    msg.data = "middle"
                    self.consecutive_middle_count += 1
                    if self.consecutive_middle_count == 7:
                        msg.data = "grab"
                        # grab_msg = String()
                        # grab_msg.data = "grab"
                        # self.publisher.publish(grab_msg)
                        # self.get_logger().info("I sent %s" % (grab_msg.data))
                        self.grab_sent = True
            else:
                msg.data = "no detection"
                self.consecutive_middle_count = 0  # 연속 카운트 초기화
                
            self.publisher.publish(msg)
            self.get_logger().info("I sent %s" % (msg.data))
        
        return 'I received successfully'

    def upload_file_wrapper(self):
        return self.upload_file()

def main(args=None):
    rclpy.init(args=args)
    flask_node = FlaskPublisher()
    app.add_url_rule('/upload', 'upload_file', flask_node.upload_file_wrapper, methods=['POST'])
    app.run(host='0.0.0.0', port="5000")
    rclpy.spin(flask_node)
    flask_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

