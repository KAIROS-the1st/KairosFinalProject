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

    def upload_file(self):
        # JSON 데이터 받기
        data = request.get_json()
        width = data['width']
        center_x = data['center_x']

        # 받은 데이터 출력
        self.get_logger().info("Width: %s, Center X Coordinate: %s" % (width, center_x))
        # # ROS 2 메시지로 변환하여 발행
        # msg = Int32MultiArray()
        # msg.data = [width, center_x]
        msg = String()
        part = width // 3
        if part > center_x:
            msg.data = "right"
        elif center_x > part * 2:
            msg.data = "left"
        else:
            msg.data = "middle"
            
        self.publisher.publish(msg)
        self.get_logger().info("I sent %s" % (msg.data))

        return 'I received successfully'

    def upload_file_wrapper(self):
        return self.upload_file()

def main(args=None):
    rclpy.init(args=args)
    flask_node = FlaskPublisher()
    app.add_url_rule('protocol', 'upload_file', flask_node.upload_file_wrapper, methods=['POST'])
    app.run(host='ip', port="port")
    rclpy.spin(flask_node)
    flask_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

