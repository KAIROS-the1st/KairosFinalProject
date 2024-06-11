import socket
from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/other', methods=['POST'])
def receive_message():
    data = request.json
    message = data.get('message')
    response = {
        'message_received': message
    }
    
    if message == "Your Trun":
        # ESP32 IP 주소와 포트 설정 (ESP32 시리얼 모니터에 표시된 IP 주소를 사용)
        esp32_ip = '172.30.1.49'
        esp32_port = 80

        # 소켓 생성
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((esp32_ip, esp32_port))

        # 메시지 전송
        msg = "Your Turn" + "\r\n"
        client_socket.sendall(msg.encode())

        # 서버로부터 응답 수신
        response = client_socket.recv(1024)
        print("Received from ESP32:", response.decode())

        # 소켓 닫기
        client_socket.close()
    
    return jsonify(response)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

