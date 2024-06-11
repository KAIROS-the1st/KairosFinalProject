import socket
import time

# ESP32 IP 주소와 포트 설정 (ESP32 시리얼 모니터에 표시된 IP 주소를 사용)
esp32_ip = '192.168.81.194'
esp32_port = 80

# 소켓 생성
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((esp32_ip, esp32_port))

# 메시지 전송
message = "Your Turn" + "\r\n"
client_socket.sendall(message.encode())
time.sleep(10)
message = "Stop" + "\r\n"
client_socket.sendall(message.encode())

# 서버로부터 응답 수신
response = client_socket.recv(1024)
print("Received from ESP32:", response.decode())

# 소켓 닫기
client_socket.close()

