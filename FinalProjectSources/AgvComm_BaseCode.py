# AGV 통신 노드 Base Code
from flask import Flask, request, jsonify
import requests

app = Flask(__name__)

# /agv_com 으로 들어오는 메세지 받음: open, close from Master / release OK from ARM
@app.route('/agv_com', methods=['POST'])
def run_script():
    data = request.get_json()
    print(data)
    # message = data.get('order_', 'class_name')
    
    # print("Received message:", message)

    return jsonify({"status": "success"})

def send_messages():
    robotarm_url = 'http://172.20.179.107:5000/release'
    message = 'release'

    response = requests.post(robotarm_url, json={'message': message})
    
    if response.status_code == 200:
        print('메시지를 성공적으로 보냈습니다.')
    else:
        print('메시지 전송에 실패했습니다.')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
