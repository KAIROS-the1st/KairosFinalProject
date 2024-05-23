# Server for app and robotarm start


from flask import Flask, request, jsonify

app = Flask(__name__)

# 예시 API 엔드포인트
@app.route('PROTOCOL', methods=['POST'])

def send_message():
    data = request.get_json()
    message = data.get('message')
    # 여기에서 라즈베리 파이가 받은 메시지 처리
    print("Received message:", message)
    return jsonify({"status": "success"})

if __name__ == '__main__':
    app.run(host='HOST_IP', port="PORT")  # Flask 서버 실행
