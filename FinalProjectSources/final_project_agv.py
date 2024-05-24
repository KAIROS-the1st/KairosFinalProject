# AGV server communication


from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('PROTOCOL', methods=['POST'])
def upload():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'No data provided'}), 400

    # 받은 데이터 출력
    print(f"Received data: {data}")

    # 필요한 경우 데이터 처리
    order_ = data.get('order_')
    class_name = data.get('class_name')

    # 간단한 응답 반환
    response = "Data received successfully"
    
    return jsonify(response), 200

if __name__ == '__main__':
    app.run(host='HOST_IP', port="PORT")