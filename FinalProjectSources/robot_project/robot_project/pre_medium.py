from flask import Flask, request
import numpy as np

app = Flask(__name__)

@app.route('/upload', methods=['POST'])
def upload_file():
    # JSON 데이터 받기
    data = request.get_json()
    width = data['width']
    center_x = data['center_x']

    # 받은 데이터 출력
    print("Width:", width)
    print("Center X Coordinate:", center_x)

    return 'I received successfully'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

