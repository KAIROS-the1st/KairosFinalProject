import serial
import struct
from flask import Flask, request, jsonify
import os, requests

app = Flask(__name__)

@app.route('/plc', methods=['POST'])
def receive_message():
    global message
    data = request.get_json()
    message = data.get('message')
    
    print("Received message:", message)
    
    response = {
        'message_received': message
    }
    main()
    
    return jsonify(response)


# 시리얼 포트 설정
ser = serial.Serial(
    port = 'COM4',  # 시리얼 포트 이름에 따라 변경해야 합니다.
    baudrate = 9600, # Fit as PLC set
    bytesize = 8,
    parity = 'N', # Parity is NONE
    stopbits = 1,
    timeout = 1
)

# CRC16 체크섬 계산 함수
def calculate_modbus_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder='little')

def build_modbus_frame(device_address, function_code, *args): # the value must be 1 byte for correct CRC calculation.
    hex_codes = []
    hex_codes.append(device_address)
    hex_codes.append(function_code)
    for arg in args:
        hex_codes.append(arg)
    CRC_BUFFER = calculate_modbus_crc(hex_codes)
    hex_codes.extend(CRC_BUFFER)
    print(hex_codes)
    return hex_codes

# ##### CRC TESTBED #####
# crc_test = [0x11, 0x06, 0x00, 0x01, 0x00, 0x03] # CRC result must be '0x9A9B'
# print(calculate_modbus_crc(crc_test))
# ### CRC TESTBED END ###

# MODBUS 통신 함수
def modbus_communication(device_address, function_code, *args):
    #hex_codes = [0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x85, 0xdb]  # 예시로 임의의 프레임을 설정하였습니다.
    #ser.write(bytearray(hex_codes))

    ser.write(build_modbus_frame(device_address, function_code, *args))

    if(function_code in [0x05,0x06,0x0F,0x10]):
        response = ser.read(8) # 응답받은 데이터의 행렬 - 데이터의 길이만큼
    elif(function_code in [0x03, 0x04]):
        response = ser.read(3) # 응답받은 데이터의 행렬 - 데이터의 길이만큼
        # 장리 필요
    #print(response)

    crc = calculate_modbus_crc(response[:-2])
    print(crc)
    if((crc[0] == response[-2])and(crc[1] == response[-1])):
        print('OK')
    else:
        print('Not OK')
    return response

def main_menu():
    print("-------------------------------------------------")
    print(" LIST MENU");
    print("-------------------------------------------------")
    print(" 0. serial")
    print(" 1. 00 01 00 00 00 01")
    print(" 2. 00 02 00 00 00 01")
    print(" 3. 00 03 00 00 00 01")
    print(" 4. serial")
    print(" 5. serial")
    print(" 6. serial")
    '''
    print(" a. serial")
    print(" b. serial")
    print(" c. serial")
    print(" d. serial")
    print(" e. serial")
    print(" f. serial")
    print(" G. serial")
    '''
    print(" Z. crc test")
    print("-------------------------------------------------")
    print(" q.  QUIT");
    print("-------------------------------------------------")
    print()
    #print("SELECT THE COMMAND NUMBER : ")
    key = input("SELECT THE COMMAND NUMBER : ")
    return key

def main():
    while True:
        # user_input = main_menu()
        user_input = message
        print(user_input, type(message))
        if user_input == '0':
            pass
        elif user_input == '1':
            try:
                
                DEVICE_ADDRESS = 0x00           # 장치 주소
                FUNCTION_CODE = 0x01   # 함수 코드 (Function Code)
                REGISTER_ADDRESS_HI = 0x00       # 읽을 레지스터 주소와 개수
                REGISTER_ADDRESS_LO = 0x00
                REGISTER_NUM_HI = 0x00
                REGISTER_NUM_LO = 0x01
                # 모드버스 통신 실행
                register_values = modbus_communication(DEVICE_ADDRESS, FUNCTION_CODE, REGISTER_NUM_HI, REGISTER_NUM_LO, REGISTER_ADDRESS_HI, REGISTER_ADDRESS_LO)
                if register_values is not None:
                    print("Register Value:", register_values)
                else:
                    print("Modbus Communication Failed")
            except Exception as e:
                print("Error:", e)
        elif user_input == '2':
            try:
                DEVICE_ADDRESS = 0x00           # 장치 주소
                FUNCTION_CODE = 0x02   # 함수 코드 (Function Code)
                REGISTER_ADDRESS_HI = 0x00       # 읽을 레지스터 주소와 개수
                REGISTER_ADDRESS_LO = 0x00
                REGISTER_NUM_HI = 0x00
                REGISTER_NUM_LO = 0x01
                # 모드버스 통신 실행
                register_values = modbus_communication(DEVICE_ADDRESS, FUNCTION_CODE, REGISTER_NUM_HI, REGISTER_NUM_LO, REGISTER_ADDRESS_HI, REGISTER_ADDRESS_LO)
                if register_values is not None:
                    print("Register Value:", register_values)
                else:
                    print("Modbus Communication Failed")
            except Exception as e:
                print("Error:", e)
        elif user_input == '3':
            try:
                DEVICE_ADDRESS = 0x00           # 장치 주소
                FUNCTION_CODE = 0x03   # 함수 코드 (Function Code)
                REGISTER_ADDRESS_HI = 0x00       # 읽을 레지스터 주소와 개수
                REGISTER_ADDRESS_LO = 0x00
                REGISTER_NUM_HI = 0x00
                REGISTER_NUM_LO = 0x01
                # 모드버스 통신 실행
                register_values = modbus_communication(DEVICE_ADDRESS, FUNCTION_CODE, REGISTER_NUM_HI, REGISTER_NUM_LO, REGISTER_ADDRESS_HI, REGISTER_ADDRESS_LO)
                if register_values is not None:
                    print("Register Value:", register_values)
                else:
                    print("Modbus Communication Failed")
            except Exception as e:
                print("Error:", e)
        elif user_input == '4':
            try:
                DEVICE_ADDRESS = 0x00           # 장치 주소
                FUNCTION_CODE = 0x06   # 함수 코드 (Function Code)
                REGISTER_ADDRESS_HI = 0x00       # 읽을 레지스터 주소와 개수
                REGISTER_ADDRESS_LO = 0x00
                REGISTER_NUM_HI = 0x00
                REGISTER_NUM_LO = 0x10
                # 모드버스 통신 실행
                register_values = modbus_communication(DEVICE_ADDRESS, FUNCTION_CODE,REGISTER_ADDRESS_HI, REGISTER_ADDRESS_LO, REGISTER_NUM_HI, REGISTER_NUM_LO)
                if register_values is not None:
                    print("Register Value:", register_values)
                else:
                    print("Modbus Communication Failed")
            except Exception as e:
                print("Error:", e)

        elif user_input == '5':
            try:
                DEVICE_ADDRESS = 0x00           # 장치 주소
                FUNCTION_CODE = 0x06   # 함수 코드 (Function Code)
                REGISTER_ADDRESS_HI = 0x00       # 읽을 레지스터 주소와 개수
                REGISTER_ADDRESS_LO = 0x00
                REGISTER_NUM_HI = 0x00
                REGISTER_NUM_LO = 0x20
                # 모드버스 통신 실행
                register_values = modbus_communication(DEVICE_ADDRESS, FUNCTION_CODE,REGISTER_ADDRESS_HI, REGISTER_ADDRESS_LO, REGISTER_NUM_HI, REGISTER_NUM_LO)
                if register_values is not None:
                    print("Register Value:", register_values)
                else:
                    print("Modbus Communication Failed")
            except Exception as e:
                print("Error:", e)

        elif user_input == '6':
            try:
                DEVICE_ADDRESS = 0x00           # 장치 주소
                FUNCTION_CODE = 0x06   # 함수 코드 (Function Code)
                REGISTER_ADDRESS_HI = 0x00       # 읽을 레지스터 주소와 개수
                REGISTER_ADDRESS_LO = 0x00
                REGISTER_NUM_HI = 0x00
                REGISTER_NUM_LO = 0x30
                # 모드버스 통신 실행
                register_values = modbus_communication(DEVICE_ADDRESS, FUNCTION_CODE,REGISTER_ADDRESS_HI, REGISTER_ADDRESS_LO, REGISTER_NUM_HI, REGISTER_NUM_LO)
                if register_values is not None:
                    print("Register Value:", register_values)
                else:
                    print("Modbus Communication Failed")
            except Exception as e:
                print("Error:", e)

        elif user_input == '7':
            pass

        elif user_input == '8':
            pass

        ###############################################################
        elif user_input == 'a':
            pass

        elif user_input == 'b':
            pass

        elif user_input == 'c':
            pass

        elif user_input == 'd':
            pass

        elif user_input == 'e':
            pass

        elif user_input == 'f':
            pass

        ###############################################################
        elif user_input == 'A':
            pass

        elif user_input == 'B':
            pass

        elif user_input == 'C':
            pass

        elif user_input == 'D':
            pass

        elif user_input == 'E':
            pass

        elif user_input == 'F':
            pass

        elif user_input == 'G':
            pass

        elif user_input == 'H':
            pass

        ###############################################################
        elif user_input == 'Z':
            CRC_DATA = [0x00, 0x03, 0x00, 0x00, 0x00, 0x01]
            AC = calculate_modbus_crc(bytes(CRC_DATA))  # 리스트를 바이트 데이터로 변환
            print(AC)

        elif user_input == 'q':
            print("You pressed 'q'. Exiting the loop.")
            ser.close()  # 시리얼 포트 닫기
            #time.sleep(3)
            break               # 'a'를 입력받으면 반복문을 종료합니다.

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
