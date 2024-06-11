// #include <WiFi.h>

// // WiFi 설정
// const char* ssid = "ConnectValue_A602_2G";  // WiFi SSID
// const char* password = "CVA602!@#$";  // WiFi 비밀번호

// void setup() {
//   Serial.begin(115200);
//   WiFi.begin(ssid, password);

//   Serial.print("Connecting to WiFi");

//   // WiFi 연결 시도
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.print(".");
//   }
  
//   Serial.println();
//   Serial.println("Connected to WiFi");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
// }

// void loop() {
//   // WiFi 연결 유지 확인
//   if (WiFi.status() == WL_CONNECTED) {
//     Serial.println("WiFi connected");
//   } else {
//     Serial.println("WiFi not connected");
//   }
//   delay(5000);  // 5초마다 상태 확인
// }



// START
// 모터를 제어할 핀 번호 정의
const int A1A = 2;  // 예시로 핀 번호를 설정
const int A1B = 4;  // 예시로 핀 번호를 설정
const int B1A = 16; // 예시로 핀 번호를 설정
const int B1B = 17; // 예시로 핀 번호를 설정

#include <WiFi.h>

// WiFi 설정
const char* ssid = "Dave";  // WiFi SSID
const char* password = "dave2009";  // WiFi 비밀번호

// 서버 설정
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
  
  server.begin();

  // 핀 모드 설정
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);
  pinMode(B1A, OUTPUT);
  pinMode(B1B, OUTPUT);

  // 초기에 정지 상태로 시작
  stop();
}

// 전진 함수
void forward() {
    digitalWrite(A1A, HIGH);
    digitalWrite(A1B, LOW);
    digitalWrite(B1A, HIGH);
    digitalWrite(B1B, LOW);
}

// 정지 함수
void stop() {
    digitalWrite(A1A, LOW);
    digitalWrite(A1B, LOW);
    digitalWrite(B1A, LOW);
    digitalWrite(B1B, LOW);
}

void loop() {
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("Client connected");
    String currentLine = "";
    
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\r');
        Serial.println(request);

        if (request.indexOf("Your Turn") != -1) {
          Serial.println("Moving forward");
          forward();
        }
        if (request.indexOf("Stop") != -1) {
          Serial.println("stop");
          stop();
        }
        
        if (request.length() == 0) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<body><h1>Hello from ESP32</h1></body>");
          client.println("</html>");
          client.println();
          break;
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}

