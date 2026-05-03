#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
unsigned long testStartTime;
unsigned long lastSentSec = 0;
bool isTesting = false;
float virtualWeight = 0;

// [추가] BLE 연결 상태를 관리하는 콜백 클래스
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println(">>> App Connected!");
      pServer->getAdvertising()->stop();
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println(">>> App Disconnected. Restarting Advertising...");
      // 연결이 끊기면 즉시 다시 광고를 시작하여 앱이 언제든 재연결되도록 함
      pServer->getAdvertising()->start();
    }
};

void setup() {
  Serial.begin(115200);
  
  BLEDevice::init("Espresso Scale");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks()); // 콜백 설정

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_READ
                    );
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID); // 서비스 UUID 등록
  pAdvertising->start();

  Serial.println("BLE Ready - 시리얼창에 's'를 치면 테스트를 시작합니다.");
}

void loop() {
  // 시리얼 입력으로 시작 제어 (저울의 버튼을 누르는 동작 대신)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      testStartTime = millis();
      lastSentSec = 0; 
      virtualWeight = 0;
      isTesting = true;
      Serial.println(">>> 추출 테스트 시작 (0초부터)");
    }
  }

  if (isTesting) {
    unsigned long now = millis();
    float seconds = (now - testStartTime) / 1000.0;
    unsigned long currentSec = (unsigned long)seconds;

    // 1초마다 데이터 전송
    if (currentSec <= 70) {
      if (currentSec > lastSentSec || (currentSec == 0 && lastSentSec == 0)) {
        
        // --- 에스프레소 유속 시뮬레이션 수식 ---
        float peakFlow = 3.9;
        float peakTime = 45.0; // peakTime을 늘려 완만한 상승 유도
        float flow = 0;

        if (seconds > 0) {
          float ratio = seconds / peakTime; 
          // Sine 기반 곡선: 0에서 우아하게 시작하여 peakFlow(3.9)를 넘지 않음
          flow = peakFlow * pow(sin(1.5708 * ratio), 3.0) * exp(1.0 - ratio);
        }

        // 안전장치 및 최소값 컷오프
        if (flow < 0.05) flow = 0;
        if (flow > 3.9) flow = 3.9;

        // 무게 누적
        virtualWeight += flow;

        // BLE 전송 및 시리얼 출력
        sendDataViaBLE(virtualWeight, flow, currentSec);
        
        lastSentSec = currentSec; 
        Serial.printf("T: %lu, F: %.2f, W: %.1f\n", currentSec, flow, virtualWeight);
      }
    } else {
      Serial.println(">>> 테스트 종료 (70초 도달)");
      isTesting = false;
    }
  }
  
  // BLE 스택의 안정성을 위해 루프 주기 조절
  delay(10); 
}

void sendDataViaBLE(float weight, float flowRate, unsigned long elapsed) {
  if (pCharacteristic == nullptr) return;
  char buffer[64];
  // 앱인벤터에서 콤마(,)로 분리할 수 있게 포맷팅
  snprintf(buffer, sizeof(buffer), "%.1f,%.2f,%lu", weight, flowRate, elapsed);
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}