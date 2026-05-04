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

  Serial.println("BLE Ready - type in serial input 's'.");
}

void loop() {
  // serial iput 
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      testStartTime = millis();
      lastSentSec = 0; 
      virtualWeight = 0;
      isTesting = true;
      Serial.println(">>> test start");
    }
  }

  if (isTesting) {
    unsigned long now = millis();
    float seconds = (now - testStartTime) / 1000.0;
    unsigned long currentSec = (unsigned long)seconds;

    if (currentSec <= 70) {
      if (currentSec > lastSentSec || (currentSec == 0 && lastSentSec == 0)) {
        
        // --- espresso flow rate simulation ---
        float peakFlow = 3.9;
        float peakTime = 45.0;
        float flow = 0;

        if (seconds > 0) {
          float ratio = seconds / peakTime; 
          flow = peakFlow * pow(sin(1.5708 * ratio), 3.0) * exp(1.0 - ratio);
        }

        if (flow < 0.05) flow = 0;
        if (flow > 3.9) flow = 3.9;

        virtualWeight += flow;

        sendDataViaBLE(virtualWeight, flow, currentSec);
        
        lastSentSec = currentSec; 
        Serial.printf("T: %lu, F: %.2f, W: %.1f\n", currentSec, flow, virtualWeight);
      }
    } else {
      Serial.println(">>> test finished (70 seconds)");
      isTesting = false;
    }
  }
  
  delay(10); 
}

void sendDataViaBLE(float weight, float flowRate, unsigned long elapsed) {
  if (pCharacteristic == nullptr) return;
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%.1f,%.2f,%lu", weight, flowRate, elapsed);
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}