#include <Arduino.h>
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ==================== Pin mapping ====================
#define LOADCELL_DOUT_PIN  2
#define LOADCELL_SCK_PIN   3
#define TOUCH_BUTTON_PIN   5

// ==================== Constants ====================
#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64      // 0.96" OLED
#define OLED_ADDR          0x3C

#define TARE_HOLD_TIME     100     
#define EXTRACT_HOLD_TIME  2000     

#define NO_FLOW_TIMEOUT    2000    
#define EXTRACT_TIMEOUT    80000   

#define MIN_WEIGHT_FOR_STABLE  5.0
#define STABLE_DURATION_MS     2000

#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Fixed calibration factor (from separate test)
#define CALIBRATION_FACTOR 1603.66f

// ==================== Objects ====================
HX711 scale;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BLECharacteristic *pCharacteristic;

// ==================== Variables ====================
enum DeviceMode { MODE_NORMAL, MODE_EXTRACT };
DeviceMode currentMode = MODE_NORMAL;

float extractStartWeight = 0;
float lastWeight = 0;
float lastFlowRate = 0;
unsigned long lastWeightChangeTime = 0;
unsigned long extractStartTime = 0;
float lastFilteredWeight = 0;
bool firstExtractSample = true;

unsigned long buttonPressStart = 0;
bool buttonPressed = false;
bool buttonHandled = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
      Serial.println(">>> App Connected!"); 
      pServer->getAdvertising()->stop();
      }
    void onDisconnect(BLEServer* pServer) {
      Serial.println(">>> App Disconnected. Restarting Advertising...");
      pServer->getAdvertising()->start();
    }
};

void setupHardware();
void setupDisplay();
void setupBLE();
void readButton();
void processButtonAction(unsigned long holdTime);
void updateNormalDisplay(float weight);
void updateExtractDisplay(float netWeight);
void updateFinalDisplay(float netWeight, unsigned long elapsed);
void sendDataViaBLE(float weight, float flowRate, unsigned long elapsed);
float readWeight();
float calculateFlowRate(float currentWeight, float lastWeight, unsigned long deltaTime);
void performTare();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-C3 Espresso Scale Starting...");
  
  setupDisplay();
  setupHardware();
  setupBLE();
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  Serial.println("HX711 initialized with fixed calibration factor");
  
  performTare();
  Serial.println("Auto tare done at startup");
  
  Serial.println("Ready!");
  updateNormalDisplay(readWeight());
}

void loop() {
  readButton();
  
  unsigned long now = millis();
  static unsigned long lastTime = 0;
  static float lastWeightForFlow = 0;
  unsigned long deltaTime = now - lastTime;
  
  if (deltaTime >= 100) {
    float currentWeight = readWeight();
    float flowRate = calculateFlowRate(currentWeight, lastWeightForFlow, deltaTime);
    
    switch (currentMode) {
      
      case MODE_NORMAL: {
        static float lockedWeight = 0.0;     
        static float lastRawWeight = 0.0;    

        float smoothWeight = (currentWeight * 0.95) + (lastRawWeight * 0.05);
        lastRawWeight = smoothWeight;

        if (smoothWeight - lockedWeight < -30.0) { 
          scale.tare();
          lockedWeight = 0.0;
          smoothWeight = 0.0;
        }

        float diff = abs(smoothWeight - lockedWeight);
        if (lockedWeight == 0.0) {
          if (diff > 1.5) lockedWeight = smoothWeight;
        } else {
          if (diff > 1.0) lockedWeight = smoothWeight;
        }

        static unsigned long stableTimer = 0;
        if (abs(smoothWeight) < 2.0) {
          if (stableTimer == 0) stableTimer = now;
          if (now - stableTimer > 3000) {
            scale.tare();
            lockedWeight = 0.0;
            stableTimer = 0;
          }
        } else {
          stableTimer = 0;
        }

        // 5. 최종 표시
        float displayOut = (abs(lockedWeight) < 0.5) ? 0.0 : lockedWeight;
        updateNormalDisplay(displayOut);
        break;
      }

      case MODE_EXTRACT: {
        unsigned long elapsedMillis = now - extractStartTime;
        unsigned long elapsedSec = elapsedMillis / 1000;
        
        float netWeight = currentWeight - extractStartWeight;
        
        // ====================================================
        // [수정 포인트 1] 저역통과필터 (LPF) 강화
        // ====================================================
        static float smoothNetWeight = 0.0;
        // 기존 0.3 -> 0.1로 변경하여 가속에 의한 순간적인 무게 상승을 강력하게 억제
        // 반응은 느려지지만, 더 정확한 "누적 무게" 추세에 집중합니다.
        smoothNetWeight = (netWeight * 0.1) + (smoothNetWeight * 0.9);

        // ====================================================
        // [수정 포인트 2] 유량(flow) 분석 및 추출 종료 판단
        // ====================================================
        static float maxSmoothWeight = 0.0; 
        static unsigned long stopTimer = 0; 

        // 현재 저역통과필터된 무게의 최대값 기록
        if (smoothNetWeight > maxSmoothWeight) {
          maxSmoothWeight = smoothNetWeight;
        }

        // 유량이 0.2g/s 이하로 2초간 유지되면 추출이 완전히 끝났다고 판단
        if (flowRate < 0.2) {
          if (stopTimer == 0) stopTimer = now; 
          
          if (now - stopTimer > NO_FLOW_TIMEOUT) { 
            currentMode = MODE_NORMAL;
            // 가속 영향이 완전히 사라진 후 측정된 현재 netWeight를 최종값으로 사용
            float finalWeight = (netWeight < 0.5) ? 0.0 : netWeight;
            updateFinalDisplay(finalWeight, elapsedSec); // 최종 결과 화면 표시 (추가 필요)
            Serial.print("Extraction Finished. Actual Net Weight: ");
            Serial.println(finalWeight);
            break; 
          }
        } else {
          stopTimer = 0; // 유량이 다시 증가하면 타이머 초기화
        }

        // ====================================================
        // [수정 포인트 3] 화면 갱신 (가속 영향을 줄인 부드러운 값 표시)
        // ====================================================
        // 70초 타임아웃 종료
        if (elapsedSec >= EXTRACT_TIMEOUT) {
          currentMode = MODE_NORMAL;
          sendDataViaBLE(smoothNetWeight, 0, elapsedSec);
          break; 
        }

        // 앱 데이터 전송 및 OLED 화면 갱신 (500ms 주기)
        static unsigned long lastSendTime = 0;
        if (now - lastSendTime >= 500) {
          // 가속 영향을 강력하게 억제한 smoothNetWeight를 앱으로 전송
          sendDataViaBLE(smoothNetWeight, flowRate, elapsedSec);
          // 화면에는 여전히 현재의 대략적인 무게(약간 부드럽게)를 표시
          updateExtractDisplay(netWeight); 
          
          lastSendTime = now;
        }
        break;
      } // case MODE_EXTRACT 종료
      
    } 

    lastWeightForFlow = currentWeight;
    lastWeight = currentWeight;
    lastTime = now;
  }
  
  delay(10);
}

void setupHardware() {
  pinMode(TOUCH_BUTTON_PIN, INPUT_PULLDOWN);
  Serial.println("Button pin configured (PULLDOWN, HIGH = pressed)");
}

void setupDisplay() {
  delay(1000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  Serial.println("OLED initialized successfully");
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(5, 25);
  display.println("Esso Scale");
  display.display();
  delay(1500);
}

void updateNormalDisplay(float weight) {
  static float lastDisplayedWeight = -999.9;
  if (abs(weight - lastDisplayedWeight) < 0.1) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("[ Normal ]");
  display.setTextSize(2);
  display.setCursor(32, 28);
  display.print(weight, 1);
  display.println("g");
  display.display();

  lastDisplayedWeight = weight;
}

void updateExtractDisplay(float netWeight) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("[ EXTRACTION ]");
  display.setTextSize(2);
  display.setCursor(32, 28);
  display.print(netWeight, 1);
  display.println("g");
  display.display();
}

void updateFinalDisplay(float netWeight, unsigned long elapsed) {
  currentMode = MODE_NORMAL;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("[ FINISHED ]");
  display.setTextSize(2);
  display.setCursor(32, 28);
  display.print(netWeight, 1);
  display.println("g");
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("Time: ");
  display.print(elapsed);
  display.print("s");
  display.display();
  delay(5000);
}

void setupBLE() {
  BLEDevice::init("Espresso Scale");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_READ
                    );
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
  Serial.println("BLE initialized successfully");
}

void sendDataViaBLE(float weight, float flowRate, unsigned long elapsed) {
  if (pCharacteristic == nullptr) return;
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%.1f,%.2f,%lu", weight, flowRate, elapsed);
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}

void readButton() {
  bool reading = digitalRead(TOUCH_BUTTON_PIN) == HIGH;
  if (reading && !buttonPressed) {
    buttonPressed = true;
    buttonPressStart = millis();
    buttonHandled = false;
  } 
  else if (!reading && buttonPressed) {
    if (!buttonHandled) {
      unsigned long holdTime = millis() - buttonPressStart;
      processButtonAction(holdTime);
    }
    buttonPressed = false;
  }
  else if (reading && buttonPressed && !buttonHandled) {
    unsigned long holdTime = millis() - buttonPressStart;
    if (holdTime >= EXTRACT_HOLD_TIME && currentMode == MODE_NORMAL && !buttonHandled) {
      buttonHandled = true;
      performTare();
    }
  }
}

void processButtonAction(unsigned long holdTime) {
  if (holdTime >= TARE_HOLD_TIME && holdTime < EXTRACT_HOLD_TIME) {
    if (currentMode == MODE_NORMAL) {
      performTare();              
      extractStartWeight = 0;      
      extractStartTime = millis();
      lastWeightChangeTime = millis();
      lastFilteredWeight = 0;
      firstExtractSample = true;
      currentMode = MODE_EXTRACT;
      sendDataViaBLE(0, 0, 0);
      Serial.println("Extraction started by mode toggle");
      updateExtractDisplay(0);
    } 
    else if (currentMode == MODE_EXTRACT) {
      currentMode = MODE_NORMAL;
      unsigned long elapsed = (millis() - extractStartTime) / 1000;
      float netWeight = readWeight() - extractStartWeight;
      Serial.print("Extraction stopped by mode toggle. Final: ");
      Serial.println(netWeight);
      sendDataViaBLE(netWeight, 0, elapsed);
      updateNormalDisplay(readWeight());
    }
    buttonHandled = true;
  }
}

float readWeight() {
  if (!scale.is_ready()) return lastWeight;
  
  float raw = scale.get_units(1);
  if (raw < 0) raw = 0;
  
  return raw;
}

float calculateFlowRate(float currentWeight, float lastWeight, unsigned long deltaTime) {
  if (deltaTime <= 0) return lastFlowRate; 
  float deltaWeight = currentWeight - lastWeight;
  if (deltaWeight < 0) deltaWeight = 0;

  float rawFlowRate = (deltaWeight / (float)deltaTime) * 1000.0;
  float filteredFlowRate = (rawFlowRate * 0.3) + (lastFlowRate * 0.7);
  
  lastFlowRate = filteredFlowRate;
  return filteredFlowRate;
}

void performTare() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(35, 30);
  display.println("TARING..."); 
  display.display();
  delay(500); 
  scale.tare();
  Serial.println("Tare done");
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(6, 25);
  display.println("Tare OK");
  display.display();
  delay(500);
}