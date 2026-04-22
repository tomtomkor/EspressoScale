#include <Arduino.h>
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
//#include <BLEUtils.h>
#include <BLEServer.h>

// ==================== 핀 정의 ====================
#define LOADCELL_DOUT_PIN  2
#define LOADCELL_SCK_PIN   3
#define TOUCH_BUTTON_PIN   5
#define OLED_SDA           6
#define OLED_SCL           7

// ==================== 상수 정의 ====================
#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64
#define OLED_ADDR          0x3C

// 버튼 시간 (밀리초)
#define TARE_HOLD_TIME     500
#define EXTRACT_HOLD_TIME  2000
#define CALIB_HOLD_TIME    5000

// 추출 자동 종료 조건
#define NO_FLOW_TIMEOUT    30000  // 20초 동안 무게 변화 없음
#define EXTRACT_TIMEOUT    80000  // 최대 추출 시간 60초

// BLE 서비스 및 특성 UUID
#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ==================== 객체 선언 ====================
HX711 scale;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BLECharacteristic *pCharacteristic;

// ==================== 전역 변수 ====================
// 상태 변수
enum DeviceMode {
  MODE_NORMAL,      // 일반 모드
  MODE_EXTRACT,     // 추출 모드
  MODE_CALIB        // 캘리브레이션 모드
};

DeviceMode currentMode = MODE_NORMAL;

// 추출 관련 변수
float extractStartWeight = 0;
float lastWeight = 0;
unsigned long lastWeightChangeTime = 0;
unsigned long extractStartTime = 0;

// Scale 변수
float calibrationFactor = 1.0;

// 버튼 관련 변수
unsigned long buttonPressStart = 0;
bool buttonPressed = false;
bool buttonHandled = false;

// ==================== 함수 선언 ====================
void setupHardware();
void setupDisplay();
void setupBLE();
void readButton();
void processButtonAction(unsigned long holdTime);
void updateNormalDisplay(float weight);
void updateExtractDisplay(float weight, float flowRate, unsigned long elapsed);
void sendDataViaBLE(float weight, float flowRate, unsigned long elapsed);
float readWeight();
float calculateFlowRate(float currentWeight, float lastWeight, unsigned long deltaTime);
void performTare();
void enterCalibrationMode();

// ==================== 메인 코드 ====================
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C3 Espresso Scale Starting...");
  
  setupHardware();
  setupDisplay();
  setupBLE();
  
  // HX711 초기화
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibrationFactor);
  
  Serial.println("Ready!");
  display.println("Ready!");
  display.display();
  delay(1000);
  updateNormalDisplay(readWeight());
}

void loop() {
  readButton();
  
  float currentWeight = readWeight();
  static unsigned long lastTime = millis();
  static float lastWeightForFlow = currentWeight;
  static float lastFilteredWeight = 0;
  
  unsigned long now = millis();
  unsigned long deltaTime = now - lastTime;
  
  if (deltaTime >= 100) {  // 0.1초마다 측정
    float flowRate = calculateFlowRate(currentWeight, lastWeightForFlow, deltaTime);
    
    switch (currentMode) {
      case MODE_NORMAL:
        if (currentWeight > 0.5) {  // 0.5g 이상 감지
          currentMode = MODE_EXTRACT;
          extractStartWeight = currentWeight;
          extractStartTime = now;
          lastWeightChangeTime = now;
          Serial.println("Extraction started");
        }
        updateNormalDisplay(currentWeight);
        break;
        
      case MODE_EXTRACT:
        {
          unsigned long elapsed = (now - extractStartTime) / 1000;
          float netWeight = currentWeight - extractStartWeight;
          
          // 무게 변화 확인
          float filteredWeight = 0.8 * currentWeight + 0.2 * lastWeight;
          if (abs(filteredWeight - lastFilteredWeight) > 0.15) {
            lastWeightChangeTime = now;
          }
          
          lastFilteredWeight = filteredWeight; 
          
          // 자동 종료 조건
          bool noFlow = (now - lastWeightChangeTime) > NO_FLOW_TIMEOUT;
          bool timeout = elapsed > (EXTRACT_TIMEOUT / 1000);
          
          if (noFlow || timeout) {
            currentMode = MODE_NORMAL;
            Serial.print("Extraction ended. Final: ");
            Serial.println(netWeight);
            sendDataViaBLE(netWeight, 0, elapsed);
          } else {
            updateExtractDisplay(netWeight, flowRate, elapsed);
            sendDataViaBLE(netWeight, flowRate, elapsed);
          }
        }
        break;
        
      case MODE_CALIB:
        // 캘리브레이션 모드 처리
        break;
    }
    
    lastWeightForFlow = currentWeight;
    lastTime = now;
  }
  
  lastWeight = currentWeight;
  delay(10);
}

// ==================== 하드웨어 초기화 ====================
void setupHardware() {
  pinMode(TOUCH_BUTTON_PIN, INPUT_PULLUP);
}

// ==================== 디스플레이 초기화 ====================
void setupDisplay() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Espresso Scale");
  display.println("v1.0");
  display.display();
}

// ==================== BLE 초기화 ====================
void setupBLE() {
  BLEDevice::init("Espresso Scale");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_READ
                    );
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("BLE ready");
}

// ==================== 버튼 처리 ====================
void readButton() {
  bool reading = digitalRead(TOUCH_BUTTON_PIN) == LOW;
  
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
    
    if (!buttonHandled) {
      if (holdTime >= CALIB_HOLD_TIME && currentMode != MODE_CALIB) {
        enterCalibrationMode();
        buttonHandled = true;
      } 
      else if (holdTime >= EXTRACT_HOLD_TIME && currentMode == MODE_NORMAL) {
        currentMode = MODE_EXTRACT;
        extractStartWeight = readWeight();
        extractStartTime = millis();
        lastWeightChangeTime = millis();
        Serial.println("Extraction start");
        buttonHandled = true;
      }
    }
  }
}

void processButtonAction(unsigned long holdTime) {
  if (holdTime >= TARE_HOLD_TIME && holdTime < EXTRACT_HOLD_TIME) {
    if (currentMode == MODE_NORMAL) {
      performTare();
    }
  }
}

// ==================== 무게 관련 함수 ====================
float readWeight() {
  if (scale.is_ready()) {
    float raw = scale.get_units(5);
    return (raw > 0 ? raw : 0);
  }
  return lastWeight;
}

float calculateFlowRate(float currentWeight, float lastWeight, unsigned long deltaTime) {
  float deltaWeight = currentWeight - lastWeight;
  if (deltaWeight < 0) deltaWeight = 0;
  return (deltaWeight / deltaTime) * 1000.0;
}

void performTare() {
  scale.tare();
  Serial.println("Tare done");
  
  display.clearDisplay();
  display.setCursor(20, 25);
  display.println("Tare OK!");
  display.display();
  delay(500);
  updateNormalDisplay(readWeight());
}

// ==================== 캘리브레이션 ====================
void enterCalibrationMode() {
  display.clearDisplay();
  display.println("Put 100g");
  display.display();
  
  // 100g 추 올릴 때까지 대기
  while(true) {
    float raw = scale.get_value(5);
    if (raw > 80 && raw < 120) {  // 100g 근접 감지
      calibrationFactor = 100.0 / raw;
      scale.set_scale(calibrationFactor);
      
      display.clearDisplay();
      display.println("Done!");
      display.display();
      delay(1000);
      break;
    }
    delay(500);
  }
}

// ==================== 디스플레이 업데이트 ====================
void updateNormalDisplay(float weight) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(weight, 1);
  display.println(" g");
  
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("Mode: Standby");
  display.display();
}

void updateExtractDisplay(float weight, float flowRate, unsigned long elapsed) {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.print("W: ");
  display.print(weight, 1);
  display.print("g  ");
  display.print("T: ");
  display.print(elapsed);
  display.println("s");
  
  display.setCursor(0, 20);
  display.print("Flow: ");
  display.print(flowRate, 2);
  display.println("g/s");
  
  /* display.setCursor(0, 45);
  if (flowRate < 1.0) {
    display.print("Pre-infusion");
  } else if (flowRate > 2.5) {
    display.print("Too Fast!");
  } else {
    display.print("Extracting");
  } */
  
  display.display();
}

// ==================== BLE 데이터 전송 ====================
void sendDataViaBLE(float weight, float flowRate, unsigned long elapsed) {
  if (pCharacteristic == nullptr) return;
  
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%.1f,%.2f,%lu", weight, flowRate, elapsed);
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}