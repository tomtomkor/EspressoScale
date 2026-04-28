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

#define TARE_HOLD_TIME     500
#define EXTRACT_HOLD_TIME  2000
#define CALIB_HOLD_TIME    5000

#define NO_FLOW_TIMEOUT    30000
#define EXTRACT_TIMEOUT    70000

#define MIN_WEIGHT_FOR_STABLE  2.0
#define STABLE_DURATION_MS     2000
#define SPIKE_LIMIT        1.0

#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ==================== Objects ====================
HX711 scale;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BLECharacteristic *pCharacteristic;

// ==================== Variables ====================
enum DeviceMode { MODE_NORMAL, MODE_EXTRACT, MODE_CALIB };
DeviceMode currentMode = MODE_NORMAL;

float extractStartWeight = 0;
float lastWeight = 0;
unsigned long lastWeightChangeTime = 0;
unsigned long extractStartTime = 0;
float lastFilteredWeight = 0;
bool firstExtractSample = true;

float calibrationFactor = 1.0;

unsigned long buttonPressStart = 0;
bool buttonPressed = false;
bool buttonHandled = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { Serial.println(">>> App Connected!"); }
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
void enterCalibrationMode();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-C3 Espresso Scale Starting...");
  
  // 1. OLED Initialization
  setupDisplay();
  
  // 2. Touch sensor Initialization 
  setupHardware();
  
  // 3. BLE Initialization
  setupBLE();
  
  // 4. HX711 Initialization
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibrationFactor);
  Serial.println("HX711 initialized successfully");
  
  // 5. Automatic taring
  performTare();
  Serial.println("Auto tare done at startup");
  
  Serial.println("Ready!");
  updateNormalDisplay(readWeight());
}

void loop() {
  readButton();
  
  float currentWeight = readWeight();
  static unsigned long lastTime = millis();
  static float lastWeightForFlow = currentWeight;
  
  unsigned long now = millis();
  unsigned long deltaTime = now - lastTime;
  
  if (deltaTime >= 100) {
    float flowRate = calculateFlowRate(currentWeight, lastWeightForFlow, deltaTime);
    
    switch (currentMode) {
      case MODE_NORMAL:
        updateNormalDisplay(currentWeight);
        break;
        
      case MODE_EXTRACT: {
        unsigned long elapsed = (now - extractStartTime) / 1000;
        float netWeight = currentWeight - extractStartWeight;
        
        float filteredWeight = 0.8 * currentWeight + 0.2 * lastWeight;
        if (firstExtractSample) {
          lastFilteredWeight = filteredWeight;
          firstExtractSample = false;
        }
        if (abs(filteredWeight - lastFilteredWeight) > 0.15) {
          lastWeightChangeTime = now;
        }
        lastFilteredWeight = filteredWeight;
        
        bool noFlow = (now - lastWeightChangeTime) > NO_FLOW_TIMEOUT;
        bool timeout = elapsed > (EXTRACT_TIMEOUT / 1000);
        
        if (noFlow || timeout) {
          currentMode = MODE_NORMAL;
          Serial.print("Extraction ended. Final: ");
          Serial.println(netWeight);
          sendDataViaBLE(netWeight, 0, elapsed);
          updateNormalDisplay(readWeight());
        } else {
          static unsigned long lastDisplayUpdate = 0;
          static float lastDisplayWeight = 0;
          static unsigned long stableStart = 0;
          static bool stable = false;
          
          if (now - lastDisplayUpdate >= 1000) {
            float delta = netWeight - lastDisplayWeight;
            if (netWeight < MIN_WEIGHT_FOR_STABLE) {
              stable = false;
            } else {
              if (abs(delta) < 0.2) {
                if (!stable) {
                  stable = true;
                  stableStart = now;
                }
              } else {
                stable = false;
              }
            }
            if (stable && (now - stableStart >= STABLE_DURATION_MS)) {
              updateFinalDisplay(netWeight, elapsed);
            } else {
              updateExtractDisplay(netWeight);
            }
            lastDisplayWeight = netWeight;
            lastDisplayUpdate = now;
          }
          
          static unsigned long lastSendTime = 0;
          if (now - lastSendTime >= 500) {
            sendDataViaBLE(netWeight, flowRate, elapsed);
            lastSendTime = now;
          }
        }
        break;
      }
      
      case MODE_CALIB:
        break;
    }
    
    lastWeightForFlow = currentWeight;
    lastTime = now;
  }
  
  lastWeight = currentWeight;
  delay(10);
}

void setupHardware() {
  pinMode(TOUCH_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Button pin configured");
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
  display.setCursor(0, 0);
  display.println("Espresso Scale");
  display.println("v1.0");
  display.display();
  delay(1500);
}

void updateNormalDisplay(float weight) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("[ Normal ]");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(weight, 1);
  display.println(" g");
  display.display();
}

void updateExtractDisplay(float netWeight) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("[ Extraction ]");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(netWeight, 1);
  display.println(" g");
  display.display();
}

void updateFinalDisplay(float netWeight, unsigned long elapsed) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("[ Final Result ]");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(netWeight, 1);
  display.println(" g");
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("Time:");
  display.print(elapsed);
  display.print("s");
  display.display();
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
        performTare();  // tare before extraction
        extractStartWeight = 0;
        extractStartTime = millis();
        lastWeightChangeTime = millis();
        lastFilteredWeight = 0;
        firstExtractSample = true;
        currentMode = MODE_EXTRACT;
        sendDataViaBLE(0, 0, 0);
        Serial.println("Extraction start with auto-tare");
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
    else if (currentMode == MODE_EXTRACT) {
      // manual stop (just in case)
      currentMode = MODE_NORMAL;
      unsigned long elapsed = (millis() - extractStartTime) / 1000;
      float netWeight = readWeight() - extractStartWeight;
      Serial.print("Extraction manually stopped. Final: ");
      Serial.println(netWeight);
      sendDataViaBLE(netWeight, 0, elapsed);
      updateNormalDisplay(readWeight());
      buttonHandled = true;
    }
  }
}

float readWeight() {
  if (!scale.is_ready()) return lastWeight;
  
  float raw = scale.get_units(5);
  if (raw < 0) raw = 0;
  
  if (abs(raw - lastWeight) > SPIKE_LIMIT) {
    Serial.println("Spike ignored");
    return lastWeight;
  }
  
  return raw;
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
  display.setTextSize(1);
  display.setCursor(20, 28);
  display.println("Tare OK");
  display.display();
  delay(500);
  updateNormalDisplay(readWeight());
}

void enterCalibrationMode() {
  DeviceMode previousMode = currentMode;
  currentMode = MODE_CALIB;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("CALIBRATION");
  display.setCursor(0, 30);
  display.println("Place 100g weight");
  display.display();
  
  bool calibrated = false;
  unsigned long startTime = millis();
  while (!calibrated && (millis() - startTime < 30000)) {
    if (scale.is_ready()) {
      float raw = scale.get_value(5);
      if (raw > 80 && raw < 120) {
        calibrationFactor = 100.0 / raw;
        scale.set_scale(calibrationFactor);
        calibrated = true;
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("Calibration OK!");
        display.setCursor(0, 30);
        display.print("Factor: ");
        display.println(calibrationFactor, 4);
        display.display();
        delay(2000);
        break;
      }
    }
    delay(200);
  }
  
  if (!calibrated) {
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("Calibration failed");
    display.setCursor(0, 40);
    display.println("Check weight");
    display.display();
    delay(2000);
  }
  
  currentMode = previousMode;
  updateNormalDisplay(readWeight());
}