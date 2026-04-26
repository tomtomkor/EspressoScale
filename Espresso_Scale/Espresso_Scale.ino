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
#define OLED_SDA           6
#define OLED_SCL           7

// ==================== Constants ====================
#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      32
#define OLED_ADDR          0x3C

// button timing (ms)
#define TARE_HOLD_TIME     500
#define EXTRACT_HOLD_TIME  2000
#define CALIB_HOLD_TIME    5000

// extraction termination
#define NO_FLOW_TIMEOUT    30000   // 30 sec
#define EXTRACT_TIMEOUT    70000   // 70 sec

// display stability thresholds (easily adjustable)
#define MIN_WEIGHT_FOR_STABLE  2.0    // grams (pre-infusion/blooming phase)
#define STABLE_DURATION_MS     2000   // milliseconds (2 sec)

// BLE
#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ==================== Objects ====================
HX711 scale;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BLECharacteristic *pCharacteristic;

// ==================== Variables ====================
enum DeviceMode { MODE_NORMAL, MODE_EXTRACT, MODE_CALIB };
DeviceMode currentMode = MODE_NORMAL;

// extraction
float extractStartWeight = 0;
float lastWeight = 0;
unsigned long lastWeightChangeTime = 0;
unsigned long extractStartTime = 0;
float lastFilteredWeight = 0;
bool firstExtractSample = true;

// calibration
float calibrationFactor = 1.0;

// button
unsigned long buttonPressStart = 0;
bool buttonPressed = false;
bool buttonHandled = false;

// BLE callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { Serial.println(">>> App Connected!"); }
    void onDisconnect(BLEServer* pServer) {
      Serial.println(">>> App Disconnected. Restarting Advertising...");
      pServer->getAdvertising()->start();
    }
};

// function prototypes
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

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C3 Espresso Scale Starting...");
  
  setupHardware();
  setupDisplay();
  setupBLE();
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibrationFactor);
  
  Serial.println("Ready!");
  display.println("Ready!");
  display.display();
  delay(1000);
  updateNormalDisplay(readWeight());
}

// ==================== Loop ====================
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
        
        // flow detection filter
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
        } else {
          // ----- Display update every 1 second -----
          static unsigned long lastDisplayUpdate = 0;
          static float lastDisplayWeight = 0;
          static unsigned long stableStart = 0;
          static bool stable = false;
          
          if (now - lastDisplayUpdate >= 1000) {
            float delta = netWeight - lastDisplayWeight;
            
            // Use configurable MIN_WEIGHT_FOR_STABLE
            if (netWeight < MIN_WEIGHT_FOR_STABLE) {
              stable = false;   // pre-infusion/blooming: never mark as stable
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
            
            // Show final result after STABLE_DURATION_MS of stability
            if (stable && (now - stableStart >= STABLE_DURATION_MS)) {
              updateFinalDisplay(netWeight, elapsed);
            } else {
              updateExtractDisplay(netWeight);
            }
            
            lastDisplayWeight = netWeight;
            lastDisplayUpdate = now;
          }
          
          // BLE send every 0.5s
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

// ==================== Hardware & Display ====================
void setupHardware() {
  pinMode(TOUCH_BUTTON_PIN, INPUT_PULLUP);
}

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

void updateNormalDisplay(float weight) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(weight, 1);
  display.println(" g");
  display.display();
}

void updateExtractDisplay(float netWeight) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(netWeight, 1);
  display.println(" g");
  display.display();
}

void updateFinalDisplay(float netWeight, unsigned long elapsed) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(netWeight, 1);
  display.println(" g");
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print("Time: ");
  display.print(elapsed);
  display.println(" s");
  display.display();
}

// ==================== BLE ====================
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
  Serial.println("BLE ready");
}

void sendDataViaBLE(float weight, float flowRate, unsigned long elapsed) {
  if (pCharacteristic == nullptr) return;
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%.1f,%.2f,%lu", weight, flowRate, elapsed);
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}

// ==================== Button Handling ====================
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
        lastFilteredWeight = extractStartWeight;
        firstExtractSample = true;
        sendDataViaBLE(extractStartWeight, 0, 0);
        Serial.println("Extraction start (manual)");
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

// ==================== Measurement ====================
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

// ==================== Calibration ====================
void enterCalibrationMode() {
  DeviceMode previousMode = currentMode;
  currentMode = MODE_CALIB;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("CALIBRATION");
  display.setCursor(0, 16);
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
        display.setCursor(0, 0);
        display.println("Calibration OK!");
        display.setCursor(0, 20);
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
    display.setCursor(0, 0);
    display.println("Calibration failed");
    display.setCursor(0, 16);
    display.println("Check weight");
    display.display();
    delay(2000);
  }
  
  currentMode = previousMode;
  updateNormalDisplay(readWeight());
}