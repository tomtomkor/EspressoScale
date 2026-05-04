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

#define TARE_HOLD_TIME     100      // 0.5 s
#define EXTRACT_HOLD_TIME  2000     // 2 s

#define NO_FLOW_TIMEOUT    10000    
#define EXTRACT_TIMEOUT    63000   

#define MIN_WEIGHT_FOR_STABLE  5.0
#define STABLE_DURATION_MS     2000

#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Fixed calibration factor (from separate test)
#define CALIBRATION_FACTOR 1729.06f

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
  
  float currentWeight = readWeight();
  static unsigned long lastTime = millis();
  static float lastWeightForFlow = currentWeight;
  
  unsigned long now = millis();
  unsigned long deltaTime = now - lastTime;
  
  if (deltaTime >= 100) {
    float flowRate = calculateFlowRate(currentWeight, lastWeightForFlow, deltaTime);
    
    switch (currentMode) {
      
      case MODE_NORMAL: {
        static float displayedWeight = 0.0; 

        static unsigned long lastAutoTare = 0;
        if (now - lastAutoTare > 10000) {
          if (abs(currentWeight) < 1.0) { 
              scale.tare(); 
              currentWeight = 0.0; 
          }
          lastAutoTare = millis();
        }

        float diff = abs(currentWeight - displayedWeight);
        if (diff >= 0.3) {
           displayedWeight = currentWeight;
        } 
   
        if (abs(displayedWeight) < 0.3) {
           displayedWeight = 0.0;
        }

        updateNormalDisplay(displayedWeight);
        break;
    }
        
      case MODE_EXTRACT: {
        unsigned long elapsed = (now - extractStartTime) / 1000;
        float netWeight = currentWeight - extractStartWeight;
        
        float filteredWeight = 0.7 * currentWeight + 0.3 * lastWeight;
        if (firstExtractSample) {
          lastFilteredWeight = filteredWeight;
          firstExtractSample = false;
        }
        if (abs(filteredWeight - lastFilteredWeight) > 0.15) {
          lastWeightChangeTime = now;
        }
        lastFilteredWeight = filteredWeight;
        
        // check noFlow only if netWeight >= MIN_WEIGHT_FOR_STABLE
        bool noFlow = false;
        if (netWeight >= MIN_WEIGHT_FOR_STABLE) {
          noFlow = (now - lastWeightChangeTime) > NO_FLOW_TIMEOUT;
        }
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
    }
    
    lastWeightForFlow = currentWeight;
    lastTime = now;
  }
  
  lastWeight = currentWeight;
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
    if (currentMode == MODE_NORMAL) updateNormalDisplay(readWeight());
    else if (currentMode == MODE_EXTRACT) updateExtractDisplay(readWeight() - extractStartWeight);
  }
  else if (reading && buttonPressed && !buttonHandled) {
    unsigned long holdTime = millis() - buttonPressStart;
    // long touch (loner than 2 seconds) -> tare (only in normal mode)
    if (holdTime >= EXTRACT_HOLD_TIME && currentMode == MODE_NORMAL && !buttonHandled) {
      buttonHandled = true;
      performTare();
    }
  }
}

void processButtonAction(unsigned long holdTime) {
  // short touch (0.5~2 seconds) -> mode toggle (Normal <-> Extract)
  if (holdTime >= TARE_HOLD_TIME && holdTime < EXTRACT_HOLD_TIME) {
    if (currentMode == MODE_NORMAL) {
      // Normal -> Extract: auto tare
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
      // Extract -> Normal: end of extraction and send last data
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
  
  float raw = scale.get_units(5);
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