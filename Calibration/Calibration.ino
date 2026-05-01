#include <HX711.h>

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(); 
  scale.tare();
  Serial.println("Tare done. Place 100g, then send any character from Serial Monitor.");
}

void loop() {
  if (Serial.available()) {
    Serial.read();
    float raw = scale.get_value(10);
    Serial.print("Raw change: ");
    Serial.println(raw);
    float factor = raw / 100;      // change 100 to your known weight 
    Serial.print("Calibration factor: ");
    Serial.println(factor, 6);
    Serial.println("Use this factor in main code (e.g., scale.set_scale(factor));");
    while(1); // 정지
  }
  delay(100);
}