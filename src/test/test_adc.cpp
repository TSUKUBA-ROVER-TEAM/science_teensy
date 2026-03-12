#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// --- 設定値 ---
const float R_REF = 1000.0;
const float R0 = 100.0;
const float ALPHA = 0.003851; 
void setup() {

  Serial.begin(115200);
  while (!Serial && millis() < 3000);


  Wire.begin();

  if (!ads.begin()) {
    Serial.println("ADS1115が見つかりません。配線（Pin 18/19）を確認してください。");
    while (1);
  }


  ads.setGain(GAIN_FOUR); 


  ads.setDataRate(RATE_ADS1115_16SPS);

  Serial.println("Teensy 4.1 Pt100 Measurement Start");
}

void loop() {
  // 1. A0の電圧を読み取る
  int16_t raw_a0 = ads.readADC_SingleEnded(0);
  float volt_a0 = ads.computeVolts(raw_a0);

  float v_supply = 3.30; 
  float current = (v_supply - volt_a0) / R_REF;

  float r_pt100 = volt_a0 / current;

  float temperature = (r_pt100 / R0 - 1.0) / ALPHA;

  Serial.print("Voltage: "); Serial.print(volt_a0, 4); Serial.print(" V, ");
  Serial.print("Resistance: "); Serial.print(r_pt100, 2); Serial.print(" Ohm, ");
  Serial.print("Temp: "); Serial.print(temperature, 2); Serial.println(" C");

  delay(1000);
}