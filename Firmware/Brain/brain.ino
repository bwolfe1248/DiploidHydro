// ============================================================
// DiploidHydro — Brain Sensor Read
// HF-006: All sensors to Serial Monitor
// ESP32 WROOM-32
// ============================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//Pin Def
#define ONE_WIRE_BUS 4     // DS18B20 data pin

#define LEVEL_1 16
#define LEVEL_2 17
#define LEVEL_3 25
#define LEVEL_4 26
#define LEVEL_5 27

//Sensors 
Adafruit_ADS1115 ads;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

//init sensors and GPIO 
void setup() {
  Serial.begin(9600);
  delay(1000);

  //Initialize I2C
  Wire.begin(21, 22);

  //Initialize ADS1115
  if (!ads.begin()) {
    Serial.println("ERROR: ADS1115 not found. Check wiring.");
    while (1);
  }
  Serial.println("ADS1115 initialized.");

  //Initialize temperature sensor
  tempSensor.begin();
  Serial.println("DS18B20 initialized.");

  //Initialize level sensor pins as inputs
  pinMode(LEVEL_1, INPUT);
  pinMode(LEVEL_2, INPUT);
  pinMode(LEVEL_3, INPUT);
  pinMode(LEVEL_4, INPUT);
  pinMode(LEVEL_5, INPUT);

  Serial.println("All sensors initialized. Starting readings...");
  Serial.println("────────────────────────────────────────");
}

//Loop 
void loop() {

  //pH Reading 
  int16_t phRaw = ads.readADC_SingleEnded(0);
  float phVoltage = ads.computeVolts(phRaw);

  //TDS Reading 
  int16_t tdsRaw = ads.readADC_SingleEnded(1);
  float tdsVoltage = ads.computeVolts(tdsRaw);

  //Temperature Reading 
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  //Level Sensor Readings 
  bool level1 = digitalRead(LEVEL_1);
  bool level2 = digitalRead(LEVEL_2);
  bool level3 = digitalRead(LEVEL_3);
  bool level4 = digitalRead(LEVEL_4);
  bool level5 = digitalRead(LEVEL_5);

  //Print to Serial 
  Serial.println("────────────────────────────────────────");
  Serial.print("pH Voltage:    "); Serial.print(phVoltage, 3); Serial.println("V");
  Serial.print("TDS Voltage:   "); Serial.print(tdsVoltage, 3); Serial.println("V");
  Serial.print("Temperature:   "); Serial.print(tempC, 2); Serial.println("°C");
  Serial.println("── Level Sensors ────────────────────────");
  Serial.print("L1: "); Serial.print(level1 ? "DRY" : "WET");
  Serial.print("  L2: "); Serial.print(level2 ? "DRY" : "WET");
  Serial.print("  L3: "); Serial.print(level3 ? "DRY" : "WET");
  Serial.print("  L4: "); Serial.print(level4 ? "DRY" : "WET");
  Serial.print("  L5: "); Serial.println(level5 ? "DRY" : "WET");

  delay(2000);
}