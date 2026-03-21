// ============================================================
// DiploidHydro — Brain Sensor Read
// HF-006: All sensors to Serial Monitor
// ESP32 WROOM-32
// ============================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

//Pin Def
#define ONE_WIRE_BUS 4     // DS18B20 data pin

#define LEVEL_1 16
#define LEVEL_2 17
#define LEVEL_3 25
#define LEVEL_4 26
#define LEVEL_5 27

//pH Constants 
const float CONST_DEFAULT_PH_V7 = 1.526;    // voltage at pH 7.0
const float CONST_DEFAULT_PH_V4 = 2.042;    // voltage at pH 4.0
const float CONST_DEFAULT_PH_SLOPE = 3.0 / (CONST_DEFAULT_PH_V4 - CONST_DEFAULT_PH_V7);
bool plotMode = false;

//Runtime Calibration Values
float calV7 = CONST_DEFAULT_PH_V7;   // loaded from storage on boot
float calV4 = CONST_DEFAULT_PH_V4;   // loaded from storage on boot
float calSlope = CONST_DEFAULT_PH_SLOPE;  // recalculated after load


// TDS calibration
const float TDS_CAL_V0 = 0.006;   // voltage at 0 ppm (distilled water)
const float TDS_CAL_TEMP = 23.37; // temperature at calibration

//Sensors 
Adafruit_ADS1115 ads;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire); 
Preferences preferences;

float voltageToPH(float voltage) {
  return 4.0 + (CONST_DEFAULT_PH_V4 - voltage) * (3.0 / (CONST_DEFAULT_PH_V4 - CONST_DEFAULT_PH_V7));
}

float voltageToPPM(float voltage, float tempC) {
  // Temperature compensation factor
  float compensationCoeff = 1.0 + (0.02 * (tempC - 25.0));
  float compensatedVoltage = voltage / compensationCoeff;
  
// DFRobot Gravity TDS sensor formula
// Source: https://wiki.dfrobot.com/Gravity__Analog_TDS_Sensor___Meter_For_Arduino_SKU__SEN0244
  float ppm = (133.42 * pow(compensatedVoltage, 3) 
            - 255.86 * pow(compensatedVoltage, 2) 
            + 857.39 * compensatedVoltage) * 0.5;
  
  return ppm;
}

//allows for calibration updates
void saveCalibration(float v7, float v4) {
  preferences.begin("diploidhydro", false);
  preferences.putFloat("phCalV7", v7);
  preferences.putFloat("phCalV4", v4);
  preferences.end();
  
  calV7 = v7;
  calV4 = v4;
  calSlope = (7.0 - 4.0) / (calV7 - calV4);
  
  Serial.println("Calibration saved.");
  Serial.print("New V7: "); Serial.println(calV7, 3);
  Serial.print("New V4: "); Serial.println(calV4, 3);
  Serial.print("New slope: "); Serial.println(calSlope, 3);
}

//listens for calibration command
void checkSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "cal7") {
      int16_t raw = ads.readADC_SingleEnded(0);
      float voltage = ads.computeVolts(raw);
      saveCalibration(voltage, calV4);
      Serial.print("Calibrated pH 7.0 at: "); Serial.println(voltage, 3);
    }
    else if (cmd == "cal4") {
      int16_t raw = ads.readADC_SingleEnded(0);
      float voltage = ads.computeVolts(raw);
      saveCalibration(calV7, voltage);
      Serial.print("Calibrated pH 4.0 at: "); Serial.println(voltage, 3);
    }
    else if (cmd == "resetcal") {
      preferences.begin("diploidhydro", false);
      preferences.remove("phCalV7");
      preferences.remove("phCalV4");
      preferences.end();
  
      calV7 = CONST_DEFAULT_PH_V7;
      calV4 = CONST_DEFAULT_PH_V4;
      calSlope = CONST_DEFAULT_PH_SLOPE;
      
      Serial.println("Calibration reset to defaults.");
      Serial.print("V7: "); Serial.println(calV7, 3);
      Serial.print("V4: "); Serial.println(calV4, 3);
      Serial.print("Slope: "); Serial.println(calSlope, 3);
    }
    else if (cmd == "ploton") {
      plotMode = true;
      Serial.println("Plot mode ON");
    }
    else if (cmd == "plotoff") {
      plotMode = false;
      Serial.println("Plot mode OFF");
    }
  }
}

//init sensors and GPIO 
void setup() {
  Serial.begin(9600);
  delay(1000);
  
// Load calibration from storage
  preferences.begin("diploidhydro", false);
  calV7 = preferences.getFloat("phCalV7", CONST_DEFAULT_PH_V7);
  calV4 = preferences.getFloat("phCalV4", CONST_DEFAULT_PH_V4);
  preferences.end();

  // Recalculate slope from loaded values
  calSlope = (7.0 - 4.0) / (calV7 - calV4);

  Serial.print("pH cal V7: "); Serial.println(calV7, 3);
  Serial.print("pH cal V4: "); Serial.println(calV4, 3);
  Serial.print("pH slope:  "); Serial.println(calSlope, 3);

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


void loop() {
  //check for commands
  checkSerialCommands();

  //pH Reading 
  int16_t phRaw = ads.readADC_SingleEnded(0);
  float phVoltage = ads.computeVolts(phRaw);

  //TDS Reading 
  int16_t tdsRaw = ads.readADC_SingleEnded(1);
  float tdsVoltage = ads.computeVolts(tdsRaw);

  //Temperature Reading 
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  float tempF = (tempC * 9.0 / 5.0) + 32.0;

  //Level Sensor Readings 
  bool level1 = digitalRead(LEVEL_1);
  bool level2 = digitalRead(LEVEL_2);
  bool level3 = digitalRead(LEVEL_3);
  bool level4 = digitalRead(LEVEL_4);
  bool level5 = digitalRead(LEVEL_5);

  //Print to Serial
  float pH = voltageToPH(phVoltage);
  float ppm = voltageToPPM(tdsVoltage, tempC);
  float ec = ppm / 500.0;

  if (!plotMode) {
    Serial.println("────────────────────────────────────────");
    Serial.print("pH:            "); Serial.print(pH, 2);
    Serial.print("  ("); Serial.print(phVoltage, 3); Serial.println("V)");
    Serial.print("TDS Voltage:   "); Serial.print(tdsVoltage, 3); Serial.println("V");
    Serial.print("PPM:           "); Serial.print(ppm, 0); Serial.println(" ppm");
    Serial.print("EC:            "); Serial.print(ec, 2); Serial.println(" mS/cm");
    Serial.print("Temperature:   "); Serial.print(tempC, 2); Serial.println("°C");
    Serial.print("Temperature:   "); Serial.print(tempF, 2); Serial.println("°F");
    Serial.println("── Level Sensors ────────────────────────");
    Serial.print("L1: "); Serial.print(level1 ? "DRY" : "WET");
    Serial.print("  L2: "); Serial.print(level2 ? "DRY" : "WET");
    Serial.print("  L3: "); Serial.print(level3 ? "DRY" : "WET");
    Serial.print("  L4: "); Serial.print(level4 ? "DRY" : "WET");
    Serial.print("  L5: "); Serial.println(level5 ? "DRY" : "WET");
  }

  if (plotMode) {
    Serial.print("pH:"); Serial.print(pH, 2);
    Serial.print(",TDS:"); Serial.print(tdsVoltage, 3);
    Serial.print(",Temp:"); Serial.print(tempC, 2);
    Serial.print(",L1:"); Serial.print(!level1 ? 1 : 0);
    Serial.print(",L2:"); Serial.print(!level2 ? 1 : 0);
    Serial.print(",L3:"); Serial.print(!level3 ? 1 : 0);
    Serial.print(",L4:"); Serial.print(!level4 ? 1 : 0);
    Serial.print(",L5:"); Serial.println(!level5 ? 1 : 0);
  }
  delay(2000);
}