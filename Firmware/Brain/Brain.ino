// ============================================================
// DiploidHydro - Brain Firmware
// ESP32 WROOM-32
// ============================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <SPI.h>
#include <Ethernet.h>

//Pin Definitions
#define ONE_WIRE_BUS  4   

//mux select lines and signal
#define MUX_S0  16         // RX2
#define MUX_S1  17         // TX2
#define MUX_S2  25
#define MUX_S3  32
#define MUX_SIG 33         // Analog signal input from mux

//W5500 SPI
#define W5500_CS   5
#define W5500_SCK  18
#define W5500_MISO 19
#define W5500_MOSI 23
#define W5500_RST  26
#define W5500_INT  27

//Network Configuration 
//place holder

//pH Calibration Constants 
// These are the default fallback values if no calibration is stored
const float CONST_DEFAULT_PH_V7    = 1.526;
const float CONST_DEFAULT_PH_V4    = 2.042;
const float CONST_DEFAULT_PH_SLOPE = 3.0 / (CONST_DEFAULT_PH_V4 - CONST_DEFAULT_PH_V7);

//TDS Calibration 
const float TDS_CAL_TEMP = 23.37;   // Temperature at calibration (°C)

//Temperature Sensor Addresses 
// Update later with addresses from temp probes
DeviceAddress TOTE1_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // TODO Update
DeviceAddress TOTE2_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };  // TODO Update
DeviceAddress TOTE3_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };  // TODO Update
DeviceAddress TOTE4_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };  // TODO Update
DeviceAddress TOTE5_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 };  // TODO Update

//Mux Channel Assignments 
// 3 IR level sensors per tote: LOW, MID, HIGH
const int TOTE_LEVEL_CHANNELS[5][3] = {
  { 0,  1,  2},   // Tote 1
  { 3,  4,  5},   // Tote 2
  { 6,  7,  8},   // Tote 3
  { 9, 10, 11},   // Tote 4
  {12, 13, 14},   // Tote 5
};

//Runtime State 
float calV7    = CONST_DEFAULT_PH_V7;
float calV4    = CONST_DEFAULT_PH_V4;
float calSlope = CONST_DEFAULT_PH_SLOPE;
bool  plotMode = false;
bool  ethernetOk = false;

//Sensor Objects 
Adafruit_ADS1115 adsTDS;    // 0x48 ADDR->GND  - TDS totes 1-4
Adafruit_ADS1115 adsPH;     // 0x49 ADDR->VCC  - pH totes 1-4
Adafruit_ADS1115 adsTote5;  // 0x4A ADDR->SDA  - pH AIN0, TDS AIN1 for tote 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
Preferences preferences;

//data struct 
struct ToteData {
  float ph;
  float phVoltage;
  float ppm;
  float ec;
  float tdsVoltage;
  float tempC;
  float tempF;
  bool  levelLow;   // true = wet
  bool  levelMid;
  bool  levelHigh;
};

//pH Conversion 
// Uses two-point calibration (ph 7 buffer and ph 4 buffer)
float voltageToPH(float voltage) {
  return 4.0 + (CONST_DEFAULT_PH_V4 - voltage) * (3.0 / (CONST_DEFAULT_PH_V4 - CONST_DEFAULT_PH_V7));
}

//TDS Conversion 
// DFRobot cubic formula with temperature compensation
// Source: https://wiki.dfrobot.com/Gravity__Analog_TDS_Sensor___Meter_For_Arduino_SKU__SEN0244
float voltageToPPM(float voltage, float tempC) {
  float compensationCoeff    = 1.0 + (0.02 * (tempC - 25.0));
  float compensatedVoltage   = voltage / compensationCoeff;
  float ppm = (133.42 * pow(compensatedVoltage, 3)
             - 255.86 * pow(compensatedVoltage, 2)
             +  857.39 * compensatedVoltage) * 0.5;
  return ppm;
}

//Mux Channel Select 
// Sets the 4 select lines on CD74HC4067 to route channel
void selectMuxChannel(int channel) {
  digitalWrite(MUX_S0, (channel >> 0) & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  digitalWrite(MUX_S3, (channel >> 3) & 1);
  delayMicroseconds(10);   // Brief settle before reading
}

//IR Level Sensor Read 
// Routes mux to specified channel and reads signal
// Returns true (WET) or false (DRY)
bool readLevelSensor(int muxChannel) {
  selectMuxChannel(muxChannel);
  bool raw = digitalRead(MUX_SIG);
  return !raw;   // Invert: HIGH=dry -> false, LOW=wet -> true
}

//Calibration Save 
// Writes pH calibration voltages to ESP32 flash via Preferences
// Recalculates slope from new values
void saveCalibration(float v7, float v4) {
  preferences.begin("diploidhydro", false);
  preferences.putFloat("phCalV7", v7);
  preferences.putFloat("phCalV4", v4);
  preferences.end();

  calV7    = v7;
  calV4    = v4;
  calSlope = (7.0 - 4.0) / (calV7 - calV4);

  Serial.println("Calibration saved.");
  Serial.print("  V7: ");    Serial.println(calV7, 3);
  Serial.print("  V4: ");    Serial.println(calV4, 3);
  Serial.print("  Slope: "); Serial.println(calSlope, 3);
}

//Serial Commands 
//   cal7     - calibrate pH 7.0 buffer  --needs to be rewritten since I need to recalibrate for 5 totes after reworking from 1
//   cal4     - calibrate pH 4.0 buffer  --needs to be rewritten since I need to recalibrate for 5 totes after reworking from 1
//   resetcal - restore default calibration constants
//   ploton   - enable Arduino Serial Plotter output format
//   plotoff  - disable plot mode, return to human-readable format
//   status   - print current calibration and network status

void checkSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "cal7") {
    int16_t raw    = adsPH.readADC_SingleEnded(0);   // Tote 1 pH on ADS49 AIN0 -- TODO update to recalibrate for all totes, not just 1
    float voltage  = adsPH.computeVolts(raw);
    saveCalibration(voltage, calV4);
    Serial.print("  pH 7.0 calibrated at: "); Serial.println(voltage, 3);
  }
  else if (cmd == "cal4") {
    int16_t raw    = adsPH.readADC_SingleEnded(0);
    float voltage  = adsPH.computeVolts(raw);
    saveCalibration(calV7, voltage);
    Serial.print("  pH 4.0 calibrated at: "); Serial.println(voltage, 3);
  }
  else if (cmd == "resetcal") {
    preferences.begin("diploidhydro", false);
    preferences.remove("phCalV7");
    preferences.remove("phCalV4");
    preferences.end();

    calV7    = CONST_DEFAULT_PH_V7;
    calV4    = CONST_DEFAULT_PH_V4;
    calSlope = CONST_DEFAULT_PH_SLOPE;

    Serial.println("Calibration reset to defaults.");
    Serial.print("  V7: ");    Serial.println(calV7, 3);
    Serial.print("  V4: ");    Serial.println(calV4, 3);
    Serial.print("  Slope: "); Serial.println(calSlope, 3);
  }
  else if (cmd == "ploton") {
    plotMode = true;
    Serial.println("Plot mode ON");
  }
  else if (cmd == "plotoff") {
    plotMode = false;
    Serial.println("Plot mode OFF");
  }
  else if (cmd == "status") {
    Serial.println("── Status ───────────────────────────────");
    Serial.print("  pH cal V7: ");    Serial.println(calV7, 3);
    Serial.print("  pH cal V4: ");    Serial.println(calV4, 3);
    Serial.print("  pH slope:  ");    Serial.println(calSlope, 3);
    Serial.print("  Ethernet:  ");    Serial.println(ethernetOk ? "OK" : "FAILED");
  }
}

//Read One Tote 
// Reads all sensors for a given tote index (0-4)
ToteData readTote(int toteIndex, DeviceAddress tempAddr) {
  ToteData d;

  // pH voltage - ADS49 AIN0-3 for totes 0-3, ADS4A AIN0 for tote 4
  int16_t phRaw;
  if (toteIndex < 4) {
    phRaw = adsPH.readADC_SingleEnded(toteIndex);
    d.phVoltage = adsPH.computeVolts(phRaw);
  } else if(toteIndex == 4) {
    phRaw = adsTote5.readADC_SingleEnded(0);
    d.phVoltage = adsTote5.computeVolts(phRaw);
  }
  d.ph = voltageToPH(d.phVoltage);

  // Temperature - individual probe called by unique address
  d.tempC = tempSensors.getTempC(tempAddr);
  d.tempF = (d.tempC * 9.0 / 5.0) + 32.0;

  // TDS voltage - ADS48 AIN0-3 for totes 0-3, ADS4A AIN1 for tote 4
  int16_t tdsRaw;
  if (toteIndex < 4) {
    tdsRaw = adsTDS.readADC_SingleEnded(toteIndex);
    d.tdsVoltage = adsTDS.computeVolts(tdsRaw);
  } else {
    tdsRaw = adsTote5.readADC_SingleEnded(1);
    d.tdsVoltage = adsTote5.computeVolts(tdsRaw);
  }
  d.ppm = voltageToPPM(d.tdsVoltage, d.tempC);
  d.ec  = d.ppm / 500.0;

  // Level sensors - 3 channels per tote from mux
  d.levelLow  = readLevelSensor(TOTE_LEVEL_CHANNELS[toteIndex][0]);
  d.levelMid  = readLevelSensor(TOTE_LEVEL_CHANNELS[toteIndex][1]);
  d.levelHigh = readLevelSensor(TOTE_LEVEL_CHANNELS[toteIndex][2]);

  return d;

}

//Print Tote Readings to console
void printTote(int toteNum, ToteData d) {
  Serial.print("── Tote "); Serial.print(toteNum); Serial.println(" ──────────────────────────────");
  Serial.print("  pH:    "); Serial.print(d.ph, 2);
  Serial.print("  (");      Serial.print(d.phVoltage, 3); Serial.println("V)");
  Serial.print("  TDS:   "); Serial.print(d.tdsVoltage, 3); Serial.println("V");
  Serial.print("  PPM:   "); Serial.print(d.ppm, 0);   Serial.println(" ppm");
  Serial.print("  EC:    "); Serial.print(d.ec, 2);    Serial.println(" mS/cm");
  Serial.print("  Temp:  "); Serial.print(d.tempC, 2); Serial.print("°C  ");
  Serial.print(d.tempF, 2); Serial.println("°F");
  Serial.print("  Level: ");
  Serial.print("LOW=");  Serial.print(d.levelLow  ? "WET" : "DRY");
  Serial.print("  MID="); Serial.print(d.levelMid  ? "WET" : "DRY");
  Serial.print("  HIGH=");Serial.println(d.levelHigh ? "WET" : "DRY");
}

//Print All Totes (plot mode) 
void printPlotLine(ToteData totes[5]) {
  for (int i = 0; i < 5; i++) {
    Serial.print("T"); Serial.print(i+1); Serial.print("_pH:");
    Serial.print(totes[i].ph, 2); Serial.print(",");
    Serial.print("T"); Serial.print(i+1); Serial.print("_EC:");
    Serial.print(totes[i].ec, 2); Serial.print(",");
    Serial.print("T"); Serial.print(i+1); Serial.print("_Temp:");
    Serial.print(totes[i].tempC, 2);
    if (i < 4) Serial.print(",");
  }
  Serial.println();
}

//Ethernet Init ─────────────────────────────────────────────
// Initializes W5500 with static IP
// Sets ethernetOk flag - checked before any network operations
void initEthernet() {
  Serial.println("Initializing W5500 Ethernet...");

  // Hardware reset W5500
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  delay(100);
  digitalWrite(W5500_RST, HIGH);
  delay(200);

  Ethernet.init(W5500_CS);
  Ethernet.begin(MAC_ADDRESS, STATIC_IP, DNS_SERVER, GATEWAY, SUBNET);

  delay(1000);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("ERROR: W5500 not found. Check wiring.");
    ethernetOk = false;
    return;
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("WARNING: Ethernet cable not connected.");
    ethernetOk = false;
    return;
  }

  ethernetOk = true;
}
 
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("DiploidHydro Brain - Booting...");
  Serial.println("────────────────────────────────────────");

  // Load pH calibration from flash
  preferences.begin("diploidhydro", false);
  calV7 = preferences.getFloat("phCalV7", CONST_DEFAULT_PH_V7);
  calV4 = preferences.getFloat("phCalV4", CONST_DEFAULT_PH_V4);
  preferences.end();
  calSlope = (7.0 - 4.0) / (calV7 - calV4);

  Serial.print("pH cal V7: "); Serial.println(calV7, 3);
  Serial.print("pH cal V4: "); Serial.println(calV4, 3);
  Serial.print("pH slope:  "); Serial.println(calSlope, 3);

  //I2C
  Wire.begin(21, 22);

  //TDS totes 1-4
  adsTDS.setGain(GAIN_ONE);
  if (!adsTDS.begin(0x48)) {
    Serial.println("ERROR: ADS1115 0x48 (TDS) not found. Check ADDR->GND wiring.");
    while (1);
  }
  Serial.println("ADS1115 0x48 (TDS) initialized.");

  //pH totes 1-4 
  adsPH.setGain(GAIN_ONE);
  if (!adsPH.begin(0x49)) {
    Serial.println("ERROR: ADS1115 0x49 (pH) not found. Check ADDR->VCC wiring.");
    while (1);
  }
  Serial.println("ADS1115 0x49 (pH) initialized.");

  //tote 5 pH + TDS
  adsTote5.setGain(GAIN_ONE);
  if (!adsTote5.begin(0x4A)) {
    Serial.println("ERROR: ADS1115 0x4A (Tote 5) not found. Check ADDR->SDA wiring.");
    while (1);
  }
  Serial.println("ADS1115 0x4A (Tote 5) initialized.");

  //temperature sensors
  tempSensors.begin();
  int tempCount = tempSensors.getDeviceCount();
  Serial.print("Temperature sensors found: "); Serial.println(tempCount);
  if (tempCount < 5) {
    Serial.println("WARNING: Expected 5 temp sensors. Check wiring and addresses.");
  }

  //mux select pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  selectMuxChannel(0);
  Serial.println("Water Level Sensor MUX initialized.");

  //W5500 Ethernet
  initEthernet();

  Serial.println("────────────────────────────────────────");
  Serial.println("All systems initialized. Starting readings...");
  Serial.println("────────────────────────────────────────");
}

//Loop 
void loop() {
  checkSerialCommands();

  // Request temperature conversion from all sensors simultaneously to avoid multiple requests
  tempSensors.requestTemperatures();

  ToteData totes[5];
  totes[0] = readTote(0, TOTE1_TEMP_ADDR);
  totes[1] = readTote(1, TOTE2_TEMP_ADDR);
  totes[2] = readTote(2, TOTE3_TEMP_ADDR);
  totes[3] = readTote(3, TOTE4_TEMP_ADDR);
  totes[4] = readTote(4, TOTE5_TEMP_ADDR);
//todo, add rest of the sensor data to output after this refactor

  if (plotMode) {
    printPlotLine(totes);
  } else {
    Serial.println("════════════════════════════════════════");
    for (int i = 0; i < 5; i++) {
      printTote(i + 1, totes[i]);
    }
    Serial.println("════════════════════════════════════════");
  }

  delay(2000);
}
