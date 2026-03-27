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
#define MUX_SIG 33

//W5500 SPI
#define W5500_CS   5
#define W5500_SCK  18
#define W5500_MISO 19
#define W5500_MOSI 23
#define W5500_RST  26
#define W5500_INT  27

//Network Configuration
//Placeholder
//byte MAC_ADDRESS[]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 };
//IPAddress STATIC_IP(192, 168, 1, 150);   // TODO: update for network
//IPAddress SUBNET(255, 255, 255, 0);
//IPAddress GATEWAY(192, 168, 1, 1);       // TODO: update for router
//IPAddress DNS_SERVER(8, 8, 8, 8);

//pH Calibration Constants
//Default fallback values if no calibration is stored in flash
const float CONST_DEFAULT_PH_V7 = 1.526;
const float CONST_DEFAULT_PH_V4 = 2.042;

//TDS Calibration
const float TDS_CAL_TEMP = 23.37;   // Temperature at calibration (°C)

//Temperature Sensor Addresses
// Update later with addresses from temp probes
// DeviceAddress TOTE1_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // TODO update
// DeviceAddress TOTE2_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };  // TODO update
// DeviceAddress TOTE3_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };  // TODO update
// DeviceAddress TOTE4_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };  // TODO update
// DeviceAddress TOTE5_TEMP_ADDR = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 };  // TODO update

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
// Per-tote calibration arrays - index 0-4 maps to totes 1-5
// Loaded from flash on boot, updated by cal7/cal4 commands
float calV7[5];
float calV4[5];
bool  plotMode   = false;
bool  ethernetOk = false;

//Sensor Objects
Adafruit_ADS1115 adsTDS;    // 0x48 ADDR->GND  - TDS totes 1-4
Adafruit_ADS1115 adsPH;     // 0x49 ADDR->VCC  - pH totes 1-4
Adafruit_ADS1115 adsTote5;  // 0x4A ADDR->SDA  - pH AIN0, TDS AIN1 for tote 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
Preferences preferences;

//Data Struct
struct ToteData {
  float ph;
  float phVoltage;
  float ppm;
  float ec;
  float tdsVoltage;
  float tempC;
  float tempF;
  bool  levelLow;
  bool  levelMid;
  bool  levelHigh;
};

//pH Conversion
// Two-point calibration using per-tote calV7 and calV4
float voltageToPH(float voltage, int toteIndex) {
  return 4.0 + (calV4[toteIndex] - voltage) * (3.0 / (calV4[toteIndex] - calV7[toteIndex]));
}

//TDS Conversion
// DFRobot cubic formula with temperature compensation
// Source: https://wiki.dfrobot.com/Gravity__Analog_TDS_Sensor___Meter_For_Arduino_SKU__SEN0244
float voltageToPPM(float voltage, float tempC) {
  float compensationCoeff  = 1.0 + (0.02 * (tempC - 25.0));
  float compensatedVoltage = voltage / compensationCoeff;
  float ppm = (133.42 * pow(compensatedVoltage, 3)
             - 255.86 * pow(compensatedVoltage, 2)
             +  857.39 * compensatedVoltage) * 0.5;
  return ppm;
}

//Mux Channel Select
// Binary encodes channel number onto 4 select lines
// Channel 0-15, S0=bit0, S1=bit1, S2=bit2, S3=bit3
void selectMuxChannel(int channel) {
  digitalWrite(MUX_S0, (channel >> 0) & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  digitalWrite(MUX_S3, (channel >> 3) & 1);
  delayMicroseconds(10);
}

//IR Level Sensor Read
// Routes mux to channel, reads digital signal
// Returns true=WET, false=DRY
bool readLevelSensor(int muxChannel) {
  selectMuxChannel(muxChannel);
  bool raw = digitalRead(MUX_SIG);
  return !raw;
}

//Calibration Save
// Writes one tote's pH calibration to flash
void saveCalibration(int toteIndex, float v7, float v4) {
  char keyV7[12], keyV4[12];
  sprintf(keyV7, "phCalV7_%d", toteIndex);
  sprintf(keyV4, "phCalV4_%d", toteIndex);

  preferences.begin("diploidhydro", false);
  preferences.putFloat(keyV7, v7);
  preferences.putFloat(keyV4, v4);
  preferences.end();

  calV7[toteIndex] = v7;
  calV4[toteIndex] = v4;

  Serial.print("Tote "); Serial.print(toteIndex + 1);
  Serial.println(" calibration saved.");
  Serial.print("  V7: "); Serial.println(calV7[toteIndex], 3);
  Serial.print("  V4: "); Serial.println(calV4[toteIndex], 3);
}

//Calibration Load
void loadAllCalibration() {
  preferences.begin("diploidhydro", false);
  for (int i = 0; i < 5; i++) {
    char keyV7[12], keyV4[12];
    sprintf(keyV7, "phCalV7_%d", i);
    sprintf(keyV4, "phCalV4_%d", i);
    calV7[i] = preferences.getFloat(keyV7, CONST_DEFAULT_PH_V7);
    calV4[i] = preferences.getFloat(keyV4, CONST_DEFAULT_PH_V4);
  }
  preferences.end();
}

//pH Sensor Read by Tote
float readPHVoltage(int toteIndex) {
  int16_t raw;
  if (toteIndex < 4) {
    raw = adsPH.readADC_SingleEnded(toteIndex);
    return adsPH.computeVolts(raw);
  } else {
    raw = adsTote5.readADC_SingleEnded(0);
    return adsTote5.computeVolts(raw);
  }
}

//TDS Sensor Read by Tote
float readTDSVoltage(int toteIndex) {
  int16_t raw;
  if (toteIndex < 4) {
    raw = adsTDS.readADC_SingleEnded(toteIndex);
    return adsTDS.computeVolts(raw);
  } else {
    raw = adsTote5.readADC_SingleEnded(1);
    return adsTote5.computeVolts(raw);
  }
}

//Serial Commands
// cal7 N     - calibrate pH 7.0 buffer on tote N (1-5)
// cal4 N     - calibrate pH 4.0 buffer on tote N (1-5)
// resetcal N - reset tote N to defaults
// resetcal all - reset all totes to defaults
// ploton     - enable Arduino Serial Plotter output format
// plotoff    - disable plot mode, return to human-readable format
// status     - print calibration values for all totes and network status
void checkSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // cal7 N
  if (cmd.startsWith("cal7")) {
    int toteNum = cmd.substring(5).toInt();
    if (toteNum < 1 || toteNum > 5) {
      Serial.println("Usage: cal7 N  (N = tote number 1-5)");
      return;
    }
    int toteIndex = toteNum - 1;
    float voltage = readPHVoltage(toteIndex);
    saveCalibration(toteIndex, voltage, calV4[toteIndex]);
    Serial.print("  Tote "); Serial.print(toteNum);
    Serial.print(" pH 7.0 calibrated at: "); Serial.println(voltage, 3);
  }

  // cal4 N
  else if (cmd.startsWith("cal4")) {
    int toteNum = cmd.substring(5).toInt();
    if (toteNum < 1 || toteNum > 5) {
      Serial.println("Usage: cal4 N  (N = tote number 1-5)");
      return;
    }
    int toteIndex = toteNum - 1;
    float voltage = readPHVoltage(toteIndex);
    saveCalibration(toteIndex, calV7[toteIndex], voltage);
    Serial.print("  Tote "); Serial.print(toteNum);
    Serial.print(" pH 4.0 calibrated at: "); Serial.println(voltage, 3);
  }

  // resetcal N or resetcal all
  else if (cmd.startsWith("resetcal")) {
    String arg = cmd.substring(9);
    arg.trim();

    if (arg == "all") {
      preferences.begin("diploidhydro", false);
      for (int i = 0; i < 5; i++) {
        char keyV7[12], keyV4[12];
        sprintf(keyV7, "phCalV7_%d", i);
        sprintf(keyV4, "phCalV4_%d", i);
        preferences.remove(keyV7);
        preferences.remove(keyV4);
        calV7[i] = CONST_DEFAULT_PH_V7;
        calV4[i] = CONST_DEFAULT_PH_V4;
      }
      preferences.end();
      Serial.println("All totes reset to defaults.");
    } else {
      int toteNum = arg.toInt();
      if (toteNum < 1 || toteNum > 5) {
        Serial.println("Usage: resetcal N  or  resetcal all");
        return;
      }
      int toteIndex = toteNum - 1;
      char keyV7[12], keyV4[12];
      sprintf(keyV7, "phCalV7_%d", toteIndex);
      sprintf(keyV4, "phCalV4_%d", toteIndex);
      preferences.begin("diploidhydro", false);
      preferences.remove(keyV7);
      preferences.remove(keyV4);
      preferences.end();
      calV7[toteIndex] = CONST_DEFAULT_PH_V7;
      calV4[toteIndex] = CONST_DEFAULT_PH_V4;
      Serial.print("Tote "); Serial.print(toteNum);
      Serial.println(" reset to defaults.");
    }
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
    Serial.println("── Calibration Status ───────────────────");
    for (int i = 0; i < 5; i++) {
      Serial.print("  Tote "); Serial.print(i + 1);
      Serial.print("  V7="); Serial.print(calV7[i], 3);
      Serial.print("  V4="); Serial.println(calV4[i], 3);
    }
    Serial.print("  Ethernet: "); Serial.println(ethernetOk ? "OK" : "FAILED");
  }
}

//Read One Tote
// Reads all sensors for a given tote index (0-4)
// Bundles into ToteData struct
ToteData readTote(int toteIndex, DeviceAddress tempAddr) {
  ToteData d;

  d.phVoltage = readPHVoltage(toteIndex);
  d.ph        = voltageToPH(d.phVoltage, toteIndex);

  d.tempC = tempSensors.getTempC(tempAddr);
  d.tempF = (d.tempC * 9.0 / 5.0) + 32.0;

  d.tdsVoltage = readTDSVoltage(toteIndex);
  d.ppm        = voltageToPPM(d.tdsVoltage, d.tempC);
  d.ec         = d.ppm / 500.0;

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

//Ethernet Init
// Initializes W5500 with static IP
// Sets ethernetOk flag - checked before any network operations
void initEthernet() {
  Serial.println("Initializing W5500 Ethernet...");

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
  Serial.print("Ethernet OK. IP: ");
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("DiploidHydro Brain - Booting...");
  Serial.println("────────────────────────────────────────");

  // Load pH calibration for all 5 totes from flash
  loadAllCalibration();
  for (int i = 0; i < 5; i++) {
    Serial.print("Tote "); Serial.print(i + 1);
    Serial.print(" cal  V7="); Serial.print(calV7[i], 3);
    Serial.print("  V4="); Serial.println(calV4[i], 3);
  }

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
  Serial.println("Commands: cal7 N, cal4 N, resetcal N, resetcal all, ploton, plotoff, status");
  Serial.println("────────────────────────────────────────");
}

//Loop
void loop() {
  checkSerialCommands();

  // Request temperature conversion from all sensors simultaneously
  tempSensors.requestTemperatures();

  ToteData totes[5];
  totes[0] = readTote(0, TOTE1_TEMP_ADDR);
  totes[1] = readTote(1, TOTE2_TEMP_ADDR);
  totes[2] = readTote(2, TOTE3_TEMP_ADDR);
  totes[3] = readTote(3, TOTE4_TEMP_ADDR);
  totes[4] = readTote(4, TOTE5_TEMP_ADDR);

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
