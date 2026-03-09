//rem 2026-03-09: Enhanced CAN RX debugging for voltage decoding per CDD/ARXML review.

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

// ---------------- PIN DEFINITIONS ----------------
const int startupPin        = 5;    // "CL15" ignition sense
const int wakeRelayPin      = 12;
const int voltageRequestPin = 11;

// ---------------- CAN IDs ----------------
#define ID_SFTYINFO  0x8BF
#define ID_CTLCMD    0x60F
#define ID_VEHINFO   0x5B0
#define ID_VEHTI     0x600
#define ID_DIAG_REQ  0x7E0
#define ID_DIAG_RESP 0x7E8
#define ID_BATSTS    0x20F

// ---------------- TIMING ----------------
#define PERIOD_SFTYINFO  10     // ms
#define PERIOD_CTLCMD    100
#define PERIOD_VEHINFO   100
#define PERIOD_VEHTI     1000
#define WAKE_DELAY       1500
#define CONNECT_PHASE    4000

// ---------------- OBJECTS, GLOBALS ----------------
Counter aliveCounterSfty;
Counter aliveCounterCtl;
Counter aliveCounterVeh;

bool systemActive   = false;
bool wakeAsserted   = false;
bool faultsCleared  = false;
unsigned long startupTime = 0;

unsigned long lastSfty  = 0;
unsigned long lastCtl   = 0;
unsigned long lastVeh   = 0;
unsigned long lastVehTi = 0;

float latestPackVoltage = 0.0;
bool voltageValid = false;

// ---- Webasto/Vehicle states (protocol defined) ----
uint8_t sftyNibble    = 0x8;
uint8_t sftyOk        = 1;
uint8_t crashDetnNeg  = 1;

uint8_t cmdNibble     = 0x6;
uint8_t isoCmd        = 1;

uint8_t infoNibble    = 0x5;

int8_t tAmb  = 20;
uint8_t vehSpd = 0;

uint8_t hvReq = 0x00;

int startYear = 2026, startMonth = 3, startDay = 9;
int startHour = 12, startMinute = 0, startSecond = 0;
uint8_t payLoad[8];

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(startupPin, INPUT_PULLUP);         
  pinMode(voltageRequestPin, INPUT_PULLUP);  
  pinMode(wakeRelayPin, OUTPUT);

  digitalWrite(wakeRelayPin, LOW);
  Can0.begin(CAN_BPS_500K);

  Serial.println("//rem 2026-03-09 Copilot: CAN diagnostics logging active");
}

// ---------------- MAIN LOOP ----------------
void loop() {
  handleStartup();
  handleVoltageRequest();

  if (systemActive) {
    unsigned long now = millis();

    if (!wakeAsserted && now - startupTime >= WAKE_DELAY) {
      digitalWrite(wakeRelayPin, HIGH);
      wakeAsserted = true;
      Serial.print("["); Serial.print(millis()); Serial.println("] WAKE asserted");
    }

    if (wakeAsserted && !faultsCleared) {
      clearBatteryFaults();
      faultsCleared = true;
    }

    if (hvReq == 0x00 && now - startupTime >= CONNECT_PHASE) {
      hvReq = 0x20;
      Serial.print("["); Serial.print(millis()); Serial.println("] HV REQUEST CONNECT");
    }

    if (now - lastSfty >= PERIOD_SFTYINFO) {
      sendSftyInfo();
      lastSfty = now;
    }
    if (now - lastCtl >= PERIOD_CTLCMD) {
      sendCtlCmd();
      lastCtl = now;
    }
    if (now - lastVeh >= PERIOD_VEHINFO) {
      sendVehInfo();
      lastVeh = now;
    }
    if (now - lastVehTi >= PERIOD_VEHTI) {
      sendVehTi();
      lastVehTi = now;
    }
  }

  canRead();
}

// ---------------- CL15 LOGIC (IGNITION START) ----------------
void handleStartup() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(startupPin);

  if (lastState == HIGH && currentState == LOW) {
    Serial.print("["); Serial.print(millis()); Serial.println("] Startup initiated");
    systemActive = true;
    wakeAsserted = false;
    faultsCleared = false;

    startupTime = millis();
    hvReq = 0x00;
    digitalWrite(wakeRelayPin, LOW);
    delay(50);
  }
  lastState = currentState;
}

// ------------- Button for Printing Voltage ----------------
void handleVoltageRequest() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(voltageRequestPin);

  if (lastState == HIGH && currentState == LOW) {
    Serial.print("["); Serial.print(millis()); Serial.print("] Pack Voltage: ");
    if (voltageValid) {
      Serial.println(latestPackVoltage, 3); 
    } else {
      Serial.println("not yet received");
    }
    delay(300);
  }
  lastState = currentState;
}

// ----------- UDS Fault Clearing -----------
void clearBatteryFaults() {
  CAN_FRAME frame;
  frame.id = ID_DIAG_REQ;
  frame.extended = false;
  frame.length = 2;
  frame.data.byte[0] = 0x10;
  frame.data.byte[1] = 0x03;
  canSend(frame, "TX-UDS-CLEAR");
  delay(50);

  frame.length = 4;
  frame.data.byte[0] = 0x14;
  frame.data.byte[1] = 0xFF;
  frame.data.byte[2] = 0xFF;
  frame.data.byte[3] = 0xFF;
  canSend(frame, "TX-UDS-CLEAR");
}

// ----------------- CAN Message Senders with CRC8 ----------------
void sendSftyInfo() {
  CAN_FRAME frame;
  frame.id = ID_SFTYINFO;
  frame.extended = false;
  frame.length = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;
  uint8_t alive = aliveCounterSfty.increment();
  frame.data.byte[1] = (sftyNibble << 4) | alive;

  uint8_t hvBit = (hvReq > 0) ? 1 : 0;
  frame.data.byte[2] =
    (crashDetnNeg << 2) |
    (hvBit << 1) |
    sftyOk;

  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, ID_SFTYINFO & 0xFF, 3, &crc);
  canSend(frame, "TX-SFTYINFO");
}

void sendCtlCmd() {
  CAN_FRAME frame;
  frame.id = ID_CTLCMD;
  frame.extended = false;
  frame.length = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  frame.data.byte[1] = (cmdNibble << 4) | aliveCounterCtl.increment();
  frame.data.byte[2] = hvReq;
  frame.data.byte[3] = isoCmd;

  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, ID_CTLCMD & 0xFF, 4, &crc);
  canSend(frame, "TX-CTLCMD");
}

void sendVehInfo() {
  CAN_FRAME frame;
  frame.id = ID_VEHINFO;
  frame.extended = false;
  frame.length = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  frame.data.byte[1] = (infoNibble << 4) | aliveCounterVeh.increment();
  int16_t rawTamb = (tAmb + 50) / 0.5;
  frame.data.byte[2] = rawTamb & 0xFF;
  frame.data.byte[3] = ((rawTamb >> 8) & 0x07) | 0xF8;
  uint16_t rawSpd = vehSpd / 0.5;
  frame.data.byte[4] = rawSpd & 0xFF;
  frame.data.byte[5] = ((rawSpd >> 8) & 0x07) | 0xF8;

  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, ID_VEHINFO & 0xFF, 6, &crc);
  canSend(frame, "TX-VEHINFO");
}

void sendVehTi() {
  CAN_FRAME frame;
  frame.id = ID_VEHTI;
  frame.extended = false;
  frame.length = 8;
  getTime();
  for (int i = 0; i < 8; i++) frame.data.byte[i] = payLoad[i];

  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, ID_VEHTI & 0xFF, 6, &crc);
  canSend(frame, "TX-VEHTIME");
}

// --------------- CAN "Read" for Pack Voltage ----------------
void canRead() {
  CAN_FRAME incoming;
  if (Can0.read(incoming)) {
    unsigned long now = millis();

    if (incoming.id == 0x136) {
      //rem 2026-03-09: Print all 8 bytes for post-run analysis!
      Serial.print("["); Serial.print(now); Serial.print("] RX CAN 0x136:");
      for (uint8_t i = 0; i < incoming.length; ++i) {
        Serial.print(" ");
        if (incoming.data.byte[i] < 0x10) Serial.print("0");
        Serial.print(incoming.data.byte[i], HEX);
      }
      Serial.println();

      //rem 2026-03-09: Compute multiple candidate voltage decodings!
      uint16_t raw1 = incoming.data.byte[0] | ((incoming.data.byte[1] & 0x0F) << 8);
      float voltage1 = raw1 * 0.1;

      uint16_t raw2 = incoming.data.byte[0] | (incoming.data.byte[1] << 8);
      float voltage2 = raw2 * 0.1;

      uint16_t raw3 = (incoming.data.byte[0] << 8) | incoming.data.byte[1];
      float voltage3 = raw3 * 0.1;

      uint16_t raw4 = incoming.data.byte[2] | (incoming.data.byte[3] << 8);
      float voltage4 = raw4 * 0.1;

      Serial.print("["); Serial.print(now); Serial.println("] Decode candidates:");
      Serial.print("  raw1(0:0F1)=");   Serial.print(raw1); Serial.print(" voltage1="); Serial.println(voltage1, 2);
      Serial.print("  raw2(0:1l)=");    Serial.print(raw2); Serial.print(" voltage2="); Serial.println(voltage2, 2);
      Serial.print("  raw3(0:1b)=");    Serial.print(raw3); Serial.print(" voltage3="); Serial.println(voltage3, 2);
      Serial.print("  raw4(2:3l)=");    Serial.print(raw4); Serial.print(" voltage4="); Serial.println(voltage4, 2);

      //rem 2026-03-09: Assign your best guess for your UI (refine after running and reviewing logs)
      latestPackVoltage = voltage1;
      voltageValid = true;
    }
    else {
      printCANFrame(incoming, "RX", now, 0xFF);
    }
  }
}

// --------------- CRC Calculation (SAE J1850, repo spec) ----------------
void computeAndSetCRC(uint8_t* data, uint8_t data_id_low, uint8_t length, uint8_t* out_crc) {
  uint8_t crc_data[16];
  crc_data[0] = data_id_low;
  for (uint8_t i = 1; i < length; i++) crc_data[i] = data[i];
  uint8_t crc = CRC8_SAE_J1850_ZERO::calculate(crc_data, length);
  data[0] = crc;
  if (out_crc) *out_crc = crc;
}

// --------------- Time Utilities ----------------
void getTime() {
  unsigned long seconds = millis() / 1000;
  uint8_t sec = (startSecond + seconds) % 60;
  uint8_t min = (startMinute + (seconds / 60)) % 60;
  uint8_t hour = (startHour + (seconds / 3600)) % 24;
  payLoad[0] = 0;
  payLoad[1] = 0;
  payLoad[2] = sec;
  payLoad[3] = min;
  payLoad[4] = hour;
  payLoad[5] = startDay;
  payLoad[6] = startMonth;
  payLoad[7] = startYear - 2000;
}

// --------------- CAN "Send" with Logging ----------------
void canSend(CAN_FRAME &frame, const char *label) {
  unsigned long now = millis();
  uint8_t crc = frame.data.byte[0];
  printCANFrame(frame, label, now, crc);
  Can0.sendFrame(frame);
}

// --------------- Print CAN Frame to Serial ----------------
void printCANFrame(const CAN_FRAME &frame, const char *label, uint32_t timestamp, uint8_t crc) {
  Serial.print("[");
  Serial.print(timestamp);
  Serial.print("] ");
  Serial.print(label);
  Serial.print(" CAN ID: 0x");
  Serial.print(frame.id, HEX);
  Serial.print(" Len: ");
  Serial.print(frame.length);
  Serial.print(" Data:");
  for (int i=0; i < frame.length; ++i) {
    Serial.print(" ");
    if (frame.data.byte[i] < 0x10) Serial.print("0");
    Serial.print(frame.data.byte[i], HEX);
  }
  if (crc != 0xFF) {
    Serial.print(" CRC:0x");
    if (crc < 0x10) Serial.print("0");
    Serial.print(crc, HEX);
  }
  Serial.println();
}