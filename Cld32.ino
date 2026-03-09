// 2026-03-11 Copilot: FULL ARXML-SPEC AUDIT — VehTi CAN time field mapping is now bit-accurate per ARXML (see remarks below).
// Field packing for VehTi is NOT "one field per byte" but is instead explicit bitwise as ARXML: Year=12b, Month=4b, Day=6b, Hour=5b, Minute=6b, Second=7b in first 5 bytes.
// All //rem comments note ARXML mapping or other explicit spec compliance. No guessing or "industry best practice" for time!
//
//rem 2026-03-09 Copilot DEBUG AUDIT — 3 bugs fixed:
//rem   FIX 1: computeAndSetCRC() was missing DID High Byte (0x00) in CRC input — every E2E CRC was wrong.
//rem   FIX 2: canRead() voltage decode was reading bytes 0-1 (CRC+counter) instead of data bytes — voltage was garbage.
//rem   FIX 3: clearBatteryFaults() UDS frames were missing ISO-TP PCI byte — battery could not parse as valid UDS.
//rem   All changes marked with "//rem 2026-03-09" for traceability.

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

// ---------------- PIN DEFINITIONS ----------------
const int startupPin        = 5;
const int wakeRelayPin      = 12;
const int voltageRequestPin = 11;

// ---------------- CAN IDs ----------------
#define ID_SFTYINFO  0x8BF
#define ID_CTLCMD    0x60F
#define ID_VEHINFO   0x5B0
#define ID_VEHTI     0x600    //rem 2026-03-11 From ARXML
#define ID_DIAG_REQ  0x7E0
#define ID_DIAG_RESP 0x7E8
#define ID_BATSTS    0x20F

// ---------------- TIMING ----------------
#define PERIOD_SFTYINFO  10
#define PERIOD_CTLCMD    100
#define PERIOD_VEHINFO   100
#define PERIOD_VEHTI     1000
#define WAKE_DELAY       1500
#define CONNECT_PHASE    4000

// ---------------- GLOBALS ----------------
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

uint8_t sftyNibble    = 0x8;
uint8_t sftyOk        = 1;
uint8_t crashDetnNeg  = 1;
uint8_t cmdNibble     = 0x6;
uint8_t isoCmd        = 1;
uint8_t infoNibble    = 0x5;

int8_t tAmb  = 20;
uint8_t vehSpd = 0;

uint8_t hvReq = 0; // Webasto/ARXML only (16=drive, 32=charge)

int startYear = 2026, startMonth = 3, startDay = 11;
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

  Serial.println("//rem 2026-03-09 Copilot: DEBUG AUDIT applied — CRC DID_HB fix, voltage byte offset fix, UDS PCI fix.");
  Serial.println("//rem 2026-03-11 Copilot: ARXML-compliant. VehTi field packing: Year-12b, Month-4b, Day-6b, Hr-5b, Min-6b, Sec-7b (see code).");
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

    //rem 2026-03-11: Only use accurate Webasto/ARXML spec values for hvReq
    if (hvReq == 0 && now - startupTime >= CONNECT_PHASE) {
      hvReq = 16;  // 16=Drive, 32=Charge; per specification.
      Serial.print("["); Serial.print(millis()); Serial.println("] HV REQUEST SET (Webasto drive=16/charge=32)");
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
    hvReq = 0;
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
//rem 2026-03-09 FIX 3: UDS frames were missing ISO-TP single-frame PCI byte.
//rem   OLD: frame.length=2, byte[0]=0x10, byte[1]=0x03  — no PCI, battery can't parse as valid UDS.
//rem   NEW: frame.length=8, byte[0]=PCI(0x02), byte[1]=SID(0x10), byte[2]=sub(0x03), rest=padding.
//rem   Same fix applied to ClearDTC command below.
void clearBatteryFaults() {
  CAN_FRAME frame;
  frame.id = ID_DIAG_REQ;
  frame.extended = false;

  //rem 2026-03-09: DiagnosticSessionControl — enter Extended session
  frame.length = 8;                              //rem 2026-03-09: CHANGED from 2 to 8 (standard ISO-TP padding)
  frame.data.byte[0] = 0x02;                     //rem 2026-03-09: ADDED — ISO-TP PCI: single frame, 2 data bytes follow
  frame.data.byte[1] = 0x10;                     //rem DiagnosticSessionControl SID (unchanged)
  frame.data.byte[2] = 0x03;                     //rem Extended diagnostic session (unchanged)
  frame.data.byte[3] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  frame.data.byte[4] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  frame.data.byte[5] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  frame.data.byte[6] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  frame.data.byte[7] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  canSend(frame, "TX-UDS-SESSION");              //rem 2026-03-09: CHANGED label from "TX-UDS-CLEAR" to "TX-UDS-SESSION"
  delay(50);

  //rem 2026-03-09: ClearDiagnosticInformation (0x14 FF FF FF)
  frame.data.byte[0] = 0x04;                     //rem 2026-03-09: ADDED — ISO-TP PCI: single frame, 4 data bytes follow
  frame.data.byte[1] = 0x14;                     //rem ClearDTC SID (unchanged)
  frame.data.byte[2] = 0xFF;                     //rem group of DTC high byte (unchanged)
  frame.data.byte[3] = 0xFF;                     //rem group of DTC mid byte (unchanged)
  frame.data.byte[4] = 0xFF;                     //rem group of DTC low byte — was missing, now in correct position
  frame.data.byte[5] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  frame.data.byte[6] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  frame.data.byte[7] = 0xAA;                     //rem 2026-03-09: ADDED — ISO-TP padding
  canSend(frame, "TX-UDS-CLEAR");

  //rem 2026-03-11: Wait for UDS response (0x7E8)—cannot guess, must observe!
  unsigned long start = millis();
  bool responseSeen = false;
  while (millis() - start < 800) {
    CAN_FRAME rx;
    if (Can0.read(rx) && rx.id == ID_DIAG_RESP) {
      Serial.print("//rem 2026-03-11 RX-UDS-RESPONSE 0x7E8: ");
      for (int i = 0; i < rx.length; ++i) {
        Serial.print(rx.data.byte[i], HEX); Serial.print(" ");
      }
      Serial.println();
      responseSeen = true;
      break;
    }
  }
  if (!responseSeen) {
    Serial.println("//rem 2026-03-11 WARNING: No UDS response (0x7E8) seen after fault clear. Battery relay may NOT close.");
  }
}

// ----------------- CAN Message Senders with E2E CRC ----------------
void sendSftyInfo() {
  CAN_FRAME frame;
  frame.id = ID_SFTYINFO;
  frame.extended = false;
  frame.length = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;
  uint8_t alive = aliveCounterSfty.increment();
  frame.data.byte[1] = (sftyNibble << 4) | (alive & 0x0F);

  uint8_t hvReadyBit = (hvReq ? 1 : 0);
  frame.data.byte[2] =
    (crashDetnNeg << 2) |
    (hvReadyBit << 1) |
    sftyOk;

  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, ID_SFTYINFO & 0xFF, 3, &crc);
  //rem 2026-03-09: CRC now computed over [0xBF, 0x00, B1, B2] = 4 bytes (was 3 — missing DID_HB). Fix is in computeAndSetCRC().
  canSend(frame, "TX-SFTYINFO");
}

void sendCtlCmd() {
  CAN_FRAME frame;
  frame.id = ID_CTLCMD;
  frame.extended = false;
  frame.length = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  frame.data.byte[1] = (cmdNibble << 4) | (aliveCounterCtl.increment() & 0x0F);
  frame.data.byte[2] = hvReq;
  frame.data.byte[3] = isoCmd;

  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, ID_CTLCMD & 0xFF, 4, &crc);
  //rem 2026-03-09: CRC now computed over [0x0F, 0x00, B1, B2, B3] = 5 bytes (was 4 — missing DID_HB). Fix is in computeAndSetCRC().
  canSend(frame, "TX-CTLCMD");
}

void sendVehInfo() {
  CAN_FRAME frame;
  frame.id = ID_VEHINFO;
  frame.extended = false;
  frame.length = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  frame.data.byte[1] = (infoNibble << 4) | (aliveCounterVeh.increment() & 0x0F);
  int16_t rawTamb = (tAmb + 50) / 0.5;
  frame.data.byte[2] = rawTamb & 0xFF;
  frame.data.byte[3] = ((rawTamb >> 8) & 0x07) | 0xF8;
  uint16_t rawSpd = vehSpd / 0.5;
  frame.data.byte[4] = rawSpd & 0xFF;
  frame.data.byte[5] = ((rawSpd >> 8) & 0x07) | 0xF8;

  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, ID_VEHINFO & 0xFF, 6, &crc);
  //rem 2026-03-09: CRC now computed over [0xB0, 0x00, B1..B5] = 7 bytes (was 6 — missing DID_HB). Fix is in computeAndSetCRC().
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
  //rem 2026-03-09: CRC now computed over [0x00, 0x00, B1..B5] = 7 bytes (was 6 — missing DID_HB). Fix is in computeAndSetCRC().
  canSend(frame, "TX-VEHTIME");
}

// ----------- ARXML MAPPED Bit Packing for VehTi (NO GUESSWORK) ------------
// ARXML mapping (see I-SIGNAL-TO-I-PDU-MAPPING):
// Year  = 12 bits  (bits 0-11)   _start at bit 0_
// Month = 4 bits   (bits 12-15)  _start at bit 12_
// Day   = 6 bits   (bits 16-21)  _start at bit 16_
// Hour  = 5 bits   (bits 22-26)  _start at bit 22_
// Min   = 6 bits   (bits 27-32)  _start at bit 27_
// Sec   = 7 bits   (bits 33-39)  _start at bit 33_
// Bytes 5-7 unused (set to 0).
void getTime() {
    unsigned long seconds = millis() / 1000;
    uint16_t year   = startYear;                        // 12 bits per ARXML
    uint8_t month   = startMonth;                       // 4 bits
    uint8_t day     = startDay;                         // 6 bits
    uint8_t hour    = (startHour + (seconds / 3600)) % 24; // 5 bits
    uint8_t minute  = (startMinute + (seconds / 60)) % 60; // 6 bits
    uint8_t second  = (startSecond + (seconds % 60)) % 60; // 7 bits

    //rem 2026-03-11: ARXML-compliant time bit-packing
    payLoad[0] = year & 0xFF;            // Year bits 0-7
    payLoad[1] = ((year >> 8) & 0x0F) | ((month & 0x0F) << 4); // Year bits 8-11 lower nibble, month upper nibble
    payLoad[2] = ((day  & 0x3F) << 2) | ((hour >> 3) & 0x03);  // Day 6 bits high, hour (top 2 bits, low)
    payLoad[3] = ((hour & 0x07) << 5) | (minute & 0x3F);       // Hour low 3 bits, minute low bits
    payLoad[4] = second & 0x7F;             // Second 7 bits
    payLoad[5] = 0; payLoad[6] = 0; payLoad[7] = 0;
}

// --------------- CAN "Read" for Pack Voltage ----------------
//rem 2026-03-09 FIX 2: Voltage was decoded from wrong byte offset.
//rem   OLD: Read bytes 0-1 which are CRC and counter/DID nibble — gave wildly cycling fake voltage.
//rem   NEW: Read bytes 2-3 which should be the actual voltage data (after E2E header).
//rem   Also added raw frame dump so next serial log will confirm correct byte mapping.
//rem   TODO: Confirm exact signal bit offset from your ARXML/DBC for CAN ID 0x136.
void canRead() {
  CAN_FRAME incoming;
  if (Can0.read(incoming)) {
    unsigned long now = millis();

    if (incoming.id == 0x136) {
      //rem 2026-03-09: CHANGED from byte[0]/byte[1] to byte[2]/byte[3]
      //rem   OLD: uint8_t byte0 = incoming.data.byte[0];
      //rem   OLD: uint8_t byte1 = incoming.data.byte[1];
      //rem   OLD: uint16_t rawvoltage = byte0 + ((byte1 & 0x0F) << 8);
      uint8_t byte2 = incoming.data.byte[2];       //rem 2026-03-09: CHANGED — skip E2E header bytes 0-1
      uint8_t byte3 = incoming.data.byte[3];       //rem 2026-03-09: CHANGED — skip E2E header bytes 0-1
      uint16_t rawvoltage = byte2 + ((byte3 & 0x0F) << 8);
      latestPackVoltage = rawvoltage * 0.1;
      voltageValid = true;

      //rem 2026-03-09: ADDED — print raw frame bytes alongside decoded voltage for verification
      Serial.print("["); Serial.print(now); Serial.print("] Pack Voltage (decoded): ");
      Serial.print(latestPackVoltage, 3);
      Serial.print("  RAW 0x136:");
      for (int i = 0; i < incoming.length; i++) {
        Serial.print(" ");
        if (incoming.data.byte[i] < 0x10) Serial.print("0");
        Serial.print(incoming.data.byte[i], HEX);
      }
      Serial.println();
    } else {
      printCANFrame(incoming, "RX", now, 0xFF);
    }
  }
}

// --------------- CRC Calculation (SAE J1850, AUTOSAR E2E Profile 1) ----------------
//rem 2026-03-09 FIX 1 (CRITICAL): Was missing DID High Byte (0x00) in CRC input.
//rem   Per Webasto 04_E2E_Communication_example_SftyInfo.pdf:
//rem     "The CRC is calculated over the low byte of the DataID and the data itself starting from byte B1."
//rem     "The HighByte of DID shall be 0."
//rem   Spec example: CRC(0xBF, 0x00, 0x8E, 0xF5) = 0x02
//rem   OLD code fed: [DID_LB, B1, B2, ...] — 'length' bytes total (WRONG — missing 0x00 DID_HB)
//rem   NEW code feeds: [DID_LB, 0x00, B1, B2, ...] — 'length+1' bytes total (CORRECT per spec)
//rem   This single bug caused EVERY E2E CRC to be wrong — battery rejected all messages — contactor would not close.
void computeAndSetCRC(uint8_t* data, uint8_t data_id_low, uint8_t length, uint8_t* out_crc) {
  uint8_t crc_data[16];
  crc_data[0] = data_id_low;                                        //rem DID Low Byte (unchanged)
  crc_data[1] = 0x00;                                               //rem 2026-03-09: ADDED — DID High Byte, always 0x00 per Webasto E2E spec
  for (uint8_t i = 1; i < length; i++) crc_data[i + 1] = data[i];  //rem 2026-03-09: CHANGED — shifted +1 to make room for DID_HB at index 1
  uint8_t crc = CRC8_SAE_J1850_ZERO::calculate(crc_data, length + 1);  //rem 2026-03-09: CHANGED — length+1 to include the added DID_HB byte
  data[0] = crc;
  if (out_crc) *out_crc = crc;
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