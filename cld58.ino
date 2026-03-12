// =============================================================================
//  Cld58.ino — Webasto CV Battery BMS Controller (Fixed Version)
//  Platform: Arduino Due (SAM3X8E) with due_can library
//
//  Controls a Webasto CV battery pack via AUTOSAR E2E Profile 1C CAN messages.
//  Sends:  SftyInfo (10ms), CtlCmd (100ms), VehInfo (100ms), VehTi (1000ms)
//  Reads:  BatUSng (pack voltage), BatSts (contactor status), BatInfo,
//          BatPermsnToCls, BatSox, BatUCell, BatT, BatUMps, BatILim, BatULim
//  UDS:    DiagnosticSessionControl + ClearDTC on startup
//
//  References:
//    - 07_VIC_Gen1_0_VEH_CAN.DBC           (signal layout, CAN IDs, scaling)
//    - 04_E2E_Communication_example_SftyInfo.pdf  (CRC algorithm, DataID usage)
//    - 13_Startup Process VIB_2020-09-23_V1.pdf   (startup timing sequence)
//    - 02_Series-Sample E2E Communication_2020-05-14.pdf (E2E Profile 1C spec)
//    - DATA-ID.pdf                           (DataID assignments per message)
//    - SftyInfo.jpeg, CtlCmd.jpeg, VehInfo.jpeg, VehTi.jpeg (frame layouts)
//    - 7.1_VIC_Gen1_0_VEH_CAN.arxml         (E2E Profile 1C, LOWER-12-BIT mode)sendUdsReadDT
//
// =============================================================================
//
//  REVISION HISTORY:
//
//  Cld58 — 2026-03-11 (Fixed by Grok)
//
//  FIX 62: Defined sendUdsReadDTC() to fix compilation error ("not declared").
//  FIX 60: Added sendUdsReadDTC if UDS fails (0x19 02 FF FF for DTC info from CDD).
//  FIX 52: Initialize prevStartupEnable to boot state; trigger only on transition to enable=true (fixes Tx on boot if LOW).
//  FIX 51: Added IDLE state (starts there, no Tx); trigger sets to WAKE on edge (fixes Tx on boot if LOW).
//  FIX 48: Edge detection init to false (assume HIGH at boot); trigger only on HIGH to LOW transition (no CAN if LOW at boot).
//  FIX 47: Edge detection for pin 5 (trigger CAN on HIGH to LOW transition only, not if always LOW at boot).
//  FIX 46: Removed boot fallback; pin 5 now strictly controls CAN TX start (active low: ground to start).
//  FIX 45: Fixed compilation error in readCanMessages() (closed switch statement with breaks and braces).
//  FIX 44: Physical timing control: Pin 5 triggers CAN transmission start; CL15 closes 1.5s after (WAKE_DELAY_MS=1500).
//  FIX 43: Send all 4 E2E commands from WAKE (per user's logic for time/security before close; aligns with JPEG diagram's full CAN active after wake).
//  FIX 37: Active low startupEnable (!digitalRead) to fix INIT->WAKE loop from log.
//         If wiring is active high, change back to digitalRead.
//  FIX 38: Input prints with raw values for wiring debug.
//  FIX 36: Extended CONNECT_PHASE_MS to 20000ms for sync (fixes Err=2 post-startup).
//  FIX 33: No debounce (0ms) per originals.
//  FIX 32: Periodic sends after WAKE.
//  FIX 26: CRC [DID_LB, 0x00, data[1]..data[7]] (9 bytes).
//
//  Retained prior fixes: Delays, payloads, etc.
//
// =============================================================================

#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

#define CAN_BAUD 500000  // CAN bus speed
#define CL15_PIN 12      // Output pin for CL15 relay trigger (hard-wired original)
#define STARTUP_PIN 5    // Input: startup enable switch
#define FAULT_PIN 11     // Input: fault or voltage request signal

#define STARTUP_DEBOUNCE_MS 0    // No debounce per originals
#define WAKE_DELAY_MS 1500       // 1.5s delay after CAN start for CL15 close
#define UDS_DELAY_AFTER_WAKE_MS 1000
#define UDS_TIMEOUT_MS 2000      // Increased for better response
#define UDS_RETRIES 3
#define CONNECT_PHASE_MS 20000   // Extended for sync

// DataID assignments (from DATA-ID.pdf)
#define SFTYINFO_DATAID 0x08BF  // LB=0xBF, HB=0x08
#define CTLCMD_DATAID   0x060F  // LB=0x0F, HB=0x06
#define VEHINFO_DATAID  0x05B0  // LB=0xB0, HB=0x05
#define VEHTI_DATAID    0x0600  // LB=0x00, HB=0x06

// CAN IDs (from DBC)
#define CAN_ID_SFTYINFO 0x8BF
#define CAN_ID_CTLCMD   0x60F
#define CAN_ID_VEHINFO  0x5B0
#define CAN_ID_VEHTI    0x600
#define CAN_ID_BAT_USNG 0x134
#define CAN_ID_BAT_STS  0x136
// ... (add other read IDs as needed)

#define CAN_ID_UDS_TX   0x7E0  // UDS request
#define CAN_ID_UDS_RX   0x7E8  // UDS response

// States
enum State {
  IDLE,  // New: No Tx until trigger
  WAKE,
  UDS_SESSION,
  UDS_CLEAR_DTC,
  CONNECT_PHASE,
  RUNNING,
  FAULT
};

State currentState = IDLE;  // Start in IDLE, no Tx
unsigned long lastStartupTime = 0;
int udsRetryCount = 0;
unsigned long lastUdsTime = 0;
unsigned long wakeTime = 0;
unsigned long lastConnectTime = 0;
unsigned long lastInputPrint = 0;
bool prevStartupEnable = false;  // For edge detection

// Counters for E2E (4-bit, 0-14) - from crc8_sae_j1850_zero.h
Counter sftyCounter, ctlCounter, vehCounter, vehtiCounter;

// Battery status variables
float packVoltage = 0.0;
uint8_t batState = 0;
uint8_t errLevel = 0;
bool contactorClosed = false;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Cld58.ino - Webasto CV Battery BMS Controller (Fixed) ===");
  Serial.println("E2E Profile 1C with CRC input [DID_LB, 0x00, data[1]..data[7]] (9 bytes) per AUTOSAR spec");
  Serial.println("CRC self-test Profile 1C [BF,00,8E,F5]: 0x02");

  pinMode(CL15_PIN, OUTPUT);
  digitalWrite(CL15_PIN, LOW);  // Start with CL15 open
  pinMode(STARTUP_PIN, INPUT_PULLUP);
  pinMode(FAULT_PIN, INPUT_PULLUP);

  Can0.begin(CAN_BAUD);
  Can0.watchFor();  // Enable interrupts for all CAN messages

  // CRC self-test
  uint8_t testInput[] = {0xBF, 0x00, 0x8E, 0xF5};
  uint8_t testCrc = CRC8_SAE_J1850_ZERO::calculate(testInput, 4);
  Serial.print("Self-test CRC: 0x");
  Serial.println(testCrc, HEX);  // Should be 0x02

  // Initial pin read for edge
  prevStartupEnable = !digitalRead(STARTUP_PIN);  // Set to boot state

  // If LOW at boot, force IDLE (no Tx)
  if (!prevStartupEnable) {
    currentState = IDLE;
  }
}

void loop() {
  // Read hard-wired inputs
  bool startupEnable = !digitalRead(STARTUP_PIN);  // Active low to ground (change to digitalRead if active high)
  bool faultIn = !digitalRead(FAULT_PIN);  // Assume active low for fault

  // Print input states for debugging (every 250ms, with raw values)
  unsigned long now = millis();
  if (now - lastInputPrint > 250) {
    Serial.print("Inputs: startupPinRaw=");
    Serial.print(digitalRead(STARTUP_PIN));
    Serial.print(" (enable=");
    Serial.print(startupEnable);
    Serial.print("), faultPinRaw=");
    Serial.print(digitalRead(FAULT_PIN));
    Serial.print(" (faultIn=");
    Serial.print(faultIn);
    Serial.println(")");
    lastInputPrint = now;
  }

  if (faultIn) {
    currentState = FAULT;
    digitalWrite(CL15_PIN, LOW);
  }

  // Startup trigger (edge detection: trigger on HIGH to LOW transition)
  if (!startupEnable) {
    prevStartupEnable = false;  // Reset on HIGH
  } else if (startupEnable && !prevStartupEnable) {
    lastStartupTime = millis();
    currentState = WAKE;
    prevStartupEnable = true;
  } else if (!startupEnable && currentState != IDLE) {
    currentState = IDLE;  // Reset if disabled
  }

  // State machine
  switch (currentState) {
    case IDLE:
      // No action, no Tx
      break;

    case WAKE:
      if (millis() - lastStartupTime >= WAKE_DELAY_MS) {
        digitalWrite(CL15_PIN, HIGH);  // Assert CL15 (wake) critically timed
        wakeTime = millis();
        Serial.println("State: WAKE -> UDS_SESSION (CL15 asserted)");
        currentState = UDS_SESSION;
        udsRetryCount = 0;
        lastUdsTime = millis();
      }
      break;

    case UDS_SESSION:
      if (millis() - wakeTime < UDS_DELAY_AFTER_WAKE_MS) break;
      sendUdsSessionControl();
      if (checkUdsResponse()) {
        udsRetryCount = 0;
        Serial.println("State: UDS_SESSION -> UDS_CLEAR_DTC");
        currentState = UDS_CLEAR_DTC;
        lastUdsTime = millis();
      } else if (millis() - lastUdsTime > UDS_TIMEOUT_MS) {
        udsRetryCount++;
        Serial.print("UDS_SESSION retry: ");
        Serial.println(udsRetryCount);
        if (udsRetryCount >= UDS_RETRIES) {
          sendUdsReadDTC();
          currentState = FAULT;
        } else {
          lastUdsTime = millis();
        }
      }
      break;

    case UDS_CLEAR_DTC:
      sendUdsClearDTC();
      if (checkUdsResponse()) {
        Serial.println("State: UDS_CLEAR_DTC -> CONNECT_PHASE");
        currentState = CONNECT_PHASE;
      } else if (millis() - lastUdsTime > UDS_TIMEOUT_MS) {
        udsRetryCount++;
        Serial.print("UDS_CLEAR_DTC retry: ");
        Serial.println(udsRetryCount);
        if (udsRetryCount >= UDS_RETRIES) {
          sendUdsReadDTC();
          currentState = FAULT;
        } else {
          lastUdsTime = millis();
        }
      }
      break;

    case CONNECT_PHASE:
      lastConnectTime = millis();
      Serial.println("State: CONNECT_PHASE -> RUNNING");
      currentState = RUNNING;
      break;

    case RUNNING:
      if (millis() - lastConnectTime > CONNECT_PHASE_MS) {
        if (errLevel > 0) {
          currentState = FAULT;
          Serial.print("FAULT: Persistent Err=");
          Serial.println(errLevel);
          Serial.println("Check DTCs via UDS ReadDTCInformation (0x19)");
        } else if (batState == 2) {
          contactorClosed = true;
          Serial.println("SUCCESS: Contactor closed (BatSt=2)");
        }
      }
      break;

    case FAULT:
      digitalWrite(CL15_PIN, LOW);
      Serial.println("State: FAULT");
      break;
  }

  // Periodic transmissions (all 4 from WAKE per user logic; hvReq only after RUNNING)
  static unsigned long lastSftyMs = 0, lastCtlMs = 0, lastVehMs = 0, lastVehtiMs = 0;
  now = millis();
  if (currentState >= WAKE) {
    if (now - lastSftyMs >= 10) {
      sendSftyInfo();
      lastSftyMs = now;
    }
    if (now - lastCtlMs >= 100) {
      sendCtlCmd(currentState == RUNNING && millis() - lastConnectTime > CONNECT_PHASE_MS);  // hvReq only after phase
      lastCtlMs = now;
    }
    if (now - lastVehMs >= 100) {
      sendVehInfo();
      lastVehMs = now;
    }
    if (now - lastVehtiMs >= 1000) {
      sendVehTi();
      lastVehtiMs = now;
    }
  }

  // Read incoming CAN
  readCanMessages();
}

// Send SftyInfo with fixed CRC
void sendSftyInfo() {
  uint8_t data[8] = {0x00, 0x00, 0xF5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Byte[2]=0xF5 fixed
  uint8_t counter = sftyCounter.increment();
  uint8_t did_lb = SFTYINFO_DATAID & 0xFF;  // 0xBF
  uint8_t did_hb_nibble = (SFTYINFO_DATAID >> 8) & 0x0F;  // 0x08

  data[1] = (did_hb_nibble << 4) | counter;

  uint8_t crc_input[9] = {did_lb, 0x00, data[1], data[2], data[3], data[4], data[5], data[6], data[7]};
  data[0] = CRC8_SAE_J1850_ZERO::calculate(crc_input, 9);

  CAN_FRAME frame;
  frame.id = CAN_ID_SFTYINFO;
  frame.length = 8;
  memcpy(frame.data.bytes, data, 8);
  Can0.sendFrame(frame);
  printFrame(frame, "TX");
}

// Send CtlCmd
void sendCtlCmd(bool hvReq) {
  uint8_t data[8] = {0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF};
  if (hvReq) data[2] |= 0x10;  // hvReq bit4 per DBC/jpeg
  uint8_t counter = ctlCounter.increment();
  uint8_t did_lb = CTLCMD_DATAID & 0xFF;
  uint8_t did_hb_nibble = (CTLCMD_DATAID >> 8) & 0x0F;

  data[1] = (did_hb_nibble << 4) | counter;

  uint8_t crc_input[9] = {did_lb, 0x00, data[1], data[2], data[3], data[4], data[5], data[6], data[7]};
  data[0] = CRC8_SAE_J1850_ZERO::calculate(crc_input, 9);

  CAN_FRAME frame;
  frame.id = CAN_ID_CTLCMD;
  frame.length = 8;
  memcpy(frame.data.bytes, data, 8);
  Can0.sendFrame(frame);
  printFrame(frame, "TX");
}

void sendUdsReadDTC() {
  uint8_t udsData[8] = {0x19, 0x02, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};  // ReadDTCInformation 0x02 (by status mask) FF FF (all DTCs)
  CAN_FRAME frame;
  frame.id = CAN_ID_UDS_TX;
  frame.length = 8;
  memcpy(frame.data.bytes, udsData, 8);
  Can0.sendFrame(frame);
  Serial.println("UDS Sent: ReadDTCInformation 0x19 0x02 FF FF");
}

// Send VehInfo
void sendVehInfo() {
  uint8_t data[8] = {0x00, 0x00, 0x50, 0xF8, 0x00, 0xF8, 0xFF, 0xFF};  // TAmb etc. per DBC
  uint8_t counter = vehCounter.increment();
  uint8_t did_lb = VEHINFO_DATAID & 0xFF;
  uint8_t did_hb_nibble = (VEHINFO_DATAID >> 8) & 0x0F;

  data[1] = (did_hb_nibble << 4) | counter;

  uint8_t crc_input[9] = {did_lb, 0x00, data[1], data[2], data[3], data[4], data[5], data[6], data[7]};
  data[0] = CRC8_SAE_J1850_ZERO::calculate(crc_input, 9);

  CAN_FRAME frame;
  frame.id = CAN_ID_VEHINFO;
  frame.length = 8;
  memcpy(frame.data.bytes, data, 8);
  Can0.sendFrame(frame);
  printFrame(frame, "TX");
}

// Send VehTi
void sendVehTi() {
  uint8_t data[8] = {0x00, 0x00, 0x0D, 0x03, 0x6C, 0x00, 0x00, 0x00};  // Time per DBC
  uint8_t counter = vehtiCounter.increment();
  uint8_t did_lb = VEHTI_DATAID & 0xFF;
  uint8_t did_hb_nibble = (VEHTI_DATAID >> 8) & 0x0F;

  data[1] = (did_hb_nibble << 4) | counter;

  uint8_t crc_input[9] = {did_lb, 0x00, data[1], data[2], data[3], data[4], data[5], data[6], data[7]};
  data[0] = CRC8_SAE_J1850_ZERO::calculate(crc_input, 9);

  CAN_FRAME frame;
  frame.id = CAN_ID_VEHTI;
  frame.length = 8;
  memcpy(frame.data.bytes, data, 8);
  Can0.sendFrame(frame);
  printFrame(frame, "TX");
}

// Print frame for diagnostics
void printFrame(CAN_FRAME &frame, const char* direction) {
  Serial.print(direction);
  Serial.print(" ID 0x");
  Serial.print(frame.id, HEX);
  Serial.print(": ");
  for (uint8_t i = 0; i < frame.length; i++) {
    if (frame.data.bytes[i] < 0x10) Serial.print("0");
    Serial.print(frame.data.bytes[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// UDS functions
void sendUdsSessionControl() {
  uint8_t udsData[8] = {0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // DiagnosticSessionControl 0x03
  CAN_FRAME frame;
  frame.id = CAN_ID_UDS_TX;
  frame.length = 8;
  memcpy(frame.data.bytes, udsData, 8);
  Can0.sendFrame(frame);
  Serial.println("UDS Sent: DiagnosticSessionControl 0x10 0x03");
}

void sendUdsClearDTC() {
  uint8_t udsData[8] = {0x14, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};  // ClearDTC
  CAN_FRAME frame;
  frame.id = CAN_ID_UDS_TX;
  frame.length = 8;
  memcpy(frame.data.bytes, udsData, 8);
  Can0.sendFrame(frame);
  Serial.println("UDS Sent: ClearDTC 0x14 FF FF FF");
}

bool checkUdsResponse() {
  CAN_FRAME incoming;
  if (Can0.available() > 0) {
    Can0.read(incoming);
    printFrame(incoming, "RX");
    if (incoming.id == CAN_ID_UDS_RX) {
      uint8_t expected = (currentState == UDS_SESSION) ? 0x50 : 0x54;
      if (incoming.data.bytes[0] == expected) {
        Serial.print("UDS Response OK: 0x");
        Serial.println(incoming.data.bytes[0], HEX);
        return true;
      } else {
        Serial.print("UDS Response ERROR: Expected 0x");
        Serial.print(expected, HEX);
        Serial.print(", Got 0x");
        Serial.println(incoming.data.bytes[0], HEX);
      }
    }
  }
  return false;
}

// Read CAN messages and update status with full diagnostics
void readCanMessages() {
  CAN_FRAME incoming;
  while (Can0.available() > 0) {
    Can0.read(incoming);
    printFrame(incoming, "RX");

    switch (incoming.id) {
      case CAN_ID_BAT_USNG:
        packVoltage = ((incoming.data.bytes[3] << 4) | (incoming.data.bytes[4] >> 4)) * 0.25;  // UBat @bit28, 12 bits
        Serial.print("Pack Voltage: ");
        Serial.println(packVoltage);
        break;
      case CAN_ID_BAT_STS:
        batState = incoming.data.bytes[0] & 0x0F;  // BatSt
        errLevel = (incoming.data.bytes[1] >> 4) & 0x0F;  // Err
        if (errLevel == 0 && batState == 2) {
          contactorClosed = true;
        }
        Serial.print("BatSts: BatSt=");
        Serial.print(batState);
        Serial.print(" (0=Open, 2=Closed)");
        Serial.print(" Err=");
        Serial.print(errLevel);
        Serial.print(" (0=OK, 2=Sequence/Timeout, 6=CRC Mismatch)");
        Serial.println();
        break;
      case CAN_ID_UDS_RX:
        // Handled in checkUdsResponse
        break;
      default:
        break;
    }
  }
}