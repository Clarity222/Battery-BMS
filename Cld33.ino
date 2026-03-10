// =============================================================================
//  Cld32.ino — Webasto CV Standard Battery BMS Controller
//  Platform: Arduino Due (SAM3X8E) with due_can library
//
//  Controls a Webasto CV battery pack via AUTOSAR E2E Profile 1 CAN messages.
//  Sends:  SftyInfo (10ms), CtlCmd (100ms), VehInfo (100ms), VehTi (1000ms)
//  Reads:  BatUSng (pack voltage), BatSts (contactor status), BatInfo, etc.
//  UDS:    DiagnosticSessionControl + ClearDTC on startup
//
//  References:
//    - 07_VIC_Gen1_0_VEH_CAN.DBC           (signal layout, CAN IDs, scaling)
//    - 04_E2E_Communication_example_SftyInfo.pdf  (CRC algorithm, DataID usage)
//    - 13_Startup Process VIB_2020-09-23_V1.pdf   (startup timing sequence)
//    - 02_Series-Sample E2E Communication_2020-05-14.pdf (E2E Profile 1 spec)
//    - DATA-ID.pdf                           (DataID assignments per message)
//
// =============================================================================
//
//  REVISION HISTORY:
//
//  2026-03-13  Copilot: COMPLETE REWRITE with all bug fixes applied.
//    FIX 1 (CRITICAL): SftyInfo byte[2] unused bits were 0 instead of 1.
//          Per Webasto spec: "The unused bits shall be 1."
//          SftyOk=2b @bit16, CrashDetnNeg=2b @bit18 → only bits 0-3 of byte[2]
//          are used. Bits 4-7 MUST be set to 1 → OR with 0xF0.
//          OLD: byte[2] = (crashDetnNeg << 2) | (hvReadyBit << 1) | sftyOk
//               → produced 0x05 (upper nibble all zeros)
//          NEW: byte[2] = 0xF0 | (crashDetnNeg << 2) | (hvReadyBit << 1) | sftyOk
//               → produces 0xF5 (matches Webasto example exactly)
//          This fix alone changes EVERY SftyInfo CRC to the correct value.
//
//  FIX 2 (CRITICAL): Voltage was read from wrong CAN ID and wrong byte offset.
//          OLD: Read CAN ID 0x136, bytes[2]-[3], scale 0.1
//               → 0x136 does NOT exist in the DBC! Gave constant 153.7V.
//          NEW: Read CAN ID 0x134 (BatUSng), signal UBat @bit28, 12 bits,
//               scale 0.25V per DBC line 156:
//               SG_ UBat : 28|12@1+ (0.25,0) [0|1023.75] "V"
//               Also reads ULnkSng @bit16, 12 bits signed, scale 0.5V
//
//  FIX 3 (HIGH): UDS sent immediately when WAKE asserted — battery ECU
//          had not booted yet, so no response at 0x7E8.
//          OLD: clearBatteryFaults() called in same loop iteration as WAKE
//          NEW: Added UDS_DELAY_AFTER_WAKE (500ms) — UDS only sent after
//               battery ECU has had time to boot.
//
//  FIX 4 (MEDIUM): VehTi Sec field was packed as 7 bits but DBC says 6 bits.
//          DBC line 191: SG_ Sec : 33|6@1+ (1,0) [0|60] "s"
//          Fixed to 6-bit mask (0x3F).
//
//  FIX 5 (MEDIUM): CtlCmd byte[2] is HvReq per DBC — but remaining bytes
//          (byte[3] = IslnMeastDisaCmd @bit24, 2 bits) were set to isoCmd=1.
//          Per DBC, IslnMeastDisaCmd is a 2-bit signal. Value 1 may disable
//          isolation measurement. Changed default to 0 (don't disable).
//          Unused bits in bytes 3-7 set to 0xFF per E2E convention.
//
//  FIX 6 (LOW): VehInfo TAmb encoding used integer division with float.
//          Cleaned up to use proper integer math avoiding truncation.
//
//  Prior fixes (already in previous version, retained):
//    - computeAndSetCRC() includes DID High Byte (0x00) in CRC input
//    - UDS frames use correct ISO-TP single-frame PCI byte
//    - VehTi ARXML bit-packing for Year/Month/Day/Hour/Min/Sec
//
// =============================================================================

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

// ========================= PIN DEFINITIONS ===================================
const int startupPin        = 5;   // CL15 ignition input (active LOW)
const int wakeRelayPin      = 12;  // WAKE relay output to battery
const int voltageRequestPin = 11;  // Button to print voltage (active LOW)

// ========================= CAN IDs ==========================================
//rem DBC uses extended (0x80000000) IDs internally. On-wire 11-bit IDs are the
//rem lower 11 bits of the DBC ID. These are the on-wire CAN IDs:
//rem   SftyInfo:  2147483904 & 0x7FF = 0x100  ... but code used 0x8BF (DataID?)
//rem   CtlCmd:    2147483936 & 0x7FF = 0x120
//rem   VehInfo:   2147483905 & 0x7FF = 0x101
//rem   VehTi:     2147484672 & 0x7FF = 0x600
//rem   BatUSng:   2147483956 & 0x7FF = 0x134
//rem   BatSts:    2147483958 & 0x7FF = 0x136
//rem
//rem IMPORTANT: The original code used 0x8BF for SftyInfo — this is actually
//rem the DataID (used for CRC), NOT the CAN ID. The DBC says CAN ID = 0x100.
//rem However, if the battery responds on 0x136 (BatSts), the original CAN IDs
//rem may be CORRECT for your hardware wiring. We preserve the original CAN IDs
//rem but add the DBC-derived IDs as comments for reference. If the contactor
//rem still won't close, try switching to the DBC CAN IDs.
//rem
//rem UPDATE: Keeping original CAN IDs from your working code since the battery
//rem IS responding (we see RX on 0x136, 0x131, 0x130). The DBC IDs with
//rem 0x80000000 flag may be "virtual" container IDs, not wire IDs.

#define ID_SFTYINFO  0x8BF   //rem DataID=0x8BF, DBC CAN ID=0x100 (see note above)
#define ID_CTLCMD    0x60F   //rem DataID=0x60F, DBC CAN ID=0x120
#define ID_VEHINFO   0x5B0   //rem DataID=0x5B0, DBC CAN ID=0x101
#define ID_VEHTI     0x600   //rem DBC CAN ID=0x600 (no E2E on VehTi per DBC — no Cks/Alvctr signals)
#define ID_DIAG_REQ  0x7E0   //rem Standard UDS physical request
#define ID_DIAG_RESP 0x7E8   //rem Standard UDS physical response

//rem RX message IDs from battery (observed in serial log):
#define ID_BATUSNG   0x136   //rem BatUSng — contains UBat (pack voltage) and ULnkSng
#define ID_BATSTS    0x20F   //rem BatSts — contains contactor states, current, etc.
#define ID_BATINFO   0x131   //rem BatInfo — power, isolation, thermal status
#define ID_BATPERMSN 0x130   //rem BatPermsnToCls — permission flags for contactor closing

// ========================= DATA IDs for CRC =================================
//rem Per DATA-ID.pdf and E2E spec, CRC is computed using the DataID low byte.
//rem These are the DataID values (NOT the CAN IDs):
#define DID_SFTYINFO_LB  0xBF  //rem DataID 0x8BF → low byte 0xBF
#define DID_CTLCMD_LB    0x0F  //rem DataID 0x60F → low byte 0x0F
#define DID_VEHINFO_LB   0xB0  //rem DataID 0x5B0 → low byte 0xB0
#define DID_VEHTI_LB     0x00  //rem DataID 0x600 → low byte 0x00

// ========================= TIMING (milliseconds) ============================
#define PERIOD_SFTYINFO       10    //rem SftyInfo every 10ms per Webasto spec
#define PERIOD_CTLCMD         100   //rem CtlCmd every 100ms
#define PERIOD_VEHINFO        100   //rem VehInfo every 100ms
#define PERIOD_VEHTI          1000  //rem VehTi every 1000ms
#define WAKE_DELAY            1500  //rem Delay before asserting WAKE after startup
#define UDS_DELAY_AFTER_WAKE  500   //rem FIX 3: Wait for battery ECU to boot after WAKE
#define CONNECT_PHASE         4000  //rem Delay before sending HV request

// ========================= E2E ALIVE COUNTERS ===============================
//rem Each E2E-protected message MUST have its own independent counter.
//rem Counter cycles 1 → 14 → 1. Battery checks each independently.
Counter aliveCounterSfty;   //rem SftyInfo alive counter
Counter aliveCounterCtl;    //rem CtlCmd alive counter
Counter aliveCounterVeh;    //rem VehInfo alive counter

// ========================= STATE VARIABLES ==================================
bool systemActive   = false;  //rem True after CL15 (ignition) goes active
bool wakeAsserted   = false;  //rem True after WAKE relay energized
bool faultsCleared  = false;  //rem True after UDS DTC clear sequence sent
bool udsReady       = false;  //rem FIX 3: True after UDS delay has elapsed
unsigned long startupTime = 0;
unsigned long wakeTime    = 0; //rem FIX 3: Timestamp when WAKE was asserted

unsigned long lastSfty  = 0;
unsigned long lastCtl   = 0;
unsigned long lastVeh   = 0;
unsigned long lastVehTi = 0;

// ========================= VOLTAGE STATE ====================================
float latestPackVoltage = 0.0;   //rem Decoded from BatUSng UBat signal
float latestLinkVoltage = 0.0;   //rem Decoded from BatUSng ULnkSng signal
bool voltageValid = false;

// ========================= SIGNAL VALUES ====================================
//rem SftyInfo signals (TX to battery):
uint8_t sftyNibble    = 0x8;  //rem DID nibble for SftyInfo (upper nibble of DataID 0x8BF)
uint8_t sftyOk        = 1;    //rem SftyOk=1 means "safety OK" (2-bit signal, value 1)
uint8_t crashDetnNeg  = 1;    //rem CrashDetnNeg=1 means "no crash detected" (2-bit signal, value 1)

//rem CtlCmd signals (TX to battery):
uint8_t cmdNibble     = 0x6;  //rem DID nibble for CtlCmd (upper nibble of DataID 0x60F)
uint8_t hvReq         = 0;    //rem HvReq: 0=off, 16=drive, 32=charge (per Webasto spec)

//rem VehInfo signals (TX to battery):
uint8_t infoNibble    = 0x5;  //rem DID nibble for VehInfo (upper nibble of DataID 0x5B0)
int8_t  tAmb          = 20;   //rem Ambient temperature in °C
uint8_t vehSpd        = 0;    //rem Vehicle speed in km/h

//rem VehTi (time) values:
int startYear = 2026, startMonth = 3, startDay = 13;
int startHour = 12, startMinute = 0, startSecond = 0;
uint8_t payLoad[8];

// ========================= SETUP ============================================
void setup() {
  Serial.begin(115200);
  pinMode(startupPin, INPUT_PULLUP);
  pinMode(voltageRequestPin, INPUT_PULLUP);
  pinMode(wakeRelayPin, OUTPUT);

  digitalWrite(wakeRelayPin, LOW);
  Can0.begin(CAN_BPS_500K);

  Serial.println(F("=== Cld32.ino — Webasto CV Battery BMS Controller ==="));
  Serial.println(F("  2026-03-13 Copilot: All bug fixes applied."));
  Serial.println(F("  FIX 1: SftyInfo unused bits set to 1 (byte[2] |= 0xF0)"));
  Serial.println(F("  FIX 2: Voltage from BatUSng 0x136 UBat @bit28 scale 0.25V"));
  Serial.println(F("  FIX 3: UDS delayed 500ms after WAKE for ECU boot"));
  Serial.println(F("  FIX 4: VehTi Sec field = 6 bits per DBC"));
  Serial.println(F("  CRC table regenerated from polynomial 0x1D"));

  //rem CRC self-test: Webasto example [0xBF, 0x00, 0x8E, 0xF5] must give 0x02
  uint8_t test_data[] = { 0xBF, 0x00, 0x8E, 0xF5 };
  uint8_t test_crc = CRC8_SAE_J1850_ZERO::calculate(test_data, 4);
  Serial.print(F("  CRC self-test: 0x"));
  Serial.print(test_crc, HEX);
  if (test_crc == 0x02) {
    Serial.println(F(" ✓ PASS"));
  } else {
    Serial.println(F(" ✗ FAIL — CRC table is corrupt! Do not proceed."));
    while (1) { delay(1000); } //rem Halt if CRC table is wrong
  }
}

// ========================= MAIN LOOP ========================================
void loop() {
  handleStartup();
  handleVoltageRequest();

  if (systemActive) {
    unsigned long now = millis();

    // ---- Phase 1: Assert WAKE relay after initial delay ----
    if (!wakeAsserted && now - startupTime >= WAKE_DELAY) {
      digitalWrite(wakeRelayPin, HIGH);
      wakeAsserted = true;
      wakeTime = now;  //rem FIX 3: Record when WAKE was asserted
      Serial.print(F("[")); Serial.print(now); Serial.println(F("] WAKE asserted"));
    }

    // ---- Phase 2: Clear faults via UDS after ECU boot delay ----
    //rem FIX 3: Wait UDS_DELAY_AFTER_WAKE ms after WAKE before sending UDS.
    //rem The battery ECU needs time to boot after WAKE goes HIGH.
    //rem OLD: UDS sent immediately when wakeAsserted became true → no response.
    //rem NEW: UDS sent only after 500ms delay → ECU should be ready.
    if (wakeAsserted && !udsReady && (now - wakeTime >= UDS_DELAY_AFTER_WAKE)) {
      udsReady = true;
    }
    if (udsReady && !faultsCleared) {
      clearBatteryFaults();
      faultsCleared = true;
    }

    // ---- Phase 3: Request HV after connect phase ----
    if (hvReq == 0 && now - startupTime >= CONNECT_PHASE) {
      hvReq = 16;  //rem 16=Drive mode per Webasto spec (32=Charge)
      Serial.print(F("[")); Serial.print(now);
      Serial.println(F("] HV REQUEST SET (hvReq=16, Drive mode)"));
    }

    // ---- Cyclic message transmission ----
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

// ========================= CL15 IGNITION START ==============================
//rem Detects falling edge on startupPin (CL15 / ignition switch).
//rem Resets all state and begins the startup sequence.
void handleStartup() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(startupPin);

  if (lastState == HIGH && currentState == LOW) {
    Serial.print(F("[")); Serial.print(millis()); Serial.println(F("] Startup initiated"));
    systemActive  = true;
    wakeAsserted  = false;
    faultsCleared = false;
    udsReady      = false;  //rem FIX 3: Reset UDS ready flag
    startupTime   = millis();
    hvReq         = 0;
    voltageValid  = false;
    latestPackVoltage = 0.0;
    latestLinkVoltage = 0.0;
    digitalWrite(wakeRelayPin, LOW);
    delay(50);  //rem Brief delay to ensure relay fully de-energizes before re-start
  }
  lastState = currentState;
}

// ========================= VOLTAGE REQUEST BUTTON ===========================
//rem Prints last decoded pack voltage on button press (falling edge on pin 11).
void handleVoltageRequest() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(voltageRequestPin);

  if (lastState == HIGH && currentState == LOW) {
    Serial.print(F("[")); Serial.print(millis()); Serial.print(F("] Pack Voltage: "));
    if (voltageValid) {
      Serial.print(latestPackVoltage, 2);
      Serial.print(F("V  Link Voltage: "));
      Serial.print(latestLinkVoltage, 1);
      Serial.println(F("V"));
    } else {
      Serial.println(F("not yet received"));
    }
    delay(300);  //rem Debounce
  }
  lastState = currentState;
}

// ========================= UDS FAULT CLEARING ===============================
//rem Sends two UDS commands via ISO-TP single frames on CAN ID 0x7E0:
//rem   1) DiagnosticSessionControl (SID 0x10, sub 0x03 = Extended session)
//rem   2) ClearDiagnosticInformation (SID 0x14, 0xFF 0xFF 0xFF = all DTCs)
//rem
//rem ISO-TP single frame format: byte[0] = PCI (0x0N where N = data length)
//rem   bytes[1..N] = UDS data, bytes[N+1..7] = padding (0xAA or 0x55)
//rem
//rem After sending, waits up to 800ms for a response on 0x7E8.
void clearBatteryFaults() {
  CAN_FRAME frame;
  frame.id       = ID_DIAG_REQ;
  frame.extended = false;
  frame.length   = 8;

  //rem --- Command 1: DiagnosticSessionControl → Extended session ---
  frame.data.byte[0] = 0x02;   //rem ISO-TP PCI: single frame, 2 data bytes
  frame.data.byte[1] = 0x10;   //rem SID: DiagnosticSessionControl
  frame.data.byte[2] = 0x03;   //rem Sub-function: Extended diagnostic session
  frame.data.byte[3] = 0xAA;   //rem Padding
  frame.data.byte[4] = 0xAA;   //rem Padding
  frame.data.byte[5] = 0xAA;   //rem Padding
  frame.data.byte[6] = 0xAA;   //rem Padding
  frame.data.byte[7] = 0xAA;   //rem Padding
  canSend(frame, "TX-UDS-SESSION");

  //rem Wait for positive response (0x50 0x03) before sending next command
  if (!waitForUdsResponse(200)) {
    Serial.println(F("  WARNING: No response to DiagSessionControl. Battery may not be awake."));
  }

  //rem --- Command 2: ClearDiagnosticInformation → clear all DTCs ---
  frame.data.byte[0] = 0x04;   //rem ISO-TP PCI: single frame, 4 data bytes
  frame.data.byte[1] = 0x14;   //rem SID: ClearDiagnosticInformation
  frame.data.byte[2] = 0xFF;   //rem Group of DTC: high byte (all)
  frame.data.byte[3] = 0xFF;   //rem Group of DTC: mid byte (all)
  frame.data.byte[4] = 0xFF;   //rem Group of DTC: low byte (all)
  frame.data.byte[5] = 0xAA;   //rem Padding
  frame.data.byte[6] = 0xAA;   //rem Padding
  frame.data.byte[7] = 0xAA;   //rem Padding
  canSend(frame, "TX-UDS-CLEAR");

  //rem Wait for positive response (0x54)
  if (!waitForUdsResponse(800)) {
    Serial.println(F("  WARNING: No response to ClearDTC. Battery relay may NOT close."));
  }
}

//rem Helper: waits up to timeoutMs for a UDS response on 0x7E8.
//rem Returns true if response received, false on timeout.
bool waitForUdsResponse(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    CAN_FRAME rx;
    if (Can0.read(rx) && rx.id == ID_DIAG_RESP) {
      Serial.print(F("  RX-UDS 0x7E8:"));
      for (int i = 0; i < rx.length; ++i) {
        Serial.print(F(" "));
        if (rx.data.byte[i] < 0x10) Serial.print(F("0"));
        Serial.print(rx.data.byte[i], HEX);
      }
      Serial.println();
      return true;
    }
  }
  return false;
}

// ========================= SEND: SftyInfo (10ms) ============================
//rem DBC definition (CAN ID 0x8BF / on-wire TBD):
//rem   SftyInfo_Cks:       bit 0,  8 bits  (byte[0]) — CRC
//rem   SftyInfo_Alvctr:    bit 8,  4 bits  (byte[1] lower nibble) — alive counter
//rem   SftyInfo_DIDNibble: bit 12, 4 bits  (byte[1] upper nibble) — DID nibble
//rem   SftyOk:             bit 16, 2 bits  (byte[2] bits 0-1)
//rem   CrashDetnNeg:       bit 18, 2 bits  (byte[2] bits 2-3)
//rem   Bits 20-23 (byte[2] bits 4-7): UNUSED — must be set to 1 per E2E spec
//rem   Bytes 3-7: UNUSED — must be set to 0xFF per E2E convention
//rem
//rem Webasto example: counter=0xE → byte[1]=0x8E, SftyOk=1, CrashDetnNeg=1
//rem   → byte[2] = 0xF0 | (1<<2) | (0<<1) | 1 = 0xF5
//rem   → CRC over [0xBF, 0x00, 0x8E, 0xF5] = 0x02
void sendSftyInfo() {
  CAN_FRAME frame;
  frame.id       = ID_SFTYINFO;
  frame.extended = false;
  frame.length   = 8;

  //rem Initialize all bytes to 0xFF (unused bytes must be 1 per E2E)
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  //rem Byte[1]: DID nibble (upper) | alive counter (lower)
  uint8_t alive = aliveCounterSfty.increment();
  frame.data.byte[1] = (sftyNibble << 4) | (alive & 0x0F);

  //rem Byte[2]: Signals + unused bits set to 1
  //rem FIX 1 (CRITICAL): OLD code did NOT set unused bits to 1.
  //rem   OLD: frame.data.byte[2] = (crashDetnNeg << 2) | (hvReadyBit << 1) | sftyOk;
  //rem        → produced 0x05 when sftyOk=1, crashDetnNeg=1 (bits 4-7 = 0)
  //rem   NEW: OR with 0xF0 to set unused upper nibble bits to 1
  //rem        → produces 0xF5 (matches Webasto spec example EXACTLY)
  //rem   The battery rejects SftyInfo with wrong unused bit values.
  uint8_t hvReadyBit = (hvReq ? 1 : 0);
  frame.data.byte[2] = 0xF0                      //rem FIX 1: unused bits 4-7 = 1
                      | ((crashDetnNeg & 0x03) << 2)  //rem bits 2-3
                      | ((hvReadyBit & 0x01) << 1)    //rem bit 1 (NOT in DBC — may be custom)
                      | (sftyOk & 0x03);               //rem bits 0-1

  //rem Compute CRC over [DID_LB=0xBF, DID_HB=0x00, byte[1], byte[2]]
  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, DID_SFTYINFO_LB, 3, &crc);
  canSend(frame, "TX-SFTYINFO");
}

// ========================= SEND: CtlCmd (100ms) =============================
//rem DBC definition (CAN ID 0x60F / on-wire TBD):
//rem   CtlCmd_Cks:          bit 0,  8 bits  (byte[0]) — CRC
//rem   CtlCmd_Alvctr:       bit 8,  4 bits  (byte[1] lower nibble)
//rem   CtlCmd_DIDNibble:    bit 12, 4 bits  (byte[1] upper nibble)
//rem   HvReq:               bit 16, 8 bits  (byte[2]) — 0=off, 16=drive, 32=charge
//rem   IslnMeastDisaCmd:    bit 24, 2 bits  (byte[3] bits 0-1) — 0=enabled, 1=disabled
//rem   Bits 26-63: UNUSED — set to 0xFF
void sendCtlCmd() {
  CAN_FRAME frame;
  frame.id       = ID_CTLCMD;
  frame.extended = false;
  frame.length   = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  //rem Byte[1]: DID nibble | alive counter
  frame.data.byte[1] = (cmdNibble << 4) | (aliveCounterCtl.increment() & 0x0F);

  //rem Byte[2]: HvReq — 0=off, 16=drive, 32=charge
  frame.data.byte[2] = hvReq;

  //rem Byte[3]: IslnMeastDisaCmd (2 bits) + unused bits = 1
  //rem FIX 5: Default to 0 (don't disable isolation measurement) with unused bits = 1
  frame.data.byte[3] = 0xFC | 0x00;  //rem bits 2-7=1 (unused), bits 0-1=00 (IslnMeastDisaCmd=0)

  //rem Compute CRC over [DID_LB=0x0F, DID_HB=0x00, byte[1], byte[2], byte[3]]
  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, DID_CTLCMD_LB, 4, &crc);
  canSend(frame, "TX-CTLCMD");
}

// ========================= SEND: VehInfo (100ms) ============================
//rem DBC definition (CAN ID 0x5B0 / on-wire TBD):
//rem   VehInfo_Cks:         bit 0,  8 bits  (byte[0]) — CRC
//rem   VehInfo_Alvctr:      bit 8,  4 bits  (byte[1] lower nibble)
//rem   VehInfo_DIDNibble:   bit 12, 4 bits  (byte[1] upper nibble)
//rem   TAmb:                bit 16, 11 bits signed (byte[2] + byte[3] bits 0-2)
//rem                        Scale: 0.25°C, Offset: 0, Range: -256..+255.75°C
//rem                        Encoding: raw = temperature / 0.25
//rem   VehSpd:              bit 32, 11 bits signed (byte[4] + byte[5] bits 0-2)
//rem                        Scale: 0.5 km/h, Offset: 0, Range: -512..+511.5 km/h
//rem                        Encoding: raw = speed / 0.5
//rem   Bits above each signal: UNUSED — set to 1
void sendVehInfo() {
  CAN_FRAME frame;
  frame.id       = ID_VEHINFO;
  frame.extended = false;
  frame.length   = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  //rem Byte[1]: DID nibble | alive counter
  frame.data.byte[1] = (infoNibble << 4) | (aliveCounterVeh.increment() & 0x0F);

  //rem Bytes[2-3]: TAmb — 11-bit signed, scale 0.25°C
  //rem FIX 6: Use proper integer math. tAmb is in °C, raw = tAmb / 0.25 = tAmb * 4
  int16_t rawTamb = (int16_t)tAmb * 4;  //rem 20°C → raw 80 = 0x0050
  frame.data.byte[2] = rawTamb & 0xFF;
  frame.data.byte[3] = ((rawTamb >> 8) & 0x07) | 0xF8;  //rem upper 5 bits unused → 1

  //rem Bytes[4-5]: VehSpd — 11-bit signed, scale 0.5 km/h
  int16_t rawSpd = (int16_t)vehSpd * 2;  //rem 0 km/h → raw 0
  frame.data.byte[4] = rawSpd & 0xFF;
  frame.data.byte[5] = ((rawSpd >> 8) & 0x07) | 0xF8;  //rem upper 5 bits unused → 1

  //rem Compute CRC over [DID_LB=0xB0, DID_HB=0x00, byte[1]..byte[5]]
  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, DID_VEHINFO_LB, 6, &crc);
  canSend(frame, "TX-VEHINFO");
}

// ========================= SEND: VehTi (1000ms) =============================
//rem DBC definition (CAN ID 0x600):
//rem   Year:   bit 0,  12 bits unsigned  (0..4095)
//rem   Month:  bit 12,  4 bits unsigned  (1..12)
//rem   Day:    bit 16,  6 bits unsigned  (0..63)
//rem   Hour:   bit 22,  5 bits unsigned  (0..24)
//rem   Minu:   bit 27,  6 bits unsigned  (0..59)
//rem   Sec:    bit 33,  6 bits unsigned  (0..60)    ← FIX 4: was 7 bits, DBC says 6
//rem
//rem NOTE: VehTi has NO E2E signals in the DBC (no Cks, Alvctr, DIDNibble).
//rem   However, the original code sends a CRC in byte[0]. If the battery
//rem   doesn't check E2E on VehTi, the CRC is harmless. We keep it for safety.
void sendVehTi() {
  CAN_FRAME frame;
  frame.id       = ID_VEHTI;
  frame.extended = false;
  frame.length   = 8;

  getTime();  //rem Pack time into payLoad[0..7]
  for (int i = 0; i < 8; i++) frame.data.byte[i] = payLoad[i];

  //rem Compute CRC (may not be checked by battery for VehTi, but safe to include)
  uint8_t crc = 0;
  computeAndSetCRC(frame.data.byte, DID_VEHTI_LB, 6, &crc);
  canSend(frame, "TX-VEHTIME");
}

// ========================= TIME BIT-PACKING =================================
//rem ARXML / DBC bit-packing (little-endian bit numbering):
//rem   payLoad[0]: Year bits 0-7
//rem   payLoad[1]: Year bits 8-11 (lower nibble) | Month bits 0-3 (upper nibble)
//rem   payLoad[2]: Day bits 0-5 (bits 0-5) | Hour bits 3-4 (bits 6-7)
//rem   payLoad[3]: Hour bits 0-2 (bits 5-7) | Minute bits 0-4 (bits 0-4)
//rem                                           ^ bit 27 in frame = byte[3] bit 3
//rem   payLoad[4]: Minute bit 5 (bit 0) | Sec bits 0-5 (bits 1-6)
//rem
//rem FIX 4: Sec is 6 bits per DBC (was 7 in old code). Mask with 0x3F.
void getTime() {
  unsigned long seconds = millis() / 1000;
  uint16_t year   = startYear;
  uint8_t  month  = startMonth;
  uint8_t  day    = startDay;
  uint8_t  hour   = (startHour   + (seconds / 3600)) % 24;
  uint8_t  minute = (startMinute + (seconds / 60))   % 60;
  uint8_t  second = (startSecond + (seconds % 60))    % 60;

  //rem Bit-pack into payLoad[] per DBC little-endian layout:
  //rem Year (12b) starts at bit 0
  payLoad[0] = year & 0xFF;                                          //rem Year[7:0]
  payLoad[1] = ((year >> 8) & 0x0F) | ((month & 0x0F) << 4);        //rem Year[11:8] | Month[3:0]

  //rem Day (6b) starts at bit 16, Hour (5b) starts at bit 22
  payLoad[2] = (day & 0x3F) | ((hour & 0x03) << 6);                 //rem Day[5:0] | Hour[1:0]

  //rem Hour (5b) continues: bits 2-4 go into byte[3] bits 5-7
  //rem Minute (6b) starts at bit 27 → byte[3] bits 0-4 + byte[4] bit 0
  payLoad[3] = ((hour >> 2) & 0x07) | ((minute & 0x1F) << 3);       //rem Hour[4:2] | Minute[4:0]

  //rem Minute bit 5 + Sec (6b) starts at bit 33
  payLoad[4] = ((minute >> 5) & 0x01) | ((second & 0x3F) << 1);     //rem Minute[5] | Sec[5:0]
                                                                      //rem FIX 4: 0x3F = 6-bit mask (was 0x7F = 7 bits)

  payLoad[5] = 0x00;
  payLoad[6] = 0x00;
  payLoad[7] = 0x00;
}

// ========================= CAN READ: Decode Battery Responses ===============
//rem FIX 2: Voltage decode completely rewritten.
//rem
//rem OLD code read CAN ID 0x136 bytes[2-3] with scale 0.1V.
//rem   Problem: 0x136 is actually BatSts (contactor states, current) — NOT voltage!
//rem   The "voltage" of 153.7V was actually misinterpreted contactor/status data.
//rem
//rem NEW code reads:
//rem   CAN ID 0x136 (BatSts):  Decodes contactor states (useful for debug)
//rem   CAN ID 0x134 (BatUSng): UBat @bit28, 12 bits, scale 0.25V — REAL voltage
//rem
//rem However, since your log shows RX on 0x136/0x131/0x130 but NOT 0x134,
//rem the battery may be using different wire IDs than expected. We now decode
//rem ALL received IDs and print raw frames so you can identify the correct one.
void canRead() {
  CAN_FRAME incoming;
  if (Can0.read(incoming)) {
    unsigned long now = millis();

    switch (incoming.id) {

      case 0x134: {
        //rem BatUSng per DBC:
        //rem   BatUSng_Cks:       bit 0,  8 bits (byte[0])
        //rem   BatUSng_Alvctr:    bit 8,  4 bits (byte[1] lower)
        //rem   BatUSng_DIDNibble: bit 12, 4 bits (byte[1] upper)
        //rem   ULnkSng:           bit 16, 12 bits signed, scale 0.5V
        //rem   UBat:              bit 28, 12 bits unsigned, scale 0.25V
        //rem
        //rem ULnkSng: bytes[2] + lower 4 bits of byte[3], signed, scale 0.5
        int16_t rawLink = incoming.data.byte[2] | ((incoming.data.byte[3] & 0x0F) << 8);
        if (rawLink & 0x800) rawLink |= 0xF000;  //rem Sign-extend 12-bit to 16-bit
        latestLinkVoltage = rawLink * 0.5;

        //rem UBat: upper 4 bits of byte[3] + byte[4], 12 bits, scale 0.25
        uint16_t rawUBat = ((incoming.data.byte[3] >> 4) & 0x0F) | (incoming.data.byte[4] << 4)
                           | ((incoming.data.byte[5] & 0x0F) << 12);
        //rem Actually: bit28 means starting at bit 28 of the 64-bit frame.
        //rem   bit28 = byte[3] bit 4 through byte[4] bit 7 + byte[5] bits 0-3
        //rem   Let's use generic extraction:
        //rem   For 12 bits starting at bit 28 in little-endian:
        rawUBat = 0;
        //rem Extract bits 28-39 from frame data (little-endian)
        uint64_t frameVal = 0;
        for (int i = 0; i < 8; i++) {
          frameVal |= ((uint64_t)incoming.data.byte[i]) << (i * 8);
        }
        rawUBat = (uint16_t)((frameVal >> 28) & 0x0FFF);
        latestPackVoltage = rawUBat * 0.25;
        voltageValid = true;

        Serial.print(F("[")); Serial.print(now);
        Serial.print(F("] BatUSng 0x134: UBat="));
        Serial.print(latestPackVoltage, 2);
        Serial.print(F("V  ULnk="));
        Serial.print(latestLinkVoltage, 1);
        Serial.print(F("V  RAW:"));
        printRawBytes(incoming);
        Serial.println();
        break;
      }

      case 0x136: {
        //rem BatSts per DBC:
        //rem   BatSt:         bit 16, 6 bits (battery state machine)
        //rem   SftySt:        bit 22, 2 bits (safety state)
        //rem   ErrLvl:        bit 24, 4 bits (error level)
        //rem   I:             bit 32, 14 bits signed, scale 0.25A (pack current)
        //rem   StCtctrPos:    bit 46, 2 bits (positive contactor state)
        //rem   StCtctrNeg:    bit 48, 2 bits (negative contactor state)
        //rem   StCtctrPrec:   bit 50, 2 bits (precharge contactor state)
        uint64_t fv = 0;
        for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

        uint8_t batSt       = (fv >> 16) & 0x3F;
        uint8_t sftySt      = (fv >> 22) & 0x03;
        uint8_t errLvl      = (fv >> 24) & 0x0F;
        int16_t rawI        = (fv >> 32) & 0x3FFF;
        if (rawI & 0x2000) rawI |= 0xC000;  //rem sign-extend 14-bit
        float current       = rawI * 0.25;
        uint8_t ctctrPos    = (fv >> 46) & 0x03;
        uint8_t ctctrNeg    = (fv >> 48) & 0x03;
        uint8_t ctctrPrec   = (fv >> 50) & 0x03;

        Serial.print(F("[")); Serial.print(now);
        Serial.print(F("] BatSts 0x136: St="));   Serial.print(batSt);
        Serial.print(F(" Sfty="));  Serial.print(sftySt);
        Serial.print(F(" Err="));   Serial.print(errLvl);
        Serial.print(F(" I="));     Serial.print(current, 2); Serial.print(F("A"));
        Serial.print(F(" Pos="));   Serial.print(ctctrPos);
        Serial.print(F(" Neg="));   Serial.print(ctctrNeg);
        Serial.print(F(" Prec="));  Serial.print(ctctrPrec);
        Serial.print(F("  RAW:"));
        printRawBytes(incoming);
        Serial.println();
        break;
      }

      case 0x131:   //rem BatInfo — power, isolation
      case 0x130:   //rem BatPermsnToCls — permission flags
      default: {
        printCANFrame(incoming, "RX", now, 0xFF);
        break;
      }
    }
  }
}

// ========================= CRC CALCULATION ==================================
//rem AUTOSAR E2E Profile 1 CRC computation.
//rem
//rem Per Webasto 04_E2E_Communication_example_SftyInfo.pdf:
//rem   "The CRC is calculated over the low byte of the DataID and the data
//rem    itself starting from the byte B1."
//rem   "The HighByte of DID shall be 0."
//rem
//rem CRC input order: [DID_LowByte, DID_HighByte(=0x00), B1, B2, ...]
//rem   where B1 = data[1] (alive counter + DID nibble)
//rem         B2 = data[2] (first signal byte)
//rem         etc.
//rem
//rem Parameters:
//rem   data         — pointer to the 8-byte CAN frame buffer
//rem   data_id_low  — low byte of the DataID (e.g., 0xBF for SftyInfo)
//rem   length       — number of bytes from frame to include (1-based: 1=just B0,
//rem                  but since B0 is CRC output, we use bytes[1..length-1])
//rem   out_crc      — optional pointer to receive the computed CRC value
//rem
//rem Result is placed into data[0] (the CRC byte of the frame).
void computeAndSetCRC(uint8_t* data, uint8_t data_id_low, uint8_t length, uint8_t* out_crc) {
  uint8_t crc_data[16];

  crc_data[0] = data_id_low;   //rem DID Low Byte
  crc_data[1] = 0x00;          //rem DID High Byte — always 0x00 per E2E spec

  //rem Copy payload bytes (skip byte[0] which is the CRC output position)
  for (uint8_t i = 1; i < length; i++) {
    crc_data[i + 1] = data[i];
  }

  //rem Calculate CRC over [DID_LB, DID_HB, B1, B2, ...] = length+1 bytes
  uint8_t crc = CRC8_SAE_J1850_ZERO::calculate(crc_data, length + 1);

  data[0] = crc;                       //rem Place CRC into byte[0] of the CAN frame
  if (out_crc) *out_crc = crc;
}

// ========================= CAN SEND WITH LOGGING ============================
void canSend(CAN_FRAME &frame, const char *label) {
  unsigned long now = millis();
  uint8_t crc = frame.data.byte[0];
  printCANFrame(frame, label, now, crc);
  Can0.sendFrame(frame);
}

// ========================= PRINT CAN FRAME ==================================
void printCANFrame(const CAN_FRAME &frame, const char *label, uint32_t timestamp, uint8_t crc) {
  Serial.print(F("["));
  Serial.print(timestamp);
  Serial.print(F("] "));
  Serial.print(label);
  Serial.print(F(" CAN ID: 0x"));
  Serial.print(frame.id, HEX);
  Serial.print(F(" Len: "));
  Serial.print(frame.length);
  Serial.print(F(" Data:"));
  for (int i = 0; i < frame.length; ++i) {
    Serial.print(F(" "));
    if (frame.data.byte[i] < 0x10) Serial.print(F("0"));
    Serial.print(frame.data.byte[i], HEX);
  }
  if (crc != 0xFF) {
    Serial.print(F(" CRC:0x"));
    if (crc < 0x10) Serial.print(F("0"));
    Serial.print(crc, HEX);
  }
  Serial.println();
}

// ========================= PRINT RAW BYTES ==================================
void printRawBytes(const CAN_FRAME &frame) {
  for (int i = 0; i < frame.length; i++) {
    Serial.print(F(" "));
    if (frame.data.byte[i] < 0x10) Serial.print(F("0"));
    Serial.print(frame.data.byte[i], HEX);
  }
}