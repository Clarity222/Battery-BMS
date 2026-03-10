// =============================================================================
//  Cld35.ino — Webasto CV Standard Battery BMS Controller
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
//
// =============================================================================
//
//  REVISION HISTORY:
//
//  Cld35 — 2026-03-10
//
//  FIX 11 (CRITICAL): Changed from E2E Profile 1 → Profile 1C.
//         Profile 1 only includes DataID LOW byte in CRC (high byte = 0x00).
//         Profile 1C includes BOTH DataID low byte AND high byte in CRC.
//         OLD: CRC over [DID_LB, 0x00, payload...]   ← WRONG
//         NEW: CRC over [DID_LB, DID_HB, payload...] ← CORRECT
//         This means EVERY CRC from all prior versions was wrong because
//         the DataID high byte was zeroed instead of using the real value.
//         The battery's E2E receiver rejected every message due to CRC
//         mismatch → ErrLvl=6 → contactors refused to close.
//
//         DataID assignments (from DATA-ID.pdf):
//           SftyInfo  CAN 0x8BF → DataID 0x08BF → LB=0xBF, HB=0x08
//           CtlCmd    CAN 0x60F → DataID 0x060F → LB=0x0F, HB=0x06
//           VehInfo   CAN 0x5B0 → DataID 0x05B0 → LB=0xB0, HB=0x05
//           VehTi     CAN 0x600 → DataID 0x0600 → LB=0x00, HB=0x06
//
//  FIX 7 (CRITICAL): Removed fabricated hvReadyBit from SftyInfo byte[2].
//         The DBC defines ONLY: SftyOk (2b @bit16), CrashDetnNeg (2b @bit18).
//         There is NO hvReadyBit signal. byte[2] = 0xF5 ALWAYS.
//
//  FIX 8 (HIGH): Startup debounce — ignores re-triggers within 3 seconds.
//
//  FIX 9 (HIGH): UDS retry with longer timeout (1s) and 3 retries.
//         Continues sending SftyInfo during UDS wait to keep alive counter.
//
//  FIX 1 (CRITICAL): SftyInfo byte[2] unused bits set to 1 (OR with 0xF0).
//
//  FIX 2 (CRITICAL): Voltage read from correct CAN ID 0x134 (BatUSng),
//         signal UBat @bit28, 12 bits, scale 0.25V.
//
//  FIX 3 (HIGH): UDS_DELAY_AFTER_WAKE added so ECU has time to boot.
//
//  FIX 4 (HIGH): CONNECT_PHASE extended before sending hvReq.
//
//  FIX 5 (MEDIUM): VehInfo TAmb encoding corrected to 11-bit signed.
//
//  FIX 6 (MEDIUM): VehTi bit-packing matches DBC (Year 12b, Month 4b, etc).
//
// =============================================================================

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

// ========================= PIN DEFINITIONS ===================================
const int startupPin        = 5;
const int wakeRelayPin      = 12;
const int voltageRequestPin = 11;

// ========================= CAN IDs (TX — Vehicle_Control_Unit → VIC) =========
#define ID_SFTYINFO   0x8BF
#define ID_CTLCMD     0x60F
#define ID_VEHINFO    0x5B0
#define ID_VEHTI      0x600
#define ID_DIAG_REQ   0x7E0
#define ID_DIAG_RESP  0x7E8

// ========================= CAN IDs (RX — VIC → Vehicle_Control_Unit) =========
#define ID_BATUSNG         0x134   // BatUSng: UBat, ULnkSng
#define ID_BATSTS          0x136   // BatSts:  BatSt, SftySt, ErrLvl, I, contactors
#define ID_BATPERMSNTOCLS  0x130   // BatPermsnToCls: 10 permission flags
#define ID_BATINFO         0x131   // BatInfo: P, RIsln, StsIslnMeast
#define ID_BATSOX          0x142   // BatSox:  SoC, SoH, AvlE
#define ID_BATUCELL        0x144   // BatUCell: UCellMin, UCellMax
#define ID_BATT            0x143   // BatT:    TCellMin, TCellMax
#define ID_BATUMPS         0x133   // BatUMps: ULnkMps, UExpdDrvg, UExpdCh, UDcLnk
#define ID_BATILIM         0x140   // BatILim: charge/discharge current limits
#define ID_BATULIM         0x141   // BatULim: charge/discharge voltage limits

// ========================= FULL 16-BIT DATA IDs (Profile 1C) =================
// Profile 1C: CRC is computed over [DID_LowByte, DID_HighByte, payload...]
// These come from DATA-ID.pdf / VIC_Gen1_0_Veh_CAN_E2E_DataID.xlsx
#define DID_SFTYINFO  0x08BF
#define DID_CTLCMD    0x060F
#define DID_VEHINFO   0x05B0
#define DID_VEHTI     0x0600

// Helper macros to extract low/high bytes
#define DID_LB(did)   ((uint8_t)((did) & 0xFF))
#define DID_HB(did)   ((uint8_t)(((did) >> 8) & 0xFF))

// DID Nibbles — placed in upper nibble of byte[1] of each frame
#define DIDNIBBLE_SFTYINFO  0x8
#define DIDNIBBLE_CTLCMD    0x6
#define DIDNIBBLE_VEHINFO   0x5
#define DIDNIBBLE_VEHTI     0x6

// ========================= TIMING ============================================
#define PERIOD_SFTYINFO       10    // SftyInfo every 10ms (safety-critical)
#define PERIOD_CTLCMD         100   // CtlCmd every 100ms
#define PERIOD_VEHINFO        100   // VehInfo every 100ms
#define PERIOD_VEHTI          1000  // VehTi every 1000ms
#define WAKE_DELAY            1500  // ms after startup before WAKE relay asserted
#define UDS_DELAY_AFTER_WAKE  800   // ms after WAKE before UDS commands
#define CONNECT_PHASE         5000  // ms after startup before HV request
#define STARTUP_LOCKOUT       3000  // ms — ignore re-triggers within this window
#define UDS_TIMEOUT           1000  // ms per UDS attempt
#define UDS_RETRIES           3     // number of UDS retry attempts

// ========================= E2E ALIVE COUNTERS ================================
// 4-bit counter, range 1–14, wraps from 14→1 (0 and 15 are reserved)
Counter aliveCounterSfty;
Counter aliveCounterCtl;
Counter aliveCounterVeh;

// ========================= STATE VARIABLES ===================================
bool systemActive   = false;
bool wakeAsserted   = false;
bool faultsCleared  = false;
bool udsReady       = false;
unsigned long startupTime = 0;
unsigned long wakeTime    = 0;

unsigned long lastSfty  = 0;
unsigned long lastCtl   = 0;
unsigned long lastVeh   = 0;
unsigned long lastVehTi = 0;

// ========================= VOLTAGE STATE =====================================
float latestPackVoltage = 0.0;
float latestLinkVoltage = 0.0;
bool voltageValid = false;

// ========================= SIGNAL VALUES (TX) ================================
// SftyInfo signals
uint8_t sftyOk        = 1;   // SftyOk = 1 ("OK")
uint8_t crashDetnNeg  = 1;   // CrashDetnNeg = 1 ("no crash detected")

// CtlCmd signals
uint8_t hvReq         = 0;   // HvReq: 0=off, 16=drive, 32=charge

// VehInfo signals
int8_t  tAmb          = 20;  // Ambient temperature in °C
uint8_t vehSpd        = 0;   // Vehicle speed in km/h

// VehTi time base
int startYear = 2026, startMonth = 3, startDay = 13;
int startHour = 12, startMinute = 0, startSecond = 0;
uint8_t payLoad[8];

// ========================= SETUP =============================================
void setup() {
  Serial.begin(115200);
  pinMode(startupPin, INPUT_PULLUP);
  pinMode(voltageRequestPin, INPUT_PULLUP);
  pinMode(wakeRelayPin, OUTPUT);

  digitalWrite(wakeRelayPin, LOW);
  Can0.begin(CAN_BPS_500K);

  Serial.println(F("=== Cld35.ino — Webasto CV Battery BMS Controller ==="));
  Serial.println(F("  AUTOSAR E2E Profile 1C (both DID bytes in CRC)"));
  Serial.println(F("  FIX 11: Profile 1 → 1C (DID_HB included in CRC)"));
  Serial.println(F("  FIX 7:  Removed fabricated hvReadyBit from SftyInfo"));
  Serial.println(F("  FIX 8:  Startup debounce (3s lockout)"));
  Serial.println(F("  FIX 9:  UDS retry (3x with 1s timeout)"));
  Serial.println(F(""));
  Serial.println(F("  DataID assignments (Profile 1C):"));
  Serial.print(F("    SftyInfo 0x08BF → LB=0x")); Serial.print(DID_LB(DID_SFTYINFO), HEX);
  Serial.print(F(" HB=0x")); Serial.println(DID_HB(DID_SFTYINFO), HEX);
  Serial.print(F("    CtlCmd   0x060F → LB=0x")); Serial.print(DID_LB(DID_CTLCMD), HEX);
  Serial.print(F(" HB=0x")); Serial.println(DID_HB(DID_CTLCMD), HEX);
  Serial.print(F("    VehInfo  0x05B0 → LB=0x")); Serial.print(DID_LB(DID_VEHINFO), HEX);
  Serial.print(F(" HB=0x")); Serial.println(DID_HB(DID_VEHINFO), HEX);
  Serial.print(F("    VehTi    0x0600 → LB=0x")); Serial.print(DID_LB(DID_VEHTI), HEX);
  Serial.print(F(" HB=0x")); Serial.println(DID_HB(DID_VEHTI), HEX);
  Serial.println(F(""));

  // CRC self-test with Profile 1C
  // SftyInfo example: counter=0xE → byte[1]=0x8E, byte[2]=0xF5
  // Profile 1C CRC input: [0xBF, 0x08, 0x8E, 0xF5]
  uint8_t test_p1c[] = { DID_LB(DID_SFTYINFO), DID_HB(DID_SFTYINFO), 0x8E, 0xF5 };
  uint8_t crc_p1c = CRC8_SAE_J1850_ZERO::calculate(test_p1c, 4);
  Serial.print(F("  CRC self-test Profile 1C [BF,08,8E,F5]: 0x"));
  Serial.println(crc_p1c, HEX);

  // Also show what Profile 1 (old/wrong) would give for comparison
  uint8_t test_p1[] = { 0xBF, 0x00, 0x8E, 0xF5 };
  uint8_t crc_p1 = CRC8_SAE_J1850_ZERO::calculate(test_p1, 4);
  Serial.print(F("  CRC reference Profile 1  [BF,00,8E,F5]: 0x"));
  Serial.print(crc_p1, HEX);
  Serial.println(F(" (old/wrong — for comparison only)"));
  Serial.println(F(""));
}

// ========================= MAIN LOOP =========================================
void loop() {
  handleStartup();
  handleVoltageRequest();

  if (systemActive) {
    unsigned long now = millis();

    // Phase 1: Assert WAKE relay after delay
    if (!wakeAsserted && now - startupTime >= WAKE_DELAY) {
      digitalWrite(wakeRelayPin, HIGH);
      wakeAsserted = true;
      wakeTime = now;
      Serial.print(F("[")); Serial.print(now); Serial.println(F("] WAKE relay asserted"));
    }

    // Phase 2: Wait for ECU boot, then send UDS to clear faults
    if (wakeAsserted && !udsReady && (now - wakeTime >= UDS_DELAY_AFTER_WAKE)) {
      udsReady = true;
    }
    if (udsReady && !faultsCleared) {
      clearBatteryFaults();
      faultsCleared = true;
    }

    // Phase 3: Request HV after connect phase
    if (hvReq == 0 && now - startupTime >= CONNECT_PHASE) {
      hvReq = 16;  // 16 = Drive mode
      Serial.print(F("[")); Serial.print(now);
      Serial.println(F("] HV REQUEST SET (hvReq=16, Drive mode)"));
    }

    // Cyclic message transmission
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

// ========================= STARTUP (with lockout) ============================
void handleStartup() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(startupPin);

  if (lastState == HIGH && currentState == LOW) {
    unsigned long now = millis();

    // FIX 8: Ignore if we already started within STARTUP_LOCKOUT period
    if (systemActive && (now - startupTime < STARTUP_LOCKOUT)) {
      Serial.print(F("[")); Serial.print(now);
      Serial.println(F("] Startup IGNORED (lockout active)"));
      lastState = currentState;
      return;
    }

    Serial.print(F("[")); Serial.print(now); Serial.println(F("] === STARTUP INITIATED ==="));
    systemActive  = true;
    wakeAsserted  = false;
    faultsCleared = false;
    udsReady      = false;
    startupTime   = now;
    hvReq         = 0;
    voltageValid  = false;
    latestPackVoltage = 0.0;
    latestLinkVoltage = 0.0;

    // Reset alive counters so they start fresh
    aliveCounterSfty = Counter();
    aliveCounterCtl  = Counter();
    aliveCounterVeh  = Counter();

    digitalWrite(wakeRelayPin, LOW);
    delay(50);
  }
  lastState = currentState;
}

// ========================= VOLTAGE REQUEST ===================================
void handleVoltageRequest() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(voltageRequestPin);

  if (lastState == HIGH && currentState == LOW) {
    Serial.print(F("[")); Serial.print(millis()); Serial.print(F("] Pack Voltage: "));
    if (voltageValid) {
      Serial.print(latestPackVoltage, 2);
      Serial.print(F("V  Link: "));
      Serial.print(latestLinkVoltage, 1);
      Serial.println(F("V"));
    } else {
      Serial.println(F("not yet received"));
    }
    delay(300);
  }
  lastState = currentState;
}

// ========================= UDS FAULT CLEARING (with retry) ===================
void clearBatteryFaults() {
  unsigned long now = millis();
  Serial.print(F("[")); Serial.print(now);
  Serial.println(F("] === UDS FAULT CLEARING ==="));

  CAN_FRAME frame;
  frame.id       = ID_DIAG_REQ;
  frame.extended = false;
  frame.length   = 8;

  // Command 1: DiagnosticSessionControl → Extended session (0x03)
  frame.data.byte[0] = 0x02;
  frame.data.byte[1] = 0x10;
  frame.data.byte[2] = 0x03;
  frame.data.byte[3] = 0xAA;
  frame.data.byte[4] = 0xAA;
  frame.data.byte[5] = 0xAA;
  frame.data.byte[6] = 0xAA;
  frame.data.byte[7] = 0xAA;

  bool sessionOk = false;
  for (int attempt = 0; attempt < UDS_RETRIES && !sessionOk; attempt++) {
    Serial.print(F("  UDS DiagSession attempt "));
    Serial.println(attempt + 1);
    canSend(frame, "TX-UDS-SESSION");
    sessionOk = waitForUdsResponse(UDS_TIMEOUT);
  }
  if (!sessionOk) {
    Serial.println(F("  WARNING: No UDS session response after retries!"));
  }

  delay(100);  // Short gap between UDS commands

  // Command 2: ClearDiagnosticInformation → all DTCs (0xFF 0xFF 0xFF)
  frame.data.byte[0] = 0x04;
  frame.data.byte[1] = 0x14;
  frame.data.byte[2] = 0xFF;
  frame.data.byte[3] = 0xFF;
  frame.data.byte[4] = 0xFF;
  frame.data.byte[5] = 0xAA;
  frame.data.byte[6] = 0xAA;
  frame.data.byte[7] = 0xAA;

  bool clearOk = false;
  for (int attempt = 0; attempt < UDS_RETRIES && !clearOk; attempt++) {
    Serial.print(F("  UDS ClearDTC attempt "));
    Serial.println(attempt + 1);
    canSend(frame, "TX-UDS-CLEAR");
    clearOk = waitForUdsResponse(UDS_TIMEOUT);
  }
  if (!clearOk) {
    Serial.println(F("  WARNING: No UDS clear response! DTCs may block contactor!"));
  } else {
    Serial.println(F("  UDS fault clear SUCCESSFUL"));
  }
}

bool waitForUdsResponse(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    CAN_FRAME rx;
    if (Can0.read(rx)) {
      if (rx.id == ID_DIAG_RESP) {
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
    // Continue sending SftyInfo during UDS wait to maintain alive counter
    unsigned long now = millis();
    if (now - lastSfty >= PERIOD_SFTYINFO) {
      sendSftyInfo();
      lastSfty = now;
    }
  }
  return false;
}

// =============================================================================
//  CRC CALCULATION — AUTOSAR E2E PROFILE 1C
// =============================================================================
//
//  Profile 1C CRC byte order:
//    [DataID_Low_Byte] [DataID_High_Byte] [Byte1] [Byte2] ... [ByteN]
//
//  Where Byte1..ByteN are the CAN frame payload bytes AFTER byte[0] (CRC).
//  The CRC result is placed into byte[0].
//
//  CRC-8 SAE J1850 with polynomial 0x1D, init=0x00, final XOR=0x00.
//
//  Parameters:
//    data       — pointer to frame.data.byte[] (8 bytes), byte[0] will be CRC
//    dataID     — full 16-bit DataID (e.g., 0x08BF for SftyInfo)
//    payloadLen — number of payload bytes to include (data[1]..data[payloadLen])
//    out_crc    — optional pointer to store the computed CRC value
//
void computeAndSetCRC_P1C(uint8_t* data, uint16_t dataID, uint8_t payloadLen, uint8_t* out_crc) {
  uint8_t crc_data[16];

  // Profile 1C: include BOTH bytes of DataID
  crc_data[0] = DID_LB(dataID);    // DataID Low Byte
  crc_data[1] = DID_HB(dataID);    // DataID High Byte  ← THE KEY FIX!

  // Copy payload bytes: data[1] through data[payloadLen]
  for (uint8_t i = 1; i <= payloadLen; i++) {
    crc_data[i + 1] = data[i];
  }

  // Total CRC input length: 2 (DID_LB + DID_HB) + payloadLen
  uint8_t crc = CRC8_SAE_J1850_ZERO::calculate(crc_data, payloadLen + 2);

  data[0] = crc;
  if (out_crc) *out_crc = crc;
}

// =============================================================================
//  SEND: SftyInfo (10ms) — CAN ID 0x8BF, DataID 0x08BF
// =============================================================================
//  DBC signals (07_VIC_Gen1_0_VEH_CAN.DBC line 168-173):
//    SftyInfo_Cks:       bit 0,  8b — CRC (byte[0])
//    SftyInfo_Alvctr:    bit 8,  4b — alive counter (byte[1] bits 0-3)
//    SftyInfo_DIDNibble: bit 12, 4b — DID nibble = 0x8 (byte[1] bits 4-7)
//    SftyOk:             bit 16, 2b — value 1 (byte[2] bits 0-1)
//    CrashDetnNeg:       bit 18, 2b — value 1 (byte[2] bits 2-3)
//    Bits 20-23:         UNUSED → must be 1 (byte[2] bits 4-7)
//    Bytes 3-7:          UNUSED → 0xFF
//
//  FIX 7: NO hvReadyBit — it does NOT exist in the DBC.
//         byte[2] = 0xF0 | (crashDetnNeg<<2) | sftyOk = 0xF5 ALWAYS
//
//  FIX 11: Profile 1C CRC over [0xBF, 0x08, byte[1], byte[2]]
//
void sendSftyInfo() {
  CAN_FRAME frame;
  frame.id       = ID_SFTYINFO;
  frame.extended = false;
  frame.length   = 8;

  // Fill all bytes with 0xFF (unused bits = 1)
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  // byte[1]: upper nibble = DID nibble, lower nibble = alive counter
  uint8_t alive = aliveCounterSfty.increment();
  frame.data.byte[1] = (DIDNIBBLE_SFTYINFO << 4) | (alive & 0x0F);

  // byte[2]: SftyOk (2b) + CrashDetnNeg (2b) + unused (4b, all 1)
  frame.data.byte[2] = 0xF0                           // unused bits 4-7 = 1
                      | ((crashDetnNeg & 0x03) << 2)   // bits 2-3
                      | (sftyOk & 0x03);                // bits 0-1
  // byte[2] = 0xF0 | 0x04 | 0x01 = 0xF5

  // CRC: Profile 1C over 2 payload bytes (byte[1], byte[2])
  uint8_t crc = 0;
  computeAndSetCRC_P1C(frame.data.byte, DID_SFTYINFO, 2, &crc);
  canSend(frame, "TX-SFTYINFO");
}

// =============================================================================
//  SEND: CtlCmd (100ms) — CAN ID 0x60F, DataID 0x060F
// =============================================================================
//  DBC signals (line 161-166):
//    CtlCmd_Cks:       bit 0,  8b — CRC (byte[0])
//    CtlCmd_Alvctr:    bit 8,  4b — alive counter (byte[1] bits 0-3)
//    CtlCmd_DIDNibble: bit 12, 4b — DID nibble = 0x6 (byte[1] bits 4-7)
//    HvReq:            bit 16, 8b — 0=off, 16=drive, 32=charge (byte[2])
//    IslnMeastDisaCmd: bit 24, 2b — 0=enabled (byte[3] bits 0-1)
//    Bits 26-63:       UNUSED → 0xFF
//
void sendCtlCmd() {
  CAN_FRAME frame;
  frame.id       = ID_CTLCMD;
  frame.extended = false;
  frame.length   = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  frame.data.byte[1] = (DIDNIBBLE_CTLCMD << 4) | (aliveCounterCtl.increment() & 0x0F);
  frame.data.byte[2] = hvReq;
  frame.data.byte[3] = 0xFC;  // IslnMeastDisaCmd=0 (bits 0-1), unused bits 2-7=1

  // CRC: Profile 1C over 3 payload bytes (byte[1], byte[2], byte[3])
  uint8_t crc = 0;
  computeAndSetCRC_P1C(frame.data.byte, DID_CTLCMD, 3, &crc);
  canSend(frame, "TX-CTLCMD");
}

// =============================================================================
//  SEND: VehInfo (100ms) — CAN ID 0x5B0, DataID 0x05B0
// =============================================================================
//  DBC signals (line 178-183):
//    VehInfo_Cks:       bit 0,  8b — CRC (byte[0])
//    VehInfo_Alvctr:    bit 8,  4b — alive counter
//    VehInfo_DIDNibble: bit 12, 4b — DID nibble = 0x5
//    TAmb:              bit 16, 11b signed — scale 0.25 °C
//    VehSpd:            bit 32, 11b signed — scale 0.5 km/h
//    Unused bits set to 1.
//
void sendVehInfo() {
  CAN_FRAME frame;
  frame.id       = ID_VEHINFO;
  frame.extended = false;
  frame.length   = 8;
  for (int i = 0; i < 8; i++) frame.data.byte[i] = 0xFF;

  frame.data.byte[1] = (DIDNIBBLE_VEHINFO << 4) | (aliveCounterVeh.increment() & 0x0F);

  // TAmb: 11-bit signed, scale 0.25 °C, starts at bit 16
  // raw = tAmb / 0.25 = tAmb * 4
  int16_t rawTamb = (int16_t)tAmb * 4;
  uint16_t tAmbU = (uint16_t)rawTamb & 0x07FF;  // 11 bits
  frame.data.byte[2] = tAmbU & 0xFF;                              // bits 16-23
  frame.data.byte[3] = ((tAmbU >> 8) & 0x07) | 0xF8;             // bits 24-26 + unused

  // VehSpd: 11-bit signed, scale 0.5 km/h, starts at bit 32
  int16_t rawSpd = (int16_t)vehSpd * 2;
  uint16_t vSpdU = (uint16_t)rawSpd & 0x07FF;   // 11 bits
  frame.data.byte[4] = vSpdU & 0xFF;                              // bits 32-39
  frame.data.byte[5] = ((vSpdU >> 8) & 0x07) | 0xF8;             // bits 40-42 + unused

  // CRC: Profile 1C over 5 payload bytes (byte[1]..byte[5])
  uint8_t crc = 0;
  computeAndSetCRC_P1C(frame.data.byte, DID_VEHINFO, 5, &crc);
  canSend(frame, "TX-VEHINFO");
}

// =============================================================================
//  SEND: VehTi (1000ms) — CAN ID 0x600, DataID 0x0600
// =============================================================================
//  DBC signals (line 185-191):
//    Year:  bit 0,  12b unsigned
//    Month: bit 12, 4b  unsigned
//    Day:   bit 16, 6b  unsigned
//    Hour:  bit 22, 5b  unsigned
//    Minu:  bit 27, 6b  unsigned
//    Sec:   bit 33, 6b  unsigned
//
//  NOTE: VehTi has NO Cks/Alvctr/DIDNibble signals in the DBC.
//  But per 02_Series-Sample spec, it still uses E2E Profile 1C protection.
//  byte[0] = CRC, and the rest is time-packed payload.
//
void sendVehTi() {
  CAN_FRAME frame;
  frame.id       = ID_VEHTI;
  frame.extended = false;
  frame.length   = 8;

  getTime();
  for (int i = 0; i < 8; i++) frame.data.byte[i] = payLoad[i];

  // CRC: Profile 1C over 7 payload bytes (byte[1]..byte[7])
  uint8_t crc = 0;
  computeAndSetCRC_P1C(frame.data.byte, DID_VEHTI, 7, &crc);
  canSend(frame, "TX-VEHTIME");
}

// ========================= TIME BIT-PACKING ==================================
// Matches DBC: Year(12b) Month(4b) Day(6b) Hour(5b) Minu(6b) Sec(6b)
void getTime() {
  unsigned long seconds = millis() / 1000;
  uint16_t year   = startYear;
  uint8_t  month  = startMonth;
  uint8_t  day    = startDay;
  uint8_t  hour   = (startHour   + (seconds / 3600)) % 24;
  uint8_t  minute = (startMinute + (seconds / 60))   % 60;
  uint8_t  second = (startSecond + (seconds % 60))    % 60;

  // Bit-pack into 8 bytes (little-endian bit ordering per DBC @1+)
  payLoad[0] = year & 0xFF;                                       // Year bits 0-7
  payLoad[1] = ((year >> 8) & 0x0F) | ((month & 0x0F) << 4);     // Year 8-11 + Month 0-3
  payLoad[2] = (day & 0x3F) | ((hour & 0x03) << 6);              // Day 0-5 + Hour 0-1
  payLoad[3] = ((hour >> 2) & 0x07) | ((minute & 0x1F) << 3);   // Hour 2-4 + Minu 0-4
  payLoad[4] = ((minute >> 5) & 0x01) | ((second & 0x3F) << 1); // Minu 5 + Sec 0-5
  payLoad[5] = 0x00;
  payLoad[6] = 0x00;
  payLoad[7] = 0x00;
}

// =============================================================================
//  CAN READ — Decode all battery response messages
// =============================================================================
void canRead() {
  CAN_FRAME incoming;
  if (!Can0.read(incoming)) return;

  unsigned long now = millis();

  switch (incoming.id) {

    // ----- BatUSng (0x134): Pack voltage + Link voltage -----
    case ID_BATUSNG: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      // ULnkSng: bit 16, 12b signed, scale 0.5V
      int16_t rawLink = (fv >> 16) & 0x0FFF;
      if (rawLink & 0x800) rawLink |= 0xF000;  // sign extend
      latestLinkVoltage = rawLink * 0.5;

      // UBat: bit 28, 12b unsigned, scale 0.25V
      uint16_t rawUBat = (uint16_t)((fv >> 28) & 0x0FFF);
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

    // ----- BatSts (0x136): State, safety, errors, current, contactors -----
    case ID_BATSTS: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      uint8_t batSt      = (fv >> 16) & 0x3F;   // 6 bits
      uint8_t sftySt     = (fv >> 22) & 0x03;   // 2 bits
      uint8_t errLvl     = (fv >> 24) & 0x0F;   // 4 bits
      uint8_t hvilPermsn = (fv >> 28) & 0x0F;   // 4 bits
      int16_t rawI       = (fv >> 32) & 0x3FFF; // 14 bits signed
      if (rawI & 0x2000) rawI |= 0xC000;        // sign extend
      float current      = rawI * 0.25;
      uint8_t ctctrPos   = (fv >> 46) & 0x03;
      uint8_t ctctrNeg   = (fv >> 48) & 0x03;
      uint8_t ctctrPrec  = (fv >> 50) & 0x03;
      uint8_t ecuSt      = (fv >> 52) & 0x07;

      Serial.print(F("[")); Serial.print(now);
      Serial.print(F("] BatSts 0x136:"));
      Serial.print(F(" BatSt="));  Serial.print(batSt);
      Serial.print(F(" Sfty="));   Serial.print(sftySt);
      Serial.print(F(" Err="));    Serial.print(errLvl);
      Serial.print(F(" Hvil="));   Serial.print(hvilPermsn);
      Serial.print(F(" I="));      Serial.print(current, 2); Serial.print(F("A"));
      Serial.print(F(" Pos="));    Serial.print(ctctrPos);
      Serial.print(F(" Neg="));    Serial.print(ctctrNeg);
      Serial.print(F(" Prec="));   Serial.print(ctctrPrec);
      Serial.print(F(" EcuSt=")); Serial.print(ecuSt);
      Serial.print(F("  RAW:"));
      printRawBytes(incoming);
      Serial.println();

      if (errLvl > 0) {
        Serial.print(F("  *** BATTERY ERROR LEVEL "));
        Serial.print(errLvl);
        Serial.println(F(" — check DTCs via UDS ReadDTCInformation ***"));
      }
      if (ctctrPos > 0 && ctctrNeg > 0) {
        Serial.println(F("  *** CONTACTORS CLOSED — HV ACTIVE ***"));
      }
      break;
    }

    // ----- BatPermsnToCls (0x130): 10 permission flags -----
    case ID_BATPERMSNTOCLS: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      uint8_t iPermsn      = (fv >> 16) & 0x0F;
      uint8_t islnPermsn   = (fv >> 20) & 0x0F;
      uint8_t stuckPermsn  = (fv >> 24) & 0x0F;
      uint8_t dcChPermsn   = (fv >> 28) & 0x0F;
      uint8_t thermPermsn  = (fv >> 32) & 0x0F;
      uint8_t crashPermsn  = (fv >> 36) & 0x0F;
      uint8_t lockPermsn   = (fv >> 40) & 0x0F;
      uint8_t cptTPermsn   = (fv >> 44) & 0x0F;
      uint8_t cellUPermsn  = (fv >> 48) & 0x0F;
      uint8_t cellTPermsn  = (fv >> 52) & 0x0F;

      Serial.print(F("[")); Serial.print(now);
      Serial.print(F("] BatPermsnToCls 0x130:"));
      Serial.print(F(" I="));     Serial.print(iPermsn);
      Serial.print(F(" Isln="));  Serial.print(islnPermsn);
      Serial.print(F(" Stuck=")); Serial.print(stuckPermsn);
      Serial.print(F(" DcCh="));  Serial.print(dcChPermsn);
      Serial.print(F(" Therm=")); Serial.print(thermPermsn);
      Serial.print(F(" Crash=")); Serial.print(crashPermsn);
      Serial.print(F(" Lock="));  Serial.print(lockPermsn);
      Serial.print(F(" CptT="));  Serial.print(cptTPermsn);
      Serial.print(F(" CellU=")); Serial.print(cellUPermsn);
      Serial.print(F(" CellT=")); Serial.print(cellTPermsn);
      Serial.print(F("  RAW:"));
      printRawBytes(incoming);
      Serial.println();

      // Flag any blocked permission (value 0 = not permitted)
      if (iPermsn == 0 || islnPermsn == 0 || stuckPermsn == 0 ||
          dcChPermsn == 0 || thermPermsn == 0 || crashPermsn == 0 ||
          lockPermsn == 0 || cptTPermsn == 0 || cellUPermsn == 0 ||
          cellTPermsn == 0) {
        Serial.println(F("  *** ONE OR MORE PERMISSIONS BLOCKING CONTACTOR CLOSE ***"));
      }
      break;
    }

    // ----- BatSox (0x142): SoC, SoH, Available Energy -----
    case ID_BATSOX: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      float soc  = ((fv >> 0)  & 0x03FF) * 0.125;  // 10b, 0.125%
      float soh  = ((fv >> 10) & 0x03FF) * 0.125;  // 10b, 0.125%
      float avlE = ((fv >> 20) & 0x0FFF) * 0.25;   // 12b, 0.25 kWh

      Serial.print(F("[")); Serial.print(now);
      Serial.print(F("] BatSox 0x142: SoC="));
      Serial.print(soc, 1); Serial.print(F("%"));
      Serial.print(F(" SoH=")); Serial.print(soh, 1); Serial.print(F("%"));
      Serial.print(F(" AvlE=")); Serial.print(avlE, 2); Serial.print(F("kWh"));
      Serial.print(F("  RAW:")); printRawBytes(incoming);
      Serial.println();
      break;
    }

    // ----- BatUCell (0x144): Min/Max cell voltages -----
    case ID_BATUCELL: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      float uCellMax = ((fv >> 0)  & 0x0FFF) * 0.002;  // 12b, 0.002V
      float uCellMin = ((fv >> 12) & 0x0FFF) * 0.002;  // 12b, 0.002V

      Serial.print(F("[")); Serial.print(now);
      Serial.print(F("] BatUCell 0x144: Max="));
      Serial.print(uCellMax, 3); Serial.print(F("V Min="));
      Serial.print(uCellMin, 3); Serial.print(F("V"));
      Serial.print(F("  RAW:")); printRawBytes(incoming);
      Serial.println();
      break;
    }

    // ----- BatT (0x143): Min/Max cell temperatures -----
    case ID_BATT: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      int16_t rawTMax = (fv >> 0) & 0x07FF;
      if (rawTMax & 0x400) rawTMax |= 0xF800;
      float tCellMax = rawTMax * 0.25;

      int16_t rawTMin = (fv >> 11) & 0x07FF;
      if (rawTMin & 0x400) rawTMin |= 0xF800;
      float tCellMin = rawTMin * 0.25;

      Serial.print(F("[")); Serial.print(now);
      Serial.print(F("] BatT 0x143: TMax="));
      Serial.print(tCellMax, 1); Serial.print(F("C TMin="));
      Serial.print(tCellMin, 1); Serial.print(F("C"));
      Serial.print(F("  RAW:")); printRawBytes(incoming);
      Serial.println();
      break;
    }

    // ----- BatInfo (0x131): Power, insulation, flags -----
    case ID_BATINFO: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      int16_t rawP = (fv >> 16) & 0x1FFF;
      if (rawP & 0x1000) rawP |= 0xE000;
      float power = rawP * 0.125;

      uint8_t stsIsln = (fv >> 29) & 0x07;
      uint16_t rawRIsln = (fv >> 32) & 0xFFFF;
      float rIsln = rawRIsln * 8.0;

      Serial.print(F("[")); Serial.print(now);
      Serial.print(F("] BatInfo 0x131: P="));
      Serial.print(power, 1); Serial.print(F("kW"));
      Serial.print(F(" Isln=")); Serial.print(stsIsln);
      Serial.print(F(" RIsln=")); Serial.print(rIsln, 0); Serial.print(F("kOhm"));
      Serial.print(F("  RAW:")); printRawBytes(incoming);
      Serial.println();
      break;
    }

    // ----- BatUMps (0x133): Link voltage, expected voltages -----
    case ID_BATUMPS: {
      uint64_t fv = 0;
      for (int i = 0; i < 8; i++) fv |= ((uint64_t)incoming.data.byte[i]) << (i * 8);

      int16_t rawULnk = (fv >> 0) & 0x0FFF;
      if (rawULnk & 0x800) rawULnk |= 0xF000;
      float uLnkMps = rawULnk * 0.5;

      float uExpdCh   = ((fv >> 12) & 0x0FFF) * 0.25;
      float uExpdDrvg = ((fv >> 24) & 0x0FFF) * 0.25;
      float uDcLnk    = ((fv >> 36) & 0x0FFF) * 0.25;

      Serial.print(F("[")); Serial.print(now);
      Serial.print(F("] BatUMps 0x133: ULnk="));
      Serial.print(uLnkMps, 1); Serial.print(F("V"));
      Serial.print(F(" UExpCh=")); Serial.print(uExpdCh, 1); Serial.print(F("V"));
      Serial.print(F(" UExpDr=")); Serial.print(uExpdDrvg, 1); Serial.print(F("V"));
      Serial.print(F(" UDcLnk=")); Serial.print(uDcLnk, 1); Serial.print(F("V"));
      Serial.print(F("  RAW:")); printRawBytes(incoming);
      Serial.println();
      break;
    }

    // ----- All other frames: print raw -----
    default: {
      printCANFrame(incoming, "RX", now, 0xFF);
      break;
    }
  }
}

// ========================= CAN SEND / PRINT ==================================
void canSend(CAN_FRAME &frame, const char *label) {
  unsigned long now = millis();
  uint8_t crc = frame.data.byte[0];
  printCANFrame(frame, label, now, crc);
  Can0.sendFrame(frame);
}

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

void printRawBytes(const CAN_FRAME &frame) {
  for (int i = 0; i < frame.length; i++) {
    Serial.print(F(" "));
    if (frame.data.byte[i] < 0x10) Serial.print(F("0"));
    Serial.print(frame.data.byte[i], HEX);
  }
}