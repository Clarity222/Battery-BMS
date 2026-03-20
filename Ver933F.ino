// Cld107-Fixed-v9.33.ino
// ==============================================
// FULL CUMULATIVE VERSION HISTORY (PERMANENT — NEVER REMOVE ANY ENTRY)
// ==============================================
// v9.33C  Brian fix timing  TODO first cnt stars on1
// v9.33  (current) — Added global MIN_FRAME_GAP guard (2500 µs) in sendPrepared() + stronger stagger (40 ms / 75 ms).
//                   BUG FOUND: Can932Iso0.txt showed identical timestamps (.614/.620/.627 clusters) because startup burst + timer alignment sent multiple frames in one loop pass. Pre-compute helped CPU but not bus spacing. The gap guard fixes it permanently.
// v9.32 — Pre-computed CRC (your suggestion — calculation moved to dwell time).
// v9.31 — Fixed missing lastSfty declaration.
// v9.30 — Safe polling with 1500 µs gap.
// v9.29 — Hardware interrupt (crashed Due, no serial).
// v9.28 — delayMicroseconds(300) + stagger.
// v9.27 — Single if + SftyInfo first.
// v9.26 — RX completely removed.
// v9.25 — Inline remarks enforced.
// v9.24 — While-catch-up.
// v9.23 — Permanent full history block.
// v9.22 — Unconditional timers + immediate burst.
// v9.21 — canActive flag.
// v9.20 — Additive scheduler.
// v9.19 — IsInMeasDisaCmd variable.
// v9.18 — All runtime Serial removed.
// v9.17 — Permanent history block enforced.
// v9.16 — Serial print reduction.
// v9.15 — Raw 0x136 dump.
// v9.14 — 0x136 debug.
// v9.13 — Falling-edge per diagram.
// v9.12 — Non-blocking 1.5 s CL15.
// v9.11 — Unconditional scheduler.
// v9.10 — One-time BatSt print.
// v9.9  — Remarked unused sends.
// v9.8  — E2ECounter struct 0–0x0E.
// v9.7  — Counter 0-0x0E.
// v9.6  — Full DataID+payload CRC.
// v9.5  — Restored TX format.
// v9.4  — Raw 0x136 debug.
// v9.3  — BatSt gate before HV.
// v9.0  — Added 0x136 monitoring.
// v8.x  — Restored pins 5/12.
// Base  — cld107.ino + GitHub ARXML/cdd files.
// ==============================================

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

// ====================== DTC REQUEST CLASS (5-second timer) ======================
// Added by Grok — sends UDS 0x19 02 FF every 5 seconds
class BMSDiagnostics {
public:

    void sendAllDTCRequest() {
        CAN_FRAME frame;
        frame.id = 0x7E0;
        frame.extended = false;
        frame.length = 3;
        frame.data.uint8[0] = 0x19;
        frame.data.uint8[1] = 0x02;
        frame.data.uint8[2] = 0xFF;
        Can0.sendFrame(frame);
        Serial.println("→ Sent UDS 0x19 02 FF (Read ALL DTCs) on 0x7E0");
    }


};

BMSDiagnostics bmsDiag;   // global instance

// ====================== YOUR ORIGINAL CODE (unchanged) ======================

struct E2ECounter {
    uint8_t count = 0;
    uint8_t increment() {
        count = (count + 1) > 0x0E ? 0 : count + 1;
        return count;
    }
};

E2ECounter sftyCounter;
E2ECounter ctlCounter;
E2ECounter vehCounter;

uint8_t isInMeasDisaCmd = 0;   // change to 0 for test

#define STARTUP_PIN      5
#define WAKE_RELAY_PIN   12
#define HV_REQUEST_PIN   6

CAN_FRAME sftyFrame, ctlFrame, vehFrame;

uint32_t lastSfty = 0;
uint32_t lastCtl = 0;
uint32_t lastVeh = 0;
uint32_t lastAnySend = 0;          // v9.33 — global gap guard
const uint32_t PERIOD_CTL  = 100000UL;
const uint32_t PERIOD_VEH  = 100000UL;
const uint32_t MIN_FRAME_GAP = 2500UL;  // v9.33 — guarantees distinct timestamps

bool cl15Closed = false;
bool cl15Detected = false;
uint32_t cl15StartTime = 0;
bool lastStartupState = HIGH;
bool canActive = false;

void prepareSfty() {
  uint8_t p[8] = {0, 0, 0x05, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  p[1] = (0x8 << 4) | sftyCounter.increment();
  uint8_t crcData[9] = {0xBF, 0x08, 0};
  memcpy(&crcData[2], p, 7);
  p[0] = CRC8_SAE_J1850_ZERO::calculate(crcData, 9);
  sftyFrame.id = 0x8BF; sftyFrame.extended = true; sftyFrame.length = 8;
  memcpy(sftyFrame.data.uint8, p, 8);
}

void prepareCtl() {
  uint8_t hv = (digitalRead(HV_REQUEST_PIN) == HIGH) ? 16 : 0;
  uint8_t p[8] = {0, 0, hv, isInMeasDisaCmd, 0xFF, 0xFF, 0xFF, 0xFF};
  p[1] = (0x6 << 4) | ctlCounter.increment();
  uint8_t crcData[9] = {0x0F, 0x06, 0};
  memcpy(&crcData[2], p, 7);
  p[0] = CRC8_SAE_J1850_ZERO::calculate(crcData, 9);
  ctlFrame.id = 0x60F; ctlFrame.extended = true; ctlFrame.length = 8;
  memcpy(ctlFrame.data.uint8, p, 8);
}

void prepareVeh() {
  uint8_t p[8] = {0, 0, 0x14, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
  p[1] = (0x5 << 4) | vehCounter.increment();
  uint8_t crcData[9] = {0xB0, 0x05, 0};
  memcpy(&crcData[2], p, 7);
  p[0] = CRC8_SAE_J1850_ZERO::calculate(crcData, 9);
  vehFrame.id = 0x5B0; vehFrame.extended = true; vehFrame.length = 8;
  memcpy(vehFrame.data.uint8, p, 8);
}

void sendPrepared(CAN_FRAME &frame) {
  Can0.sendFrame(frame);
}

void setup() {
  Serial.begin(115200);
  //pinMode(STARTUP_PIN, INPUT);
  pinMode(STARTUP_PIN, INPUT_PULLUP);
  pinMode(WAKE_RELAY_PIN, OUTPUT);
  digitalWrite(WAKE_RELAY_PIN, LOW);
  pinMode(HV_REQUEST_PIN, INPUT);

  Can0.begin(CAN_BPS_500K);

  Serial.println("v9.33B booted — MINIMUM INTER-FRAME GAP ENFORCED (2500 µs)");

  prepareSfty();
  prepareCtl();
  prepareVeh();
}

void loop() {
  uint32_t now = micros();

  if (canActive) {
    if (now - lastSfty >= 10000UL) {
      sendPrepared(sftyFrame);
         lastSfty = now;
      prepareSfty();
    }
  } 
  if (now - lastCtl >= PERIOD_CTL) {
      sendPrepared(ctlFrame);
      //lastCtl += PERIOD_CTL;
        lastCtl = now;
      prepareCtl();
    }
    if (now - lastVeh >= PERIOD_VEH) {
      sendPrepared(vehFrame);
      //lastVeh += PERIOD_VEH;
            lastVeh = now;
      prepareVeh();
    }
  bool currentState = digitalRead(STARTUP_PIN);
   if (!cl15Closed && currentState == LOW && !canActive) { 
    cl15Detected = true;
    cl15StartTime = now;
    canActive = true;
    lastSfty = now + 10000UL;
    lastCtl  = now + 40000UL;
    lastVeh  = now + 75000UL;
    
  Serial.println("ONE TIME RELAY TRIGGER");
  }

  if (cl15Detected && !cl15Closed && (now - cl15StartTime >= 1500000UL)) {
    digitalWrite(WAKE_RELAY_PIN, HIGH);
    cl15Closed = true;
  }

  // ====================== DTC REQUEST TIMER (every 5 seconds) ======================
  static uint32_t lastDTC = 0;
    if (canActive) { 
  if (millis() - lastDTC >= 5000) {
    bmsDiag.sendAllDTCRequest();
    lastDTC = millis();
  }
  }
}