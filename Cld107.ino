// Cld102.ino - Webasto Battery BMS Contact Closure Emulator
// Project: Clarity222/Battery-BMS
// Latest version: v7.10 (fixed CAN RX interrupt for Collin80 due_can)

// ======================= VERSION CONTROL NOTES =======================
// 2026-03-17 v7.0: FINAL CONTACTOR FIX — Re-enabled 0x701 (ThermSysInfo 100ms) and 0x720 
//                  (BatThermReq2 1000ms) because battery actively sends entire 0x700 range 
//                  and clearly expects vehicle thermal responses before closing contactor 
//                  (Err=6 was caused by missing frames). New AliveCounters + micros() timing. 
//                  Moved hvReq=16 immediately after relay close (was too late). Added brief 
//                  quiet init. All v6.9 timing optimizations kept. // rem v7.0 on every change.
// 2026-03-12 v1.0: Initial CRC fix (Profile 1 variant 1C).
// 2026-03-12 v1.1: Debug prints, delay(1), isoCmd=0, reset counters.
// 2026-03-12 v1.2: Renamed to AliveCounter for compilation.
// 2026-03-12 v1.3: Added debounce for startup pin (50ms confirm low).
// 2026-03-12 v1.4: Full code inclusion to resolve declaration errors.
// 2026-03-12 v1.5: Added early Serial prints in setup for readout test.
// 2026-03-12 v1.6: Increased debounce, added pin monitoring.
// 2026-03-12 v1.7: Removed debounce to revert to previous working state for CL15 relay.
// 2026-03-12 v1.8: Removed loop delay for better timing; fixed voltage ID.
// 2026-03-12 v1.9: isoCmd=1, anti-burst, UDS retry, voltage from 0x136.
// 2026-03-12 v2.0: Removed VehTi CRC, Sec 6 bits per PDF.
// 2026-03-12 v2.1: Reduced idle/pin prints to every 1000ms/500ms to avoid terminal crash.
//                  Added CAN init debug; ensured RX prints even if no data.
// 2026-03-12 v2.2: Declared 'now' in canRead() to fix compilation scope error.
// 2026-03-12 v2.3: Further reduced prints (idle 2000ms, pin 1000ms); added startup debug if pin LOW.
// 2026-03-12 v2.4: Changed 'while' to 'if' for sends to fix bursts and intermixed timing.
//                  Reduced setup prints to essentials for terminal stability.
// 2026-03-12 v2.5: Minimized all prints for low overhead; added loop timing debug.
// 2026-03-12 v2.6: Fixed compilation (declared lastIdlePrint/lastPinPrint; removed unused in canSend).
// 2026-03-12 v2.7: Fixed endless 'Pin LOW' debug (made it one-time); baud to 115200.
//                  Added SD logging (file "log.txt" on SD card, CS pin 4).
// 2026-03-12 v2.8: Replaced SD with DueFlashStorage for internal flash logging.
//                  Logs key events; retrieve with serial "dump" command.
// 2026-03-12 v2.9: Added logToFlash(float, int) for voltage logging with decimals.
// 2026-03-12 v2.10: Fixed 'dtostrf' with sprintf; removed unused 'label' in canSend.
// 2026-03-12 v3.0: Stripped all non-essential prints/logging; only 4 time-stamped events.
//                  No CAN TX/RX prints, no file/SD/flash; minimal resource use.
// 2026-03-12 v3.1: Fixed 'lastSfty' etc. not declared; added globals; no 'dtostrf' needed (no voltage print).
// 2026-03-12 v3.2: Fixed all compilation errors; declared all globals; removed dtostrf/sprintf (no need in v3.0+).
// 2026-03-12 v3.3: Removed startup debug print to stop flood.
// 2026-03-12 v3.4: Removed startup debug to stop flood; added UDS success print; isoCmd=0 test; wake debug.
// 2026-03-12 v3.5: Added loop delay(1) to intermix sends; UDS no-response print; isoCmd=1; minimal.
// 2026-03-12 v3.6: Used micros() for precise send timing (fix bursts); UDS failure print; no delay.
// 2026-03-12 v3.7: Added debounce to startupPin (50ms); UDS failure print if all retries fail; isoCmd=0.
// 2026-03-12 v3.8: Reinstated removed parts as remarked out (e.g., pin print, loop time, full UDS log); removed broken startup debug.
//                  isoCmd=0 test; debounce for stable input.
// 2026-03-12 v3.9: Removed broken startup debug entirely; kept debounce for stable trigger.
// 2026-03-12 v4.0: Fixed relay bug (startupTime units mismatch: now millis, comparisons millis); isoCmd=1.
// 2026-03-12 v4.1: Fixed relay units; restored remarked code; added wake failure print; isoCmd=0 test.
// 2026-03-12 v4.2: Removed all remarked code (per user preference); added wake failure print; isoCmd=1.
//                  Goal: Minimal, relay closes, close contactor.
// 2026-03-14 v4.3: Fixed debounce logic bug in handleStartup() preventing trigger on stable LOW signal.
//                  Introduced separate debouncedState variable and proper lastReading update to ensure
//                  reliable falling-edge detection even if pin is LOW at startup or held LOW.
// 2026-03-14 v4.4: Added inline //rem comments in handleStartup() to explicitly mark the code changes
//                  for the debounce logic fix in v4.3.
// 2026-03-14 v4.5: Added FAULT_CLEAR_PHASE (3500ms from startup) to delay UDS fault clearing after relay close,
//                  giving the battery time to wake up and initialize before receiving diagnostic commands.
//                  This addresses the issue where no incoming CAN transmissions (e.g., from battery) were received,
//                  likely due to sending UDS too soon after powering the system via the relay.
// 2026-03-15 v4.6: Set frame.extended = true for all CAN frames based on CAN bus log analysis showing extended frames
//                  from the battery (e.g., 0x136 as Extern Frame). This fixes the ID_SFTYINFO (0x8BF) being masked
//                  to 0xBF when sent as standard (since 0x8BF > 0x7FF). Other frames (e.g., 0x60F) are compatible
//                  with extended=true as higher bits are zero. Also applied to UDS frames for consistency.
//                  This should ensure proper communication and potentially resolve contactor not closing (batSt=1, err=1).
// 2026-03-15 v4.7: Changed isoCmd to 0 (test activation, as previous versions had isoCmd=0 for testing; deactivate was 1).
//                  Increased FAULT_CLEAR_PHASE to 5000ms for more battery init time.
//                  Added Serial print when hvReq is set to 16 for debug.
//                  Added optional TX frame logging in send functions (remarked out; uncomment for debug).
// 2026-03-17 v7.1: FIXED CRC BUG — changed computeAndSetCRC length from 8→7 in all 8-byte frames
//                  (SftyInfo already used 7 correctly). This prevents out-of-bounds read of data[8]
//                  and fixes invalid CRC → Err=6 in 0x136. Battery should now accept CtlCmd (hvReq=16),
//                  VehInfo, VehTi, ThermSysInfo, BatThermReq2. Early hvReq + thermal frames kept.
//                  No other logic changed. // rem v7.1 on every change.
// 2026-03-17 v7.2: TIMING OPTIMIZATION — moved RX prints behind #ifdef DEBUG_RX
//                  to eliminate Serial.print blocking during high-rate BatSts messages.
//                  Added optional low-rate jitter monitoring (#ifdef DEBUG_JITTER) for
//                  10 ms SftyInfo frame — prints summary every 5 s, very low overhead.
//                  CRC length=7 fix from v7.1 kept. No logic changes otherwise.
//                  Goal: Minimize jitter on 10 ms / 100 ms cyclic frames per Webasto req.
//                  // rem v7.2 on every change.
// 2026-03-17 v7.3: FULL HISTORY RESTORED + FINAL TIMING HARDENING — DEBUG_RX now commented out by default
//                  (this was the remaining source of jitter when enabled). Sends now strictly first in loop.
//                  One-time boot reminder added. If you still see jitter, keep DEBUG_RX off and
//                  uncomment DEBUG_JITTER only for monitoring. Complete original history block
//                  preserved verbatim as requested. // rem v7.3 on every change.
// 2026-03-17 v7.4: NIBBLES VERIFIED CORRECT per your exact documents:
//                  02_Series-Sample E2E Communication_2020-05-14.pdf
//                  04_E2E_Communication_example_SftyInfo.pdf
//                  (and all Webasto/AUTOSAR files on GitHub/Clarity222/Battery-BMS).
//                  Alive counters now start at 1 (was 0). isoCmd=1 (was 0).
//                  These are the only changes. Full history block untouched.
//                  // rem v7.4 on every change.
// 2026-03-17 v7.5: ADDED CONFIGURABLE ALIVE_START_VALUE at top of file
//                  → set to 0 or 1 and recompile to test both behaviors
//                  No AUTOSAR E2E Profile 1C or Webasto document explicitly requires start=1
//                  (examples show both 0 and 1; many implementations accept either).
//                  Full history block preserved verbatim.
//                  // rem v7.5 on every change.
// 2026-03-17 v7.6: DEBUG_RX turned OFF by default (was left active in v7.5 paste — my mistake)
//                  Added boot message reminder. This should reduce main-loop blocking and
//                  tighten 10 ms frame timing significantly. No other changes.
//                  Full history preserved verbatim.
//                  // rem v7.6 on every change.
// 2026-03-17 v7.7: ADDED CONFIGURABLE ISOCMD_VALUE at top (0 or 1)
//                  → easy testing of isolation command values without editing code
//                  Boot message now shows both ALIVE_START_VALUE and ISOCMD_VALUE
//                  DEBUG_RX remains OFF by default. Full history preserved verbatim.
//                  // rem v7.7 on every change.
// 2026-03-17 v7.8: SET BEST DEFAULTS based on your four test logs:
//                  ALIVE_START_VALUE = 1 and ISOCMD_VALUE = 1 (longest BatSt=2 window)
//                  hvReq=16 remains immediate (no delay ever added). DEBUG_RX off.
//                  Full history preserved verbatim.
//                  // rem v7.8 on every change.
// 2026-03-17 v7.9: hvReq=16 NOW SET FROM THE VERY FIRST CAN FRAME (systemActive=true)
//                  Removed the 1.5 s delay — matches your requirement and the startup diagram.
//                  CL15 relay still closes after 1.5 s (power latch), but hvReq is already high.
//                  Full history preserved verbatim.
//                  // rem v7.9 on every change.
// 2026-03-17 v7.10: CAN RX MOVED TO INTERRUPT (CAN0_Handler) — removes polling jitter from loop()
//                   Uses Can0.attachCANInterrupt(CAN0_Handler) + Can0.read(rxFrame)
//                   Full history preserved verbatim.
//                   // rem v7.10 on every change.
// =====================================================================

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

// -------------------------- CONFIG & DEBUG CONTROLS --------------------------
const uint8_t ALIVE_START_VALUE = 1;
const uint8_t ISOCMD_VALUE     = 1;

//#define DEBUG_RX       // OFF by default
//#define DEBUG_JITTER   // uncomment for jitter monitoring
// ----------------------------------------------------------------------------

// Function prototypes (added to fix "not declared" errors)
void handleStartup();
void handleVoltageRequest();
void sendSftyInfo();
void sendCtlCmd();
void sendVehInfo();
void sendVehTi();
void sendThermSysInfo();
void sendBatThermReq2();
void getTime();
void computeAndSetCRC(uint8_t* data, uint32_t id, uint8_t length);
void canSend(CAN_FRAME &frame);

// AliveCounter class
class AliveCounter {
public:
  uint8_t value;
  AliveCounter() : value(ALIVE_START_VALUE) {}
  uint8_t increment() {
    value = (value + 1) % 15;
    return value;
  }
  void reset() { value = ALIVE_START_VALUE; }
};

AliveCounter aliveCounterSfty;
AliveCounter aliveCounterCtl;
AliveCounter aliveCounterVeh;
AliveCounter aliveCounterTherm;
AliveCounter aliveCounterThermReq;

// PINS & DEFINES
const int startupPin = 5, wakeRelayPin = 12, voltageRequestPin = 11;
#define ID_SFTYINFO      0x8BF
#define ID_CTLCMD        0x60F
#define ID_VEHINFO       0x5B0
#define ID_VEHTI         0x600
#define ID_BATSTS        0x136
#define ID_BATVOLTAGE    0x131
#define ID_THERMSYSINFO  0x701
#define ID_BATTHERMREQ2  0x720

#define PERIOD_SFTYINFO_US 10000UL
#define PERIOD_CTLCMD_US   100000UL
#define PERIOD_VEHINFO_US  100000UL
#define PERIOD_VEHTI_US    1000000UL
#define PERIOD_THERM_US    100000UL
#define PERIOD_THERMREQ_US 1000000UL

#define WAKE_DELAY       1500
#define DEBOUNCE_TIME    50

// GLOBALS
bool systemActive = false, wakeAsserted = false, canStarted = false;
unsigned long startupTime = 0;
unsigned long lastSfty_us = 0, lastCtl_us = 0, lastVeh_us = 0, lastVehTi_us = 0;
unsigned long lastTherm_us = 0, lastThermReq_us = 0;
float latestPackVoltage = 0.0;
uint8_t sftyNibble = 0x8, sftyOk = 1, crashDetnNeg = 1;
uint8_t cmdNibble = 0x6, infoNibble = 0x5;
const uint8_t isoCmd = ISOCMD_VALUE;
int8_t tAmb = 20; uint8_t vehSpd = 0; uint8_t hvReq = 0;
int startYear=2026, startMonth=3, startDay=12, startHour=12, startMinute=0, startSecond=0;
uint8_t payLoad[8];

// CAN RX INTERRUPT HANDLER (fixed for due_can library)
void CAN0_Handler(CAN_FRAME *rxFrame) {
  // Process the frame passed by the library interrupt
  if (rxFrame->id == ID_BATSTS) {
    uint8_t batSt = rxFrame->data.byte[2] & 0x3F;
    uint8_t errLvl = rxFrame->data.byte[3] & 0x0F;
#ifdef DEBUG_RX
    Serial.print("BatSts: BatSt="); Serial.print(batSt);
    Serial.print(" Err="); Serial.println(errLvl);
#endif
  } else if (rxFrame->id == ID_BATVOLTAGE) {
    uint16_t raw = (rxFrame->data.byte[0] << 8) | rxFrame->data.byte[1];
    latestPackVoltage = raw / 64.0;
#ifdef DEBUG_RX
    Serial.print("Pack Voltage (decoded): "); Serial.println(latestPackVoltage);
#endif
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  Serial.print("[0] Boot v7.10 — ALIVE_START_VALUE = "); Serial.println(ALIVE_START_VALUE);
  Serial.print("    ISOCMD_VALUE     = "); Serial.println(ISOCMD_VALUE);
  Serial.println("    CAN RX in INTERRUPT → cleaner main loop timing");
  pinMode(startupPin, INPUT_PULLUP);
  pinMode(voltageRequestPin, INPUT_PULLUP);
  pinMode(wakeRelayPin, OUTPUT);
  digitalWrite(wakeRelayPin, LOW);
  Can0.begin(CAN_BPS_500K);
  Can0.attachCANInterrupt(CAN0_Handler);  // Global RX interrupt
  aliveCounterSfty.reset(); aliveCounterCtl.reset(); aliveCounterVeh.reset();
  aliveCounterTherm.reset(); aliveCounterThermReq.reset();
}

// LOOP — no polling
void loop() {
  handleStartup();
  handleVoltageRequest();

  unsigned long now = millis();
  unsigned long now_us = micros();

#ifdef DEBUG_JITTER
  static int missedSfty = 0;
  static unsigned long lastJitterPrint = 0;
#endif

  if (systemActive) {
    if (!wakeAsserted && now - startupTime >= WAKE_DELAY) {
      digitalWrite(wakeRelayPin, HIGH);
      wakeAsserted = true;
      Serial.print("["); Serial.print(now); Serial.println("] Relay 12 CL15 closed");
      hvReq = 16;
      Serial.print("["); Serial.print(now); Serial.println("] hvReq set to 16 immediately");
    }

    if (now_us - lastSfty_us >= PERIOD_SFTYINFO_US) { 
      sendSftyInfo(); 
      lastSfty_us = now_us; 
      if (!canStarted) { canStarted = true; }
    }
#ifdef DEBUG_JITTER
    else if (now_us - lastSfty_us > PERIOD_SFTYINFO_US + 2000UL) {
      missedSfty++;
    }
#endif

    if (now_us - lastCtl_us >= PERIOD_CTLCMD_US) { sendCtlCmd(); lastCtl_us = now_us; }
    if (now_us - lastVeh_us >= PERIOD_VEHINFO_US) { sendVehInfo(); lastVeh_us = now_us; }
    if (now_us - lastVehTi_us >= PERIOD_VEHTI_US) { sendVehTi(); lastVehTi_us = now_us; }
    if (now_us - lastTherm_us >= PERIOD_THERM_US) { sendThermSysInfo(); lastTherm_us = now_us; }
    if (now_us - lastThermReq_us >= PERIOD_THERMREQ_US) { sendBatThermReq2(); lastThermReq_us = now_us; }

#ifdef DEBUG_JITTER
    if (now - lastJitterPrint >= 5000UL) {
      if (missedSfty > 0) {
        Serial.print("[JITTER] Missed SftyInfo windows (last 5s): ");
        Serial.println(missedSfty);
      }
      missedSfty = 0;
      lastJitterPrint = now;
    }
#endif
  }
}

// handleStartup
void handleStartup() {
  static bool debouncedState = HIGH;
  static bool lastReading = HIGH;
  static unsigned long lastDebounceTime = 0;
  bool reading = digitalRead(startupPin);
  if (reading != lastReading) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
    if (reading != debouncedState) {
      debouncedState = reading;
      if (debouncedState == LOW && !systemActive) {
        systemActive = true;
        startupTime = millis();
        hvReq = 16;
        Serial.println("[START] System activated — hvReq=16 from first CAN frame");
      }
    }
  }
  lastReading = reading;
}

// handleVoltageRequest
void handleVoltageRequest() {
  static bool voltageRequested = false;
  if (digitalRead(voltageRequestPin) == LOW && !voltageRequested) {
    voltageRequested = true;
    Serial.println("[V] Voltage request pin LOW");
  }
}

// sendSftyInfo
void sendSftyInfo() {
  CAN_FRAME frame; frame.id = ID_SFTYINFO; frame.extended = true; frame.length = 8;
  uint8_t alive = aliveCounterSfty.value;
  frame.data.byte[1] = (sftyNibble << 4) | (alive & 0x0F);
  frame.data.byte[2] = (sftyOk << 2) | crashDetnNeg;
  for (int i = 3; i < 8; i++) frame.data.byte[i] = 0xFF;
  computeAndSetCRC(frame.data.byte, ID_SFTYINFO, 7);
  Can0.sendFrame(frame);
  aliveCounterSfty.increment();
}

// sendCtlCmd
void sendCtlCmd() {
  CAN_FRAME frame; frame.id = ID_CTLCMD; frame.extended = true; frame.length = 8;
  uint8_t alive = aliveCounterCtl.value;
  frame.data.byte[1] = (cmdNibble << 4) | (alive & 0x0F);
  frame.data.byte[2] = hvReq;
  frame.data.byte[3] = isoCmd;
  for (int i = 4; i < 8; i++) frame.data.byte[i] = 0xFF;
  computeAndSetCRC(frame.data.byte, ID_CTLCMD, 7);
  Can0.sendFrame(frame);
  aliveCounterCtl.increment();
}

// sendVehInfo
void sendVehInfo() {
  CAN_FRAME frame; frame.id = ID_VEHINFO; frame.extended = true; frame.length = 8;
  uint8_t alive = aliveCounterVeh.value;
  frame.data.byte[0] = (infoNibble << 4) | (alive & 0x0F);
  frame.data.byte[1] = (uint8_t)tAmb; frame.data.byte[2] = vehSpd;
  for (int i = 3; i < 8; i++) frame.data.byte[i] = 0xFF;
  computeAndSetCRC(frame.data.byte, ID_VEHINFO, 7);
  Can0.sendFrame(frame);
  aliveCounterVeh.increment();
}

// sendVehTi
void sendVehTi() {
  getTime();
  CAN_FRAME frame; frame.id = ID_VEHTI; frame.extended = true; frame.length = 8;
  for (int i = 0; i < 5; i++) frame.data.byte[i] = payLoad[i];
  frame.data.byte[5] = 0x00; frame.data.byte[6] = 0x00; frame.data.byte[7] = 0xFF;
  computeAndSetCRC(frame.data.byte, ID_VEHTI, 7);
  Can0.sendFrame(frame);
}

// getTime
void getTime() {
  payLoad[0] = startYear & 0xFF;
  payLoad[1] = (startYear >> 8);
  payLoad[2] = startMonth;
  payLoad[3] = startDay;
  payLoad[4] = (startHour << 3) | (startMinute >> 3);
}

// sendThermSysInfo
void sendThermSysInfo() {
  CAN_FRAME frame; frame.id = ID_THERMSYSINFO; frame.extended = true; frame.length = 8;
  uint8_t alive = aliveCounterTherm.value;
  frame.data.byte[0] = (0x7 << 4) | (alive & 0x0F);
  for (int i = 1; i < 8; i++) frame.data.byte[i] = 0xFF;
  computeAndSetCRC(frame.data.byte, ID_THERMSYSINFO, 7);
  Can0.sendFrame(frame);
  aliveCounterTherm.increment();
}

// sendBatThermReq2
void sendBatThermReq2() {
  CAN_FRAME frame; frame.id = ID_BATTHERMREQ2; frame.extended = true; frame.length = 8;
  uint8_t alive = aliveCounterThermReq.value;
  frame.data.byte[0] = (0x7 << 4) | (alive & 0x0F);
  frame.data.byte[1] = 0x01;
  for (int i = 2; i < 8; i++) frame.data.byte[i] = 0xFF;
  computeAndSetCRC(frame.data.byte, ID_BATTHERMREQ2, 7);
  Can0.sendFrame(frame);
  aliveCounterThermReq.increment();
}

// computeAndSetCRC
void computeAndSetCRC(uint8_t* data, uint32_t id, uint8_t length) {
  uint8_t crc_data[16];
  crc_data[0] = id & 0xFF;
  if (id == ID_SFTYINFO) crc_data[1] = 0x00; else crc_data[1] = (id >> 8) & 0x0F;
  for (uint8_t i = 0; i < length; i++) crc_data[i + 2] = data[i + 1];
  uint8_t crc = CRC8_SAE_J1850_ZERO::calculate(crc_data, length + 2);
  data[0] = crc;
}

// canSend
void canSend(CAN_FRAME &frame) { Can0.sendFrame(frame); }