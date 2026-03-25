// Ver501
// ==============================================
// FULL CUMULATIVE VERSION HISTORY (PERMANENT — NEVER REMOVE ANY ENTRY)
// ==============================================
// v9.33C  Brian fix timing  TODO first cnt stars on1
// ... (your full history block unchanged) ...
// ==============================================

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"
#include "BatteryBMS.h"          // ← NEW: critical prepare + CRC code now protected here

#include <RTCDue.h>
   // and can_common.h is pulled in automatically

RTCDue rtc(0);         // 0 = internal oscillator

// ====================== DTC REQUEST CLASS (unchanged) ======================
class BMSDiagnostics {
public:
    void sendAllDTCRequest() {
        CAN_FRAME frame;
        frame.id = 0x7E0;
        frame.extended = false;
        frame.length = 2;
        frame.data.uint8[0] = 0x19;
        frame.data.uint8[1] = 0x02;
        Can0.sendFrame(frame);
        Serial.println("→ Sent UDS 0x19 02 FF (Read ALL DTCs) on 0x7E0");
    }
};
BMSDiagnostics bmsDiag;

// ====================== YOUR GLOBALS (only non-critical ones remain) ======================
uint8_t isInMeasDisaCmd = 1;   // change to 0 for test

#define STARTUP_PIN      5
#define WAKE_RELAY_PIN   12
#define HV_REQUEST_PIN   6


uint32_t lastSfty = 0;
uint32_t lastCtl = 0;
uint32_t lastVeh = 0;
uint32_t lastTi = 0;

uint32_t lastAnySend = 0;
const uint32_t PERIOD_CTL  = 100000UL;
const uint32_t PERIOD_VEH  = 100000UL;
const uint32_t PERIOD_Ti  =  1000000UL;
const uint32_t MIN_FRAME_GAP = 2500UL;

bool cl15Closed = false;
bool cl15Detected = false;
uint32_t cl15StartTime = 0;
bool lastStartupState = HIGH;
bool canActive = false;
bool canStop = false;
uint8_t batSt = 1;
uint8_t hvReq = 0;
void sendPrepared(CAN_FRAME &frame) {
    Can0.sendFrame(frame);
}

void setup() {
    Serial.begin(115200);
    pinMode(STARTUP_PIN, INPUT_PULLUP);
    pinMode(WAKE_RELAY_PIN, OUTPUT);
    digitalWrite(WAKE_RELAY_PIN, LOW);
    pinMode(HV_REQUEST_PIN, INPUT);

    Can0.begin(CAN_BPS_500K);
    Can0.watchFor();
    Serial.println("v9.33C booted — critical CAN prep now in BatteryBMS library");

    prepareSfty();
    prepareCtl(hvReq, isInMeasDisaCmd);   // ← updated
    prepareVeh();
       rtc.setDate(24, 3, 2025);     // day, month, year
    rtc.setTime(12, 34, 0);   
rtc.begin();
//  rtc.setTime(17, 24, 45);  // HH:MM:SS
  //rtc.setDate(23, 3, 2026); // DD:MM:YYYY
}



/*




void sendVehTi() {
  uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Time per DBC

  frame.id = 0x410;
  frame.length = 8;
  memcpy(frame.data.bytes, data, 8);
  Can0.sendFrame(frame);
  printFrame(frame, "TX");
}

void sendVehTi() {
  CAN_FRAME tempFrame;
    // Read RTC (assuming Due RTC library returns reasonable values)
    uint16_t year  = rtc.getYear();   // e.g. 2025
    uint8_t  month = rtc.getMonth();  // 1..12
    uint8_t  day   = rtc.getDay();    // 1..31
    uint8_t  hour  = rtc.getHours();  // 0..23
    uint8_t  minu  = rtc.getMinutes();// 0..59
    uint8_t  sec   = rtc.getSeconds(); // 0..59

    // Pack into 64-bit integer (LSB = bit 0 of DBC)
    uint64_t packed = 0;
    packed |= ((uint64_t)(year - 2000) & 0xFFF) << 0;   // bits  0–11
    packed |= ((uint64_t)month          & 0xF)   << 12;  // bits 12–15
    packed |= ((uint64_t)day            & 0x3F)  << 16;  // bits 16–21
    packed |= ((uint64_t)hour           & 0x1F)  << 22;  // bits 22–26
    packed |= ((uint64_t)minu           & 0x3F)  << 27;  // bits 27–32
    packed |= ((uint64_t)sec            & 0x3F)  << 33;  // bits 33–38

    // Extract to 8 bytes, big-endian (byte 0 = highest bits)
    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
        data[i] = (packed >> (56 - i * 8)) & 0xFF;   // 56 = 64-8
    }

    // Prepare and send frame
    CAN_FRAME Frame;  // assuming your library struct
    Frame.id       = 0x400;         // Correct extended ID !!!
    Frame.extended = true;          // MUST be extended (29-bit)
    Frame.length   = 8;
    memcpy(Frame.data.uint8, data, 8);

    Can0.sendFrame(Frame);

    // Optional debug output
    Serial.print("TX VehTi ID=0x400  ");
    Serial.print(year);   Serial.print("-");
    Serial.print(month);  Serial.print("-");
    Serial.print(day);    Serial.print(" ");
    Serial.print(hour);   Serial.print(":");
    Serial.print(minu);   Serial.print(":");
    Serial.println(sec);
}
*/
void sendVehTi() {

    uint16_t year  = rtc.getYear();
    uint8_t  month = rtc.getMonth();
    uint8_t  day   = rtc.getDay();
    uint8_t  hour  = rtc.getHours();
    uint8_t  minu  = rtc.getMinutes();
    uint8_t  sec   = rtc.getSeconds();

    uint64_t packed = 0;

    // Pack signals (Intel / little-endian)
    packed |= ((uint64_t)year  & 0xFFF) << 0;
    packed |= ((uint64_t)month & 0xF)   << 12;
    packed |= ((uint64_t)day   & 0x3F)  << 16;
    packed |= ((uint64_t)hour  & 0x1F)  << 22;
    packed |= ((uint64_t)minu  & 0x3F)  << 27;
    packed |= ((uint64_t)sec   & 0x3F)  << 33;

    uint8_t data[8];

    // ✅ FIX: Little-endian byte order
    for (int i = 0; i < 8; i++) {
        data[i] = (packed >> (i * 8)) & 0xFF;
    }

    CAN_FRAME Frame;
    Frame.id       = 0x80000400;  // ✅ FIXED ID
    Frame.extended = true;
    Frame.length   = 8;

    memcpy(Frame.data.uint8, data, 8);

    Can0.sendFrame(Frame);
}


void loop() {
    uint32_t now = micros();

    // ====================== LIVE 0x136 DECODER (unchanged) ======================
    CAN_FRAME incoming;
    while (Can0.available() > 0) {
        Can0.read(incoming);
        if (incoming.id == 0x136 && incoming.length == 8) {
            uint8_t stateByte = incoming.data.uint8[2];
            batSt = stateByte & 0x3F;
            
          //  if (!canActive && (batSt == 2)) {
           //     canActive = true;
            //    lastSfty = now + 10000UL;
             //   lastCtl  = now + 40000UL;
              //  lastVeh  = now + 75000UL;
               // Serial.println(">>> 0x136 C2 (BatSt==2) DETECTED — ARDUINO CAN TRANSMISSION STARTED <<<");
           // }
            if (batSt == 3) {
                Serial.println(">>> BatSt == 3 reached — READY FOR HVREQ <<<");
            }
        }
    }

    if (canActive) {
        if (now - lastSfty >= 10000UL) {
            sendPrepared(sftyFrame);
            lastSfty = now;
            prepareSfty();
        }
      
        if (now - lastCtl >= PERIOD_CTL) {
            sendPrepared(ctlFrame);
            lastCtl = now;
            batSt = 1;
            hvReq = 0; // Disconnect
            if (batSt == 3) hvReq = 16;
            prepareCtl(hvReq, isInMeasDisaCmd);   // ← updated
        }
        
        if (now - lastVeh >= PERIOD_VEH) {
            sendPrepared(vehFrame);
            lastVeh = now;
            prepareVeh();
        }
   if (now - lastTi >= PERIOD_Ti) {
            sendVehTi();
           // sendPrepared(vehFrame);
            lastTi = now;

            //prepareVeh();
        }



    }

    if ( now- cl15StartTime >= 1500000 && !cl15Closed) { 
            digitalWrite(WAKE_RELAY_PIN, HIGH);
            cl15Closed = true;

           }
  
    bool currentState = digitalRead(STARTUP_PIN);
    if (!cl15Closed && currentState == LOW && !canActive) { 
        cl15Detected = true;
        cl15StartTime = now; 
        canActive= true;
                lastSfty = now + 10000UL;
               lastCtl  = now + 40000UL;
              lastVeh  = now + 75000UL;            
               Serial.println("Input Trigger Can start");
    }

    // ====================== DTC REQUEST TIMER (unchanged) ======================
 
          /*
    static uint32_t lastDTC = 0;
    if (canActive) { 
        if (millis() - lastDTC >= 150000) {
            bmsDiag.sendAllDTCRequest();
            Serial.println("TIME TRIGGER");
            lastDTC = millis();
        }
    }
*/

}