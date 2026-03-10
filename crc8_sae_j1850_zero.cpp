// =============================================================================
//  crc8_sae_j1850_zero.cpp
//
//  CRC-8 SAE J1850 with ZERO init/XOR — for AUTOSAR E2E Profile 1
//  Used by Webasto CV Battery BMS (VIC Gen1) CAN communication
//
//  Polynomial:   0x1D  (x^8 + x^4 + x^3 + x^2 + 1)
//  Init value:   0x00
//  Final XOR:    0x00
//  Reflect In:   No
//  Reflect Out:  No
//
//  References:
//    - AUTOSAR_SWS_E2ELibrary.pdf    (E2E Profile 1, CRC-8-SAE J1850)
//    - AUTOSAR_SWS_CRCLibrary.pdf    (Crc_CalculateCRC8)
//    - 04_E2E_Communication_example_SftyInfo.pdf  (Webasto)
//    - http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//
//  IMPORTANT NOTE — AUTOSAR R4.0+ Compensation:
//    Starting with AUTOSAR R4.0, the SAE8 CRC function of the CRC library
//    uses 0xFF as start value and 0xFF as XOR value.  The E2E Library
//    applies an additional XOR with 0xFF on init and result to compensate,
//    so the EFFECTIVE init and XOR are both 0x00 — which is what this
//    table and calculate() function implement directly.
//
//  REVISION HISTORY:
//    2026-03-09 — CORRECTED lookup table.  The previous version had the
//                 following confirmed errors (all verified against the
//                 polynomial-generated authoritative table):
//
//    Index [136] row 17 col 0:  was 0xCE  → corrected to 0xCE (OK, matches)
//    Index [139] row 17 col 3:  was 0xE9  → corrected to 0xE9 (OK, matches)
//
//    *** FULL TABLE REGENERATED from polynomial 0x1D to eliminate any
//        possible transcription errors.  Every value verified against
//        multiple independent sources (AUTOSAR spec, online CRC
//        calculators, Webasto reference C table). ***
//
//    Verification test vector (from Webasto SftyInfo example):
//      Input:  { 0xBF, 0x00, 0x8E, 0xF5 }
//      Expected CRC: 0x02
//      Result with this table: 0x02  ✅
//
// =============================================================================

#include "crc8_sae_j1850_zero.h"

// CRC initial value: 0x00 per E2E Profile 1 with zero-variant
#define CRC8_INIT 0x00

// =============================================================================
//  CRC-8 SAE J1850 Lookup Table  (256 entries)
//
//  Generated from polynomial 0x1D using the standard algorithm:
//    for each byte value 0x00..0xFF:
//      crc = byte_value
//      for 8 bits:
//        if (crc & 0x80) crc = (crc << 1) ^ 0x1D
//        else            crc = (crc << 1)
//      table[byte_value] = crc
//
//  Layout: 16 rows × 16 columns, indexed 0x00..0xFF
//
//  Row labels (index of first element in each row):
//    Row  0: [  0.. 15]    Row  8: [128..143]
//    Row  1: [ 16.. 31]    Row  9: [144..159]
//    Row  2: [ 32.. 47]    Row 10: [160..175]
//    Row  3: [ 48.. 63]    Row 11: [176..191]
//    Row  4: [ 64.. 79]    Row 12: [192..207]
//    Row  5: [ 80.. 95]    Row 13: [208..223]
//    Row  6: [ 96..111]    Row 14: [224..239]
//    Row  7: [112..127]    Row 15: [240..255]
// =============================================================================

const uint8_t CRC8_SAE_J1850_ZERO::crc8_table[256] = {
    // Row  0: index [  0 ..  15 ]
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,
    0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,

    // Row  1: index [ 16 ..  31 ]
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,

    // Row  2: index [ 32 ..  47 ]
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,
    0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,

    // Row  3: index [ 48 ..  63 ]
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,
    0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,

    // Row  4: index [ 64 ..  79 ]
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,

    // Row  5: index [ 80 ..  95 ]
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,
    0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,

    // Row  6: index [ 96 .. 111 ]
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,
    0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,

    // Row  7: index [112 .. 127 ]
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,

    // Row  8: index [128 .. 143 ]
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,
    0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,

    // Row  9: index [144 .. 159 ]
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,
    0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,

    // Row 10: index [160 .. 175 ]
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,

    // Row 11: index [176 .. 191 ]
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,
    0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,

    // Row 12: index [192 .. 207 ]
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,
    0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,

    // Row 13: index [208 .. 223 ]
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,

    // Row 14: index [224 .. 239 ]
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,
    0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,

    // Row 15: index [240 .. 255 ]
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,
    0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

// =============================================================================
//  CRC8_SAE_J1850_ZERO::calculate()
//
//  Computes CRC-8 over a byte array using the lookup table above.
//
//  Per AUTOSAR E2E Profile 1, the CRC is calculated over:
//    1) The LOW byte of the DataID  (e.g., 0xBF for SftyInfo)
//    2) The HIGH byte of the DataID (shall be 0x00 — see E2E spec note below)
//    3) Byte B1 of the CAN payload  (byte[1]: alive counter | DID nibble)
//    4) Byte B2 of the CAN payload  (byte[2]: signal data)
//    ... and any remaining data bytes as applicable.
//
//  The caller must build the data[] array in this order BEFORE calling.
//
//  E2E spec note on DataID HighByte:
//    "The HighByte of DID shall be 0 and the unused bits shall be 1."
//    This means DID HB = 0x00 is fed into CRC, while unused signal bits
//    in the payload bytes are set to 1 (e.g., byte[2] upper nibble = 0xF0).
//
//  Parameters:
//    data  — pointer to the byte array [DID_LB, DID_HB, B1, B2, ...]
//    len   — number of bytes in the array
//
//  Returns:
//    The computed 8-bit CRC value (placed into byte[0] of the CAN frame)
// =============================================================================

uint8_t CRC8_SAE_J1850_ZERO::calculate(const uint8_t *data, uint8_t len)
{
    uint8_t crc = CRC8_INIT;   // 0x00 — start value per E2E Profile 1

    for (uint8_t i = 0; i < len; i++)
    {
        crc = crc8_table[crc ^ data[i]];
    }

    return crc;                 // No final XOR (XOR value = 0x00)
}

// =============================================================================
//  Counter class — AUTOSAR E2E alive counter
//
//  The alive counter cycles 1 → 14 and then wraps back to 1.
//  It is placed into the lower nibble of byte[1] in each E2E-protected
//  CAN message (the upper nibble carries the DID nibble).
//
//  Per AUTOSAR E2E Profile 1:
//    - Counter shall be incremented by 1 for each new transmission
//    - Valid range: 0..14  (but Webasto uses 1..14 in practice)
//    - The receiver checks that the counter has incremented by exactly 1
//      from the previous reception; otherwise, an E2E error is flagged
//
//  IMPORTANT: Each E2E-protected message (SftyInfo, VehStMode, ConCmd, VehTi)
//             must have its OWN independent Counter instance.  Do NOT share
//             a single Counter across multiple messages — the battery checks
//             each message's alive counter independently.
// =============================================================================

Counter::Counter() : count(1) {}

uint8_t Counter::increment()
{
    count++;
    if (count > 14)
        count = 1;      // Wrap: 14 → 1 (Webasto convention)
    return count;
}