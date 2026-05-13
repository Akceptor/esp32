
// namesender.ino
// Inline CRSF passthrough with MSP_SET_NAME injection.
//
// ESP32 sits between an RC receiver (CRSF out on its TX pin) and a flight
// controller (CRSF RX pin). The sketch forwards every received CRSF frame
// byte-for-byte to the FC. Roughly once per second, after forwarding the
// next non-MSP CRSF frame, an MSP_SET_NAME frame carrying "ESP <N>"
// (N=1..9 rolling) is injected into the stream.

#include <Arduino.h>

// ---- User config ----
#define CRSF_RX_PIN       16        // Serial1 RX  <- receiver CRSF TX
#define CRSF_TX_PIN       17        // Serial2 TX  -> FC CRSF RX
#define CRSF_BAUD         420000
#define DEBUG_BAUD        115200
#define INJECT_PERIOD_MS  1000

// ---- CRSF protocol ----
#define CRSF_SYNC_FC              0xC8
#define CRSF_ADDR_FC              0xC8
#define CRSF_ADDR_HANDSET         0xEA
#define CRSF_FRAMETYPE_MSP_REQ    0x7A
#define CRSF_FRAMETYPE_MSP_RESP   0x7B
#define CRSF_FRAMETYPE_MSP_WRITE  0x7C
#define CRSF_MAX_FRAME_LEN        64
#define CRSF_RX_TIMEOUT_MS        10

// ---- MSP ----
#define MSP_SET_NAME      11
#define NAME_MAX_LENGTH   16
// CRSF-MSP status byte: start=1, seq=0. Matches betaflight-tx-lua-scripts.
#define MSP_CRSF_STATUS   0x30

HardwareSerial rxIn(1);   // from receiver
HardwareSerial txOut(2);  // to FC

// Parser state
static uint8_t  frameBuf[CRSF_MAX_FRAME_LEN];
static uint8_t  frameIdx    = 0;
static uint8_t  expectedLen = 0;
static uint32_t lastByteMs  = 0;

// Injection state
static bool     injectPending = false;
static uint32_t lastTickMs    = 0;
static uint8_t  nameCounter   = 1;

static uint8_t crc8_dvb_s2(const uint8_t *buf, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t msp_xor_crc(const uint8_t *buf, uint8_t len) {
    uint8_t c = 0;
    for (uint8_t i = 0; i < len; i++) c ^= buf[i];
    return c;
}

// Build and send a CRSF-encapsulated MSP_SET_NAME frame.
//   Layout (26 bytes):
//     [0]   0xC8                 CRSF sync (FC addr)
//     [1]   0x18 (24)            CRSF length: type + 22 wrap + crsf_crc
//     [2]   0x7A                 frame type: MSP_REQ
//     [3]   0xC8                 ext dest = FC
//     [4]   0xEA                 ext src  = handset
//     [5]   0x30                 MSP-CRSF status (start=1, seq=0)
//     [6]   0x10 (16)            MSP payload size
//     [7]   0x0B (11)            MSP function = SET_NAME
//     [8..23] name bytes, zero-padded to 16
//     [24]  msp_crc              XOR over bytes [6..23]
//     [25]  crsf_crc             CRC-8/DVB-S2 over bytes [2..24]
static void sendMspSetName(uint8_t counter) {
    uint8_t frame[26];
    char name[NAME_MAX_LENGTH] = {0};
    snprintf(name, sizeof(name), "ESP %u", counter);

    frame[0] = CRSF_SYNC_FC;
    frame[1] = 24;
    frame[2] = CRSF_FRAMETYPE_MSP_REQ;
    frame[3] = CRSF_ADDR_FC;
    frame[4] = CRSF_ADDR_HANDSET;
    frame[5] = MSP_CRSF_STATUS;
    frame[6] = NAME_MAX_LENGTH;
    frame[7] = MSP_SET_NAME;
    for (uint8_t i = 0; i < NAME_MAX_LENGTH; i++) {
        frame[8 + i] = (uint8_t)name[i];
    }
    frame[24] = msp_xor_crc(&frame[6], 1 + 1 + NAME_MAX_LENGTH);   // 18 bytes
    frame[25] = crc8_dvb_s2(&frame[2], 1 + 22);                    // 23 bytes

    txOut.write(frame, sizeof(frame));
}

// Returns true once a complete, CRC-valid CRSF frame has been forwarded
// to txOut; *outFrameType is set to that frame's type byte.
static bool readAndForward(uint8_t *outFrameType) {
    uint32_t now = millis();
    if (frameIdx > 0 && (now - lastByteMs) > CRSF_RX_TIMEOUT_MS) {
        frameIdx = 0;
        expectedLen = 0;
    }

    while (rxIn.available()) {
        uint8_t b = (uint8_t)rxIn.read();
        lastByteMs = millis();

        if (frameIdx == 0) {
            if (b == CRSF_SYNC_FC) frameBuf[frameIdx++] = b;
            continue;
        }

        if (frameIdx == 1) {
            // CRSF length = type + payload + crc (so at least 2)
            if (b < 2 || b > CRSF_MAX_FRAME_LEN - 2) {
                frameIdx = 0;
                continue;
            }
            expectedLen = b;
            frameBuf[frameIdx++] = b;
            continue;
        }

        frameBuf[frameIdx++] = b;
        if (frameIdx >= (uint8_t)(2 + expectedLen)) {
            uint8_t calcCrc = crc8_dvb_s2(&frameBuf[2], expectedLen - 1);
            uint8_t rxCrc   = frameBuf[2 + expectedLen - 1];
            if (calcCrc == rxCrc) {
                *outFrameType = frameBuf[2];
                txOut.write(frameBuf, 2 + expectedLen);
                frameIdx = 0;
                expectedLen = 0;
                return true;
            }
            frameIdx = 0;
            expectedLen = 0;
        }
    }
    return false;
}

static bool isMspType(uint8_t t) {
    return t == CRSF_FRAMETYPE_MSP_REQ
        || t == CRSF_FRAMETYPE_MSP_RESP
        || t == CRSF_FRAMETYPE_MSP_WRITE;
}

void setup() {
    Serial.begin(DEBUG_BAUD);
    rxIn.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, -1);    // RX only
    txOut.begin(CRSF_BAUD, SERIAL_8N1, -1, CRSF_TX_PIN);   // TX only
    lastTickMs = millis();
    Serial.println("[namesender] up");
}

void loop() {
    uint32_t now = millis();
    if ((uint32_t)(now - lastTickMs) >= INJECT_PERIOD_MS) {
        injectPending = true;
        lastTickMs = now;
    }

    uint8_t frameType = 0xFF;
    if (readAndForward(&frameType)) {
        if (injectPending && !isMspType(frameType)) {
            sendMspSetName(nameCounter);
            nameCounter = (nameCounter % 9) + 1;
            injectPending = false;
        }
    }
}
