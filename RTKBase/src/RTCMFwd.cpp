// RTCMFwd.cpp - BS module for forwarding RTCM3 corrections over RFM69
#include <RTKF3F.h>
#include "RTCMFwd.h"

#define REG_VERSION 0x10
#define EXPECTED_RFM69_VERSION 0x24

static HardwareSerial* rtcmInput = nullptr;
static RFM69* rtcmRadio = nullptr;

/* while (SerialGNSS.available()) {
           uint8_t b = SerialGNSS.read();

       // Detect RTCM3 start

       if (!readingRtcm) {
           if (b == 0xD3) {
               rtcmBuffer[0] = b;
               rtcmIndex = 1;
               readingRtcm = true;
           }
       }
       else {
           if (rtcmIndex < BUFFER_SIZE - 1) {
               rtcmBuffer[rtcmIndex++] = b;

               // Once we have enough for length (after 3 bytes)
               if (rtcmIndex == 3) {
                   expectedLength = ((rtcmBuffer[1] & 0x03) << 8) | rtcmBuffer[2];
                   expectedLength += 6; // Add header (3) + CRC (3)
               }

               if (rtcmIndex == expectedLength) {
                   // Full RTCM message received
                   airBegin = millis();
                   radio.send(255, rtcmBuffer, rtcmIndex); // 255 = Braodcast
                   airTime += millis() - airBegin;
                   Serial.printf("Sent RTCM (%d bytes) %lu ms, %.2f %%\n", rtcmIndex, (unsigned long)airTime, 100.0 * airTime / (millis()-airInit));
                   readingRtcm = false;
                   rtcmIndex = 0;
               }
           }
           else {
               // Overflow
               readingRtcm = false;
               rtcmIndex = 0;
           }
       }
   }
   */

bool verifyRadio(RFM69& radio) {
    uint8_t version = radio.readReg(REG_VERSION);
    if (version != EXPECTED_RFM69_VERSION) {
        Serial.printf("verifyRadio: Unexpected RFM69 version 0x%02X (expected 0x%02X)\n", version, EXPECTED_RFM69_VERSION);
        return false;
    }
    return true;
}

void initRTCMForwarder(HardwareSerial* input, RFM69* radio) {
  rtcmInput = input;
  rtcmRadio = radio;
}

// Serial.printf("Sent RTCM (%d bytes) %lu ms, %.2f %%\n", rtcmIndex, (unsigned long)airTime, 100.0 * airTime / (millis()-airInit));         
void updateRTCMForwarder() {
  if (!rtcmInput || !rtcmRadio) return;

  if (rtcmInput->available()) {
    uint8_t packet[1000];
    size_t len = 0;

    packet[0] = MSG_RTCM;  // Prefix with message type
    while (rtcmInput->available() && len < sizeof(packet) - 1) {
      packet[len + 1] = rtcmInput->read();
      len++;
    }

    if (len > 0) {
      rtcmRadio->send(255, packet, len + 1);
	  Serial.printf("Sent RTCM (%zu bytes)\n", len + 1);
    }
  }
}
