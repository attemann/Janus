// RTCMForwarder.cpp - BS module for forwarding RTCM3 corrections over RFM69
#include <RTKF3F.h>
#include "RTCMFwd.h"
#include <LiquidCrystal_I2C.h>

#define REG_VERSION 0x10
#define EXPECTED_RFM69_VERSION 0x24

static HardwareSerial* rtcmInput = nullptr;
static RFM69* rtcmRadio = nullptr;
static uint8_t rtcmTargetNode = 0;
static unsigned long lastSendTime = 0;

extern LiquidCrystal_I2C lcd;

bool verifyRadio(RFM69& radio, const char* label) {
    uint8_t version = radio.readReg(REG_VERSION);
    if (version != EXPECTED_RFM69_VERSION) {
        Serial.printf("%s radio not detected! REG_VERSION = 0x%02X\n", label, version);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(String(label) + " Init");
        lcd.setCursor(0, 1);
        lcd.print("FAILED");
        return false;
    }
    return true;
}

void initRTCMForwarder(HardwareSerial* input, RFM69* radio, uint8_t targetNode) {
  rtcmInput = input;
  rtcmRadio = radio;
  rtcmTargetNode = targetNode;
  lastSendTime = 0;
}

void updateRTCMForwarder() {
  if (!rtcmInput || !rtcmRadio) return;

  const unsigned long now = millis();
  if (now - lastSendTime < RTCM_INTERVAL) return;  // 1 Hz limit

  if (rtcmInput->available()) {
    uint8_t packet[64];
    size_t len = 0;

    packet[0] = MSG_RTCM;  // Prefix with message type
    while (rtcmInput->available() && len < sizeof(packet) - 1) {
      packet[len + 1] = rtcmInput->read();
      len++;
    }

    if (len > 0) {
      rtcmRadio->send(rtcmTargetNode, packet, len + 1);
      lastSendTime = now;
    }
  }
}
