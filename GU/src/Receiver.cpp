// Receiver.cpp
#include <RTKF3F.h>
#include <RFM69.h>  
#include "EventDetection.h"
#include "GUTX.h"
#include "Receiver.h"

extern Slope slope;
extern GNSSFix lastFix;
extern bool showRTCM;

void handleRadioMessage(uint8_t* data, uint8_t len) {
  if (len < 1) return;

  switch (data[0]) {
  case MSG_RTCM: {
      if (len <= 1) {
          if (showRTCM) Serial.println("RTCM message too short");
          return;
      }
      const uint8_t* rtcm = data + 1;
      size_t rtcmLen = len - 1;

      if (!isValidRTCM(rtcm, rtcmLen)) {
          if (showRTCM) Serial.println(">>> Invalid RTCM frame — rejected");
          return;
      }

      //Serial.printf("? Forwarding valid RTCM (%d bytes)\n", rtcmLen);
      SerialGNSS.write(rtcm, rtcmLen);
      break;
  }

  }
}