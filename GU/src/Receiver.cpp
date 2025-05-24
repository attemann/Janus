// Receiver.cpp
#include <RTKF3F.h>
#include <RFM69.h>  
#include "EventDetection.h"
#include "GUTX.h"

extern Slope slope;
extern GNSSFix lastFix;

void handleRadioMessage(uint8_t* data, uint8_t len) {
  if (len < 1) return;

  switch (data[0]) {
    case MSG_RTCM:
      if (len > 1) {
        Serial2.write(data + 1, len - 1);
      }
      break;

    case MSG_FLIGHT_SETTINGS:
      if (len >= 9) {
        uint16_t angle10 = data[1] | (data[2] << 8);
        bool aBaseLeft = data[3];
        uint8_t gliderId = data[4];

        int16_t offsetN_cm = data[5] | (data[6] << 8);
        int16_t offsetE_cm = data[7] | (data[8] << 8);

        slope.setGliderId(gliderId); 
        slope.setSlopeAngle(angle10 / 10.0f);
        slope.setABaseLeft(aBaseLeft);
        slope.setPilotOffsetNED(offsetN_cm / 100.0f, offsetE_cm / 100.0f, 0);

        initEventDetection();
      }
      break;

    case MSG_REQ_POS: {
      // send relative pos to base (true), not pilot (false)
      txRelPos(lastFix, true);
      break;
    }

    default:
      break;
  }
}