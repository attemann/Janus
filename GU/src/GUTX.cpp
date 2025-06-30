// GUTX.cpp
#include <Arduino.h>
#include <RFM69.h> 
#include <RTKF3F.h>
#include "GUTX.h"

extern RFM69 rxtxRadio;
extern Slope slope;
extern GNSSFix lastFix;


void sendWithReturnFreq(uint8_t dest, const uint8_t* msg, uint8_t len) {
  // Optional: stabilize before frequency change
  rxtxRadio.setMode(RF69_MODE_SLEEP);
  rxtxRadio.setFrequency(BS_RX_FREQ);
  rxtxRadio.setMode(RF69_MODE_TX);

  rxtxRadio.send(dest, msg, len);

  rxtxRadio.setMode(RF69_MODE_SLEEP);
  rxtxRadio.setFrequency(BS_TX_FREQ);
  rxtxRadio.setMode(RF69_MODE_RX);
}

uint8_t fixStatus(GNSSFix fix) {
  uint8_t status=0;

  if (fix.gpsFix)     status |= STATUS_GPS_FIX;       // Bit 3: GNSS fix valid
  if (fix.rtkFloat)   status |= STATUS_RTK_FLOAT;     // Bit 1: RTK Float
  if (fix.rtkFix)     status |= STATUS_RTK_FIX;       // Bit 2: RTK Fixed
  if (fix.diffUsed)   status |= STATUS_DGNSS_USED;    // Bit 4: Differential corrections used

  status |= STATUS_LINK_OK;  
  return status;
}

void txEvent(EventCode code, uint8_t status_flags) {
    uint8_t msg[6];
    encodeEventMessage(msg, slope.getGliderId(),
        static_cast<uint8_t>(code),  // cast added
        status_flags);
    sendWithReturnFreq(BASE_NODE_ID, msg, sizeof(msg));
}

void txRelPos(GNSSFix fix, bool isRelativeToBase) {
  uint8_t msg[6];
  int n,e,d;

  if (isRelativeToBase) {   // relative position to base
    n = fix.relNorth;
    e = fix.relEast;
    d = fix.relDown;
  } else {
    n = fix.adjNorth;  // relative position to pilot
    e = fix.adjEast;
    d = fix.adjDown;
  }

  int16_t n_dm = n / 10;
  int16_t e_dm = e / 10;
  int16_t d_dm = d / 10;

  encodeRelPosMessage(msg, slope.getGliderId(), n_dm, e_dm, d_dm, fixStatus(fix));
  sendWithReturnFreq(BASE_NODE_ID, msg, sizeof(msg));
}

void txMsg(uint8_t msgCode, uint8_t d) {
  uint8_t msg[6];
  encodeMiscMessage(msg, slope.getGliderId(), d);
  sendWithReturnFreq(BASE_NODE_ID, msg, sizeof(msg));
}