// GUTX.cpp
#include <Arduino.h>
#include <RFM69.h> 
#include <RTKF3F.h>

#include "RadioModule.h"

#include "GUTX.h"

extern RadioModule radioMod;
//extern Slope slope;



void txEvent(EventCode code, uint8_t status_flags) {
    uint8_t msg[6];
    encodeEventMessage(msg, NODEID_GU,
        static_cast<uint8_t>(code),  // cast added
        status_flags);
    radioMod.sendWithReturnFreq(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, msg, sizeof(msg));
}

void txRelPos(GNSSModule::GNSSFix fix, bool isRelativeToBase) {
/*  uint8_t msg[6];
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

  //encodeRelPosMessage(msg, slope.getGliderId(), n_dm, e_dm, d_dm, fixStatus(fix));
  radioMod.sendWithReturnFreq(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, msg, sizeof(msg));
*/
}

void txMsg(uint8_t msgCode, uint8_t d) {
  //uint8_t msg[6];
  //encodeMiscMessage(msg, slope.getGliderId(), d);
  //radioMod.sendWithReturnFreq(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, msg, sizeof(msg));
}