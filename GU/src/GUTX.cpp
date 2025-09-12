// GUTX.cpp
#include <Arduino.h>
#include <RFM69.h> 

#include <RTKF3F.h>
#include "RadioModule.h"
#include "GUTX.h"

extern RadioModule radioMod;

static inline int16_t cm_to_dm_i16(float cm) {
    // round to nearest decimeter and clamp
    long v = lroundf(cm / 10.0f);
    if (v > INT16_MAX) v = INT16_MAX;
    if (v < INT16_MIN) v = INT16_MIN;
    return (int16_t)v;
}

void txMsg(MessageType code) {
    uint8_t msg[1];
	msg[0] = static_cast<uint8_t>(code);
    radioMod.sendWithReturnFreq(NODEID_CD, FREQUENCY_CD, FREQUENCY_CD, msg, sizeof(msg));
}

// Pack N/E/D in centimeters (int32_t, little-endian) and send (12 bytes).
void txRelPos32(const GNSSFix& fix, bool isRelativeToBase) {
    uint8_t msg[6];

    float n_cm = isRelativeToBase ? fix.relNorth : fix.adjNorth;
    float e_cm = isRelativeToBase ? fix.relEast : fix.adjEast;
    float d_cm = isRelativeToBase ? fix.relDown : fix.adjDown;

    int16_t n_dm = cm_to_dm_i16(n_cm);
    int16_t e_dm = cm_to_dm_i16(e_cm);
    int16_t d_dm = cm_to_dm_i16(d_cm);

    // little-endian pack: N,E,D (int16_t each)
    msg[0] = (uint8_t)(n_dm & 0xFF);
    msg[1] = (uint8_t)((n_dm >> 8) & 0xFF);
    msg[2] = (uint8_t)(e_dm & 0xFF);
    msg[3] = (uint8_t)((e_dm >> 8) & 0xFF);
    msg[4] = (uint8_t)(d_dm & 0xFF);
    msg[5] = (uint8_t)((d_dm >> 8) & 0xFF);

    // radioMod is an object, not a pointer; also don't forget the semicolon.
    radioMod.sendWithReturnFreq(NODEID_CD, FREQUENCY_CD, FREQUENCY_CD, msg, sizeof(msg));
}
