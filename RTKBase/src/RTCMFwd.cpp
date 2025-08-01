// RTCMFwd.cpp - BS module for forwarding RTCM3 corrections over RFM69
#include <RTKF3F.h>
#include "RTCMFwd.h"

#define REG_VERSION 0x10
#define EXPECTED_RFM69_VERSION 0x24

extern HardwareSerial SerialGNSS;
extern RFM69 radio;








