// RTCMFwd.h
#pragma once

#ifndef RTCMFWD_H
#define RTCMFWD_H


#include <Arduino.h>
#include <RFM69.h>

bool verifyRadio(RFM69& radio);

void initRTCMForwarder(HardwareSerial* input, RFM69* radio);
void updateRTCMForwarder();

#endif