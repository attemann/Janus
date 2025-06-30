// RTCMForwarder.h
#pragma once

#include <Arduino.h>
#include <RFM69.h>

void initRTCMForwarder(HardwareSerial* input, RFM69* radio, uint8_t targetNode);
void updateRTCMForwarder();
bool verifyRadio(RFM69& radio, const char* label);
