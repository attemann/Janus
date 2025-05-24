// SendCmd.h
#pragma once

#include <RFM69.h>
#include <RTKF3F.h>

void sendCommand(const Slope& slope, uint8_t commandId, const uint8_t* params = nullptr, uint8_t paramLen = 0);
void sendFlightSettings(const Slope& slope);
void sendPosRequest(const Slope& slope);