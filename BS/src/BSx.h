#pragma once

#ifndef BS_H
#define BS_H

#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>

class StateMachine;
class Slope;

extern Slope slope;
extern StateMachine taskStateMachine;

void debugTaskStates(int sim);
void debugButtons(int sim);
String generateStars(int count);
bool verifyRadio(RFM69& radio, const char* label);

#endif
