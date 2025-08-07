// EventDetection.cpp
#include <Arduino.h>
#include <math.h>
#include <RTKF3F.h>
//#include <Const.h>

extern Slope slope;

static float A_x, A_y;
static float B_x, B_y;
static float slope_dx, slope_dy;
static float slope_nx, slope_ny;
//static bool initialized = false;

bool checkCrossing(const GNSSModule::GNSSFix& fix, EventCode& event) {
  static bool wasBetweenBases = false;

  float px = fix.adjEast;
  float py = fix.adjNorth;

  float dA = (px - A_x) * slope_nx + (py - A_y) * slope_ny;
  float dB = (px - B_x) * slope_nx + (py - B_y) * slope_ny;

  bool between = (dA > 0 && dB < 0);

  if (!wasBetweenBases && between) {
    wasBetweenBases = true;
    event = (dA <= 0) ? EventCode::EVT_CROSS_A_IN : EventCode::EVT_CROSS_B_IN;
    return true;
  }

  if (wasBetweenBases && !between) {
    wasBetweenBases = false;
    event = (dA <= 0) ? EventCode::EVT_CROSS_A_OUT : EventCode::EVT_CROSS_B_OUT;
    return true;
  }

  return false;
}
