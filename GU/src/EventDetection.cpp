// EventDetection.cpp
#include <Arduino.h>
#include <math.h>
#include <RTKF3F.h>
#include <Const.h>

extern Slope slope;

static float A_x, A_y;
static float B_x, B_y;
static float slope_dx, slope_dy;
static float slope_nx, slope_ny;
//static bool initialized = false;

void initEventDetection() {
  float slopeDeg = slope.getSlopeAngle();
  float rad = radians(90.0f - slopeDeg);  // 0° = north, increasing clockwise

  // Unit vector in slope direction
  slope_dx = cos(rad);
  slope_dy = sin(rad);

  // Normal vector (perpendicular to slope)
  slope_nx = -slope_dy;
  slope_ny =  slope_dx;

  // Flip direction if A base is on the left
  if (!slope.getABaseLeft()) {
    slope_nx *= -1;
    slope_ny *= -1;
  }

  // Place A and B bases relative to pilot location (0,0)
  A_x = slope_nx * -(SLOPELENGTH / 2);
  A_y = slope_ny * -(SLOPELENGTH / 2);
  B_x = slope_nx *  (SLOPELENGTH / 2);
  B_y = slope_ny *  (SLOPELENGTH / 2);
}

bool checkForCrossingEvent(const GNSSFix& fix, EventCode& event) {
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
