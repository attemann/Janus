// Airborne.cpp

#include <sys/_stdint.h>
//#include <Const.h>
#include "Airborne.h"
#include <math.h>
#include "GUTX.h"  // Assumes GU_sendEvent(uint8_t event) is declared here

AirborneDetector::AirborneDetector()
  : bufferIndex(0), airborne(false),
    thresholdAirborne(THRESHOLD_AIRBORNE), thresholdLanded(THRESHOLD_LANDED),
    lastSampleTime(0), sampleInterval(100)
{
  for (int i = 0; i < DETECTOR_BUFFER_SIZE; ++i) {
    posBuffer[i] = {0.0f, 0.0f};
  }
}

void AirborneDetector::begin(unsigned long intervalMs) {
  sampleInterval = intervalMs;
  lastSampleTime = millis();
}

void AirborneDetector::setThresholds(float airborneDist, float landedDist) {
  thresholdAirborne = airborneDist;
  thresholdLanded = landedDist;
}

bool AirborneDetector::isCurrentlyAirborne() const {
  return airborne;
}

void AirborneDetector::update(float north, float east) {
  unsigned long now = millis();
  if (now - lastSampleTime < sampleInterval) return;
  lastSampleTime = now;

  // Store current position
  posBuffer[bufferIndex] = {north, east};
  int pastIndex = (bufferIndex + 1) % DETECTOR_BUFFER_SIZE;
  bufferIndex = pastIndex;

  // Compare with position 3 seconds ago
  Position& current = posBuffer[(bufferIndex + DETECTOR_BUFFER_SIZE - 1) % DETECTOR_BUFFER_SIZE];
  Position& past = posBuffer[pastIndex];

  float dn = current.north - past.north;
  float de = current.east - past.east;
  float dist = sqrt(dn * dn + de * de);

  if (!airborne && dist > thresholdAirborne) {
    airborne = true;
    //txEvent(EventCode::EVT_AIRBORNE,0);
  } else if (airborne && dist < thresholdLanded) {
    airborne = false;
    //txEvent(EventCode::EVT_LANDED,0);
  }
}