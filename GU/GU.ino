//GU.ino

#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include "src/GUTX.h"
#include "src/EventDetection.h"
#include "src/GPSInterface.h"
#include "src/Receiver.h"
#include "src/Airborne.h"

RFM69 rxtxRadio;
Slope slope;

GNSSFix lastFix;
uint8_t gliderId=1;

AirborneDetector airborneDetector;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for USB serial ready (dev/debug only)

  Serial.println ("RTKF3F Glider unit starting...");

  Serial2.begin(115200);  // GNSS UART

  // Radio setup
  rxtxRadio.initialize(RF69_868MHZ, NODE_ID, NETWORK_ID);
  rxtxRadio.setHighPower();               // Needed for RFM69HCW
  rxtxRadio.setFrequency(BS_TX_FREQ);     // Default receive frequency for RTCM
  rxtxRadio.setPowerLevel(31);            // Max power level (adjust as needed)

  // Subsystem initializations
  initGPS();
  initEventDetection();

  //enable airborne detector
  airborneDetector.begin();

  // Optional: visual feedback
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println ("RTKF3F Glider unit ready!");
}

void loop() {
  // --- 1. Handle incoming messages from Base Station ---
  if (rxtxRadio.receiveDone()) {
    handleRadioMessage(rxtxRadio.DATA, rxtxRadio.DATALEN);  // e.g., RTCM or MSG_FLIGHTSETTINGS
  }

  // --- 2. Poll GNSS and check for base/safety crossings ---
  GNSSFix fix;

  if (readGNSSFix(fix)) {

    // store last fix
    lastFix=fix;

    EventCode event;

    if (checkForCrossingEvent(fix, event)) {
      txEvent(event, fixStatus(fix));
    }

    airborneDetector.update(fix.relNorth, fix.relEast);
  }

  // Optional: blink LED on valid GNSS fix
  // digitalWrite(LED_BUILTIN, fix.gpsFix ? HIGH : LOW);
}
