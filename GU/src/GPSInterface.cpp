//GPSInterface.cpp
#include <Arduino.h>
#include <RTKF3F.h>
#include "GPSInterface.h"

// UBX sync bytes and message details
#define UBX_SYNC1            0xB5
#define UBX_SYNC2            0x62
#define UBX_CLASS_NAV        0x01
#define UBX_ID_RELPOSNED     0x3C
#define UBX_RELPOSNED_LEN    40

extern Slope slope;

void initGPS() {
  // GNSS UART must already be initialized (e.g., Serial2.begin())

  // Optional: Clear old config
  Serial2.println("$CFG-CFG=RESET");
  delay(50);

  // Set ROVER mode
  Serial2.println("$CFG-RTK-MODE=ROVER");
  delay(20);

  // Dynamic model: Airborne <2g
  Serial2.println("$CFG-DYNMODEL=AIRBORNE_2G");
  delay(20);

  // 25 Hz output rate
  Serial2.println("$CFG-RATE=25");
  delay(20);

  // Enable NAV-RELPOSNED output
  Serial2.println("$CFG-MSG=UART1,NAV-RELPOSNED,25");
  delay(20);

  // Disable NMEA (optional)
  Serial2.println("$CFG-MSG=UART1,GGA,0");
  Serial2.println("$CFG-MSG=UART1,RMC,0");
  Serial2.println("$CFG-MSG=UART1,GSV,0");
  Serial2.println("$CFG-MSG=UART1,GSA,0");
  delay(20);

  // Save config
  Serial2.println("$CFG-CFG=SAVE");
  delay(50);
}

bool readGNSSFix(GNSSFix& fix) {
  static uint8_t buffer[48];

  while (Serial2.available() >= 48) {
    if (Serial2.read() == UBX_SYNC1 && Serial2.peek() == UBX_SYNC2) {
      Serial2.read(); // Consume SYNC2

      if (Serial2.read() == UBX_CLASS_NAV && Serial2.read() == UBX_ID_RELPOSNED) {
        uint16_t length = Serial2.read() | (Serial2.read() << 8);
        if (length != UBX_RELPOSNED_LEN) return false;

        Serial2.readBytes(buffer, UBX_RELPOSNED_LEN);

        // Extract relative position
        int32_t relN    = *(int32_t*)(buffer + 4);
        int32_t relE    = *(int32_t*)(buffer + 8);
        int32_t relD    = *(int32_t*)(buffer + 12);
        int8_t  relN_hp = buffer[16];
        int8_t  relE_hp = buffer[17];
        int8_t  relD_hp = buffer[18];

        // Extract flags
        uint32_t flags = *(uint32_t*)(buffer + 20);

        // Compute full relative position in meters
        fix.relNorth = (relN + relN_hp * 0.1f) / 100.0f;
        fix.relEast  = (relE + relE_hp * 0.1f) / 100.0f;
        fix.relDown  = (relD + relD_hp * 0.1f) / 100.0f;

        int n,e,d;
        slope.getPilotOffsetNED(n,e,d);
        fix.adjNorth = fix.relNorth + n;
        fix.adjEast  = fix.relEast  + e;
        fix.adjDown  = fix.relDown  + d;

        // Decode flags based on UBX-RELPOPNED documentation
        fix.gpsFix   = flags & (1 << 0);                    // GNSS fix valid
        fix.diffUsed = flags & (1 << 1);                    // Differential corrections used
        uint8_t relMode = (flags >> 2) & 0x03;
        fix.rtkFix   = (relMode == 0x02);                   // 0x02 = RTK fixed
        fix.rtkFloat = (relMode == 0x01);                   // 0x01 = RTK float

        return true;
      }
    }
  }

  return false;
}
