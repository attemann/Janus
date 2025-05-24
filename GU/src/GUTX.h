// GUTX.h
#pragma once

#ifndef GUTX_H
#define GUTX_H

  #include <Arduino.h>
  //#include <RFM69.h>
  #include <RTKF3F.h>

  // Sends a 5-byte event packet from GU to BS
  // Parameters:
  //   glider_id     : 0–255 unique ID of glider
  //   event         : EventCode enum (4 bits)
  //   status_flags  : Status bitfield (use STATUS_* constants from EventFormat.h)
  void txEvent(EventCode event, uint8_t status_flags);
  void txRelPos(GNSSFix fix, bool isRelativeToBase);
  void txMsg(uint8_t msgCode);
  uint8_t fixStatus(GNSSFix fix);

#endif
