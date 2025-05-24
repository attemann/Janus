//GPSInterface.h
#pragma once

#ifndef GPSINTERFACE_H
#define GPSINTERFACE_H

  #include <RTKF3F.h>

  void initGPS();
  bool readGNSSFix(GNSSFix& fix);

#endif