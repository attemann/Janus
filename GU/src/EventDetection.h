// EventDetection.h
#pragma once

#ifndef EVENTDETECTION_H
#define EVENTDETECTION_H

  #include <RTKF3F.h> 

  void initEventDetection();
  bool checkForCrossingEvent(const GNSSModule::GNSSFix& fix, EventCode& event);

#endif