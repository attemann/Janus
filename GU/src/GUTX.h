// GUTX.h
#pragma once

#ifndef GUTX_H
#define GUTX_H

#include <Arduino.h>
#include <RTKF3F.h>

void txRelPos32(const GNSSFix& fix, bool isRelativeToBase);
void txMsg(MessageType code);


#endif
