#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// Disse er definert i LCD.cpp
extern String lastlcdLine1;
extern String lastlcdLine2;

// Prototypen
void updateLCD(bool force = false, String line1 = "", String line2 = "");

#endif