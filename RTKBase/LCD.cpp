#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "LCD.h"

// OBS: LCD-objektet må være "extern" om det ligger i .ino,
// ellers flytt lcd-definisjonen hit og lag en getter.

extern LiquidCrystal_I2C lcd;

// Faktisk definisjon
String lastlcdLine1 = "";
String lastlcdLine2 = "";

void updateLCD(bool force, String line1, String line2) {
    if (!force && line1 == lastlcdLine1 && line2 == lastlcdLine2) return;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);

    lastlcdLine1 = line1;
    lastlcdLine2 = line2;
}