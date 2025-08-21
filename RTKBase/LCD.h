#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//LCD i2C
//#define LCD_SDA       21
//#define LCD_SCL       22

class LCDManager {
public:
    LCDManager(LiquidCrystal_I2C& lcdRef) : lcd(lcdRef) {}

    void begin() {
        lcd.init();
        lcd.backlight();
        lcd.clear();
        prevLine[0] = String(16, ' ');
        prevLine[1] = String(16, ' ');
    }

    void setLine(uint8_t line, const String& text) {
        if (line > 1) return;

        String padded = text;
        if (padded.length() > 16)
            padded = padded.substring(0, 16);
        while (padded.length() < 16)
            padded += ' ';

        if (prevLine[line] != padded) {
            lcd.setCursor(0, line);
            lcd.print(padded);
            prevLine[line] = padded;
        }
    }

    void clear() {
        lcd.clear();
        prevLine[0] = String(16, ' ');
        prevLine[1] = String(16, ' ');
    }

private:
    LiquidCrystal_I2C& lcd;
    String prevLine[2];
};
#endif