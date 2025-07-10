// SurveyIn.cpp

#include <Arduino.h>
#include <RTKF3F.h>
#include "LCD.h"
#include "SurveyIn.h"

// === Monitor fixes ===
int checkFIX(String& fixTitle, int& sats, float& HDOP) {
int fix = 0;
static String incomingLine = "";

    while (serialGPS.available()) {
        char c = serialGPS.read();

        if (c == '\n') {
            incomingLine.trim();

            if (parseGGAStatus(incomingLine, fix, sats, HDOP)) {
                switch (fix) {
                case 0: 
                    fixTitle = "Invalid";
                    break;
                case 1:  
                    fixTitle = "GPS fix"; 
                    break;
                case 2:  
                    fixTitle = "DGPS fix"; 
                    break;
                case 3:  
                    fixTitle = "PPS fix"; 
                    break;
                case 4:  
                    fixTitle = "RTK fixed"; 
                case 5:  
                    fixTitle = "RTK float"; 
                    break;
                case 6:  
                    fixTitle = "DeadRecko"; 
                    break;
                case 7:  
                    fixTitle = "Manual"; 
                    break;
                case 8:  
                    fixTitle = "Simulator"; 
                    break;
                default: fixTitle = "Unknown"; 
                    break;
                }
            }

            incomingLine = "";
        }
        else {
            incomingLine += c;
        }
    }

    return fix;
}