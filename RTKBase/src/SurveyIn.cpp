// SurveyIn.cpp

#include <Arduino.h>
#include <RTKF3F.h>
#include "LCD.h"
#include "SurveyIn.h"

extern HardwareSerial& serialGPS;  // GNSS input

bool sendCommandWaitOK(const String& cmd, uint32_t timeout = 1000) {
    serialGPS.println(cmd);
    updateLCD(true, "Sending", ">"+cmd);

    Serial.print("Sending ");
    Serial.println(cmd);

    uint32_t start = millis();
    String response = "";

    while (millis() - start < timeout) {
        while (serialGPS.available()) {
            char c = serialGPS.read();
            response += c;
            if (response.endsWith("OK\r\n")) return true;
            if (response.endsWith("ERROR\r\n")) return false;
        }
    }
    return false;  // timeout
}

// === Configure GNSS for getting GPS stable fixes ===
void enableStableGPS() {
    uint32_t timeout = 1000;
    sendCommandWaitOK("unlog", timeout);
    sendCommandWaitOK("mode rover", timeout);
    sendCommandWaitOK("gpgga 1", timeout);
}


// === Configure GNSS for Survey-In Mode ===
void enableSurveyIn(int surveyTime) {
    uint32_t timeout = 1000;
    sendCommandWaitOK("unlog", timeout);
    sendCommandWaitOK("gpgga 1", timeout);
    sendCommandWaitOK("mode base time " + String(surveyTime), timeout);
}

void enableRTCM(int frequency) {
    uint32_t timeout = 1000;
    sendCommandWaitOK("rtcm1005 " + String(frequency), timeout);
    sendCommandWaitOK("rtcm1074 " + String(frequency), timeout);
    sendCommandWaitOK("rtcm1084 " + String(frequency), timeout);
    sendCommandWaitOK("rtcm1094 " + String(frequency), timeout);
    sendCommandWaitOK("rtcm1124 " + String(frequency), timeout);
    sendCommandWaitOK("rtcm1230 " + String(frequency), timeout);
}

bool parseGGAStatus(const String& line,int& fix, int& sats, float& HDOP) {

    Serial.println(line);

    if (!line.startsWith("$GPGGA") && !line.startsWith("$GNGGA")) return false;

    int fieldIndex = 0;
    int lastPos = 0;
    int nextPos = 0;
    String fields[15];  // GGA typically has up to 15 fields

    // Split line into fields
    while ((nextPos = line.indexOf(',', lastPos)) != -1 && fieldIndex < 15) {
        fields[fieldIndex++] = line.substring(lastPos, nextPos);
        lastPos = nextPos + 1;
    }
    fields[fieldIndex] = line.substring(lastPos);  // Add last field

    if (fieldIndex < 7) return false;  // Not enough fields

    fix = fields[6].toInt();
    sats = fields[7].toInt();
	HDOP = fields[8].toFloat();

    //Serial.printf("Parsed GGA: fix quality = %d, satellites = %d, HDOP=%f4.2\n", fix, sats, HDOP);
    return true;
}

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