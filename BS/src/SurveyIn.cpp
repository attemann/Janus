// SurveyIn.cpp

#include <Arduino.h>
#include <RTKF3F.h>
#include  "SurveyIn.h"

#define SIM_SURVEYIN

extern HardwareSerial& serialGPS;  // GNSS input

// === Timing and thresholds ===
unsigned long surveyStartTime = 0;
const unsigned long surveyMinDuration = 120000; // 2 minutes
const float requiredAccuracy = 2.0; // meters

// === Configure GNSS for Survey-In Mode ===
void configureSurveyIn() {
  serialGPS.println("CFG-RTK-MODE BASE");
  serialGPS.println("CFG-SURVEYIN-ENABLE TRUE");
  serialGPS.println("CFG-SURVEYIN-MIN-DURATION 120");
  serialGPS.println("CFG-SURVEYIN-POS-ACCURACY 2.0");
  serialGPS.println("CFG-CFG SAVE");
  surveyStartTime = millis();
}

// === Parse line like: "$SVIN,DUR=130,ACC=1.5" ===
bool parseSurveyStatus(String line, unsigned long& durMs, float& accM) {
  if (!line.startsWith("$SVIN")) return false;
  int durIndex = line.indexOf("DUR=");
  int accIndex = line.indexOf("ACC=");
  if (durIndex == -1 || accIndex == -1) return false;

  durMs = line.substring(durIndex + 4, line.indexOf(',', durIndex)).toInt() * 1000;
  accM = line.substring(accIndex + 4).toFloat();
  Serial.printf("Survey: %lu ms, %.2f m\n", durMs, accM);
  return true;
}

// === Monitor Survey-In Completion ===
bool checkSurveyStatus(unsigned long &durationMs, float &accuracyM) {
  static String incomingLine;

  #ifdef SIM_SURVEYIN
        durationMs=random(5000); // 5 sec
        accuracyM=random(100)/100; //100 cm=1m
        return true;
  #endif

  while (serialGPS.available()) {
    char c = serialGPS.read();
    if (c == '\n') {
      if (parseSurveyStatus(incomingLine, durationMs, accuracyM)) {


        if (durationMs >= surveyMinDuration && accuracyM <= requiredAccuracy) {
          Serial.println("Survey-in complete!");
          configureRTCM();
          return true;
        }
      }
      incomingLine = "";
    } else {
      incomingLine += c;
    }
  }
  return false;
}

// === Switch GNSS to Fixed Base after survey ===
void configureRTCM() {
  serialGPS.println("CFG-SURVEYIN-ENABLE FALSE");
  serialGPS.println("CFG-MSG UART2 RTCM3 1005 1");
  serialGPS.println("CFG-MSG UART2 RTCM3 1074 1");
  serialGPS.println("CFG-MSG UART2 RTCM3 1084 1");
  serialGPS.println("CFG-MSG UART2 RTCM3 1094 1");
  serialGPS.println("CFG-MSG UART2 RTCM3 1124 1");
  serialGPS.println("CFG-CFG SAVE");
}