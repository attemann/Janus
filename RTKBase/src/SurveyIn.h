#ifndef SURVEY_IN_H
#define SURVEY_IN_H

// === Function Prototypes ===
void enableStableGPS();
void enableSurveyIn(int surveyTime);
void enableRTCM(int frequency);
int checkFIX(String& fixTitle, int& sats, float& HDOP);

#endif