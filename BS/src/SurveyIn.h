#ifndef SURVEY_IN_H
#define SURVEY_IN_H

// === Function Prototypes ===
void configureSurveyIn();
bool checkSurveyStatus(unsigned long &durationMs, float &accuracyM);
//bool parseSurveyStatus(String line, unsigned long& durMs, float& accM);
void configureRTCM();

#endif