/*
 Name:		RTKBaseTask.ino
 Created:	8/20/2025 8:09:58 AM
 Author:	JanOlavEndrerud
*/
#include "DisplayTask.h"

void setup() {
    Serial.begin(115200);
    startDisplayTask();  // ← this starts LCD updates

    sendToDisplay("Booting", "RTKBase v1.0");
}

void loop() {
    sendToDisplayIfChanged("Surveying", "10 sec left", 3000);

    // Other work...
}